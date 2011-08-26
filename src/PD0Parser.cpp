#include <dvl_teledyne/PD0Parser.hpp>
#include <dvl_teledyne/PD0Raw.hpp>
#include <endian.h>
#include <stdexcept>
#include <base/float.h>

using namespace dvl_teledyne;

int PD0Parser::extractPacket(uint8_t const* buffer, size_t size, size_t max_size) const
{
    // Look for the first "thing" that looks like a valid header start
    size_t packet_start;
    for (packet_start = 0; packet_start < size; ++packet_start)
    {
        if (buffer[packet_start] == raw::Header::ID)
            break;
    }

    if (packet_start == size)
    {
        // no packet start in buffer, discard everything
        return -size;
    }
    else if (packet_start)
    {
        // realign the IODriver buffer to the start of the candidate packet
        return -(packet_start - 1);
    }
    else if (buffer[1] != raw::Header::DATA_SOURCE_ID)
    {
        // not actually a packet. Drop the first two bytes and let IODriver call
        // us back
        return -2; 
    }
    else if (size < sizeof(raw::Header))
    {
        // cannot parse the rest of the header yet ... wait for new data
        return 0;
    }

    raw::Header const& header = *reinterpret_cast<raw::Header const*>(buffer);
    // This is the size EXCLUDING CHECKSUM
    uint16_t ensemble_size = le16toh(header.size);
    uint16_t total_size = ensemble_size + 2;
    if (max_size < total_size)
    {
        // Assume that this packet is not valid as it has a size too big. Drop
        // the first two bytes, and let IODriver call us back to parse the rest
        // of the buffer
        return -2;
    }
    else if (size < total_size)
    {
        // Have to wait for new data (we don't have a full packet yet)
        return 0;
    }

    uint16_t checksum = 0;
    for (int i = 0; i < ensemble_size; ++i)
        checksum += buffer[i];

    uint16_t msg_checksum = le16toh(*reinterpret_cast<uint16_t const*>(buffer + ensemble_size));
    if (checksum == msg_checksum)
        return ensemble_size + 2;
    else
    {
        // Not a valid message. Drop the message IDs and let IODriver call us
        // back to find the start of the actual packet
        return -2;
    }

    uint32_t offsets[256];
    for (int i = 0; i < header.msg_count; ++i)
        offsets[i] = le16toh(header.offsets[i]);

    // Validate sizes
    uint32_t expected_offset = 0;
    for (int i = 0; i < header.msg_count; ++i)
    {
        if (expected_offset != 0 && offsets[i] != expected_offset)
            return -2;

        uint32_t msg_id   = le16toh(*reinterpret_cast<uint16_t const*>(buffer + offsets[i]));
        uint32_t msg_size = getSizeOfMessage(msg_id);
        if (msg_size != 0)
            expected_offset = offsets[i] + msg_size;
    }
}

int PD0Parser::getSizeOfMessage(uint16_t msg_id) const
{
    return 0;
}

void PD0Parser::parseEnsemble(uint8_t const* buffer, size_t size)
{
    // Validate the message sizes
    raw::Header const& header = *reinterpret_cast<raw::Header const*>(buffer);

    uint32_t offsets[256];
    for (int i = 0; i < header.msg_count; ++i)
        offsets[i] = le16toh(header.offsets[i]);

    invalidateCellReadings();
    for (int i = 0; i < header.msg_count; ++i)
        parseMessage(buffer + offsets[i], size - offsets[i]);
}

void PD0Parser::invalidateCellReadings()
{
    for (int i = 0; i < mAcqConf.cell_count; ++i)
    {
        for (int beam = 0; beam < 4; ++beam)
        {
            mCellReadings.readings[i].velocity[beam]  = base::unset<float>();
            mCellReadings.readings[i].correlation[beam] = base::unset<float>();
            mCellReadings.readings[i].intensity[beam] = base::unset<float>();
            mCellReadings.readings[i].quality[beam]   = base::unset<float>();
        }
    }
}

void PD0Parser::parseMessage(uint8_t const* buffer, size_t size)
{
    uint16_t msg_id   = le16toh(*reinterpret_cast<uint16_t const*>(buffer));
    switch(msg_id)
    {
    case raw::FixedLeader::ID:
        parseFixedLeader(buffer, size);
        if (mCellReadings.readings.size() != mAcqConf.cell_count)
        {
            mCellReadings.readings.resize(mAcqConf.cell_count);
            invalidateCellReadings();
        }
        break;
    case raw::VariableLeader::ID:
        parseVariableLeader(buffer, size);
        break;
    case raw::VelocityMessage::ID:
        mCellReadings.time = mDeviceState.time;
        parseVelocityReadings(buffer, size);
        break;
    case raw::CorrelationMessage::ID:
        mCellReadings.time = mDeviceState.time;
        parseCorrelationReadings(buffer, size);
        break;
    case raw::IntensityMessage::ID:
        mCellReadings.time = mDeviceState.time;
        parseIntensityReadings(buffer, size);
        break;
    case raw::QualityMessage::ID:
        mCellReadings.time = mDeviceState.time;
        parseQualityReadings(buffer, size);
        break;
    case raw::BottomTrackingMessage::ID:
        mBottomTracking.time = mDeviceState.time;
        parseBottomTrackingReadings(buffer, size);
        break;
    }
}

static Sensors parseSensors(uint8_t bitfield)
{
    Sensors result;
    result.calculates_speed_of_sound = bitfield & raw::PD0_CALCULATE_SPEED_OF_SOUND;
    result.depth             = bitfield & raw::PD0_DEPTH_SENSOR;
    result.yaw           = bitfield & raw::PD0_YAW_SENSOR;
    result.pitch             = bitfield & raw::PD0_PITCH_SENSOR;
    result.roll              = bitfield & raw::PD0_ROLL_SENSOR;
    result.salinity          = bitfield & raw::PD0_SALINITY_SENSOR;
    result.temperature       = bitfield & raw::PD0_TEMPERATURE_SENSOR;
    return result;
}

void PD0Parser::parseFixedLeader(uint8_t const* buffer, size_t size)
{
    if (size < sizeof(raw::FixedLeader))
        throw std::runtime_error("parseFixedLeader: error in stream, buffer size too small");

    raw::FixedLeader const& leader = *reinterpret_cast<raw::FixedLeader const*>(buffer);
    mDeviceInfo.fw_version           = leader.fw_version;
    mDeviceInfo.fw_revision          = leader.fw_revision;
    mDeviceInfo.cpu_board_serno      = le64toh(leader.cpu_board_serno);
    mDeviceInfo.system_configuration = le16toh(leader.system_configuration);
    mDeviceInfo.beam_count           = leader.beam_count;
    mDeviceInfo.available_sensors    = parseSensors(leader.available_sensors);

    mAcqConf.lag_duration                  = leader.lag_duration;
    mAcqConf.cell_count                    = leader.cell_count;
    mAcqConf.pings_per_ensemble            = le16toh(leader.pings_per_ensemble);
    mAcqConf.cell_length                   = 0.01f * le16toh(leader.cell_length);
    mAcqConf.blank_after_transmit_distance = 0.01f * le16toh(leader.blank_after_transmit_distance);
    mAcqConf.profiling_mode                = leader.profiling_mode;
    mAcqConf.low_correlation_threshold     = leader.low_correlation_threshold;
    mAcqConf.code_repetition_count         = leader.code_repetition_count;
    mAcqConf.water_layer_min_ping_threshold = 1.0f / 255 * leader.water_layer_min_ping_threshold;
    mAcqConf.water_layer_velocity_threshold = 0.001 * le16toh(leader.water_layer_velocity_threshold);
    uint64_t milliseconds =
        static_cast<uint32_t>(leader.time_between_ping_groups_min * 60 * 1000) +
        static_cast<uint32_t>(leader.time_between_ping_groups_sec * 1000) +
        static_cast<uint32_t>(leader.time_between_ping_groups_hundredth * 10);
    mAcqConf.time_between_ping_groups = base::Time::fromMicroseconds(milliseconds * 1000);
    mAcqConf.yaw_alignment     = M_PI / 180 * 0.01 * le16toh(leader.yaw_alignment);
    mAcqConf.yaw_bias          = M_PI / 180 * 0.01 * le16toh(leader.yaw_bias);
    mAcqConf.first_cell_distance   = 0.01f * le16toh(leader.first_cell_distance);
    mAcqConf.transmit_pulse_length  = 0.01f * le16toh(leader.transmit_pulse_length);
    mAcqConf.water_layer_start      = leader.water_layer_start;
    mAcqConf.water_layer_end        = leader.water_layer_end;
    mAcqConf.false_target_threshold = leader.false_target_threshold;
    mAcqConf.low_latency_trigger    = leader.low_latency_trigger;
    mAcqConf.transmit_lag_distance  = 0.01f * le16toh(leader.transmit_lag_distance);
    mAcqConf.narrow_bandwidth_mode  = le16toh(leader.narrow_bandwidth_mode);
    mAcqConf.used_sensors           = parseSensors(leader.used_sensors);

    uint8_t mode = leader.coordinate_transformation_mode;
    switch(mode & raw::PD0_COORDINATE_SYSTEM_MASK)
    {
    case raw::PD0_COORD_BEAM:
        mOutputConf.coordinate_system = BEAM;
        break;
    case raw::PD0_COORD_INSTRUMENT:
        mOutputConf.coordinate_system = INSTRUMENT;
        break;
    case raw::PD0_COORD_SHIP:
        mOutputConf.coordinate_system = SHIP;
        break;
    case raw::PD0_COORD_EARTH:
        mOutputConf.coordinate_system = EARTH;
        break;
    default: throw std::runtime_error("unexpected value for coordinate_transformation_mode & raw::PD0_COORDINATE_SYSTEM_MASK");
    }
    mOutputConf.use_attitude = mode & raw::PD0_USE_ATTITUDE;
    mOutputConf.use_3beam_solution = mode & raw::PD0_USE_3BEAM_SOLUTION;
    mOutputConf.use_binary_mapping = mode & raw::PD0_USE_BINARY_MAPPING;
}

void PD0Parser::parseVariableLeader(uint8_t const* buffer, size_t size)
{
    if (size < sizeof(raw::VariableLeader))
        throw std::runtime_error("parseVariableLeader: error in stream, buffer size too small");

    raw::VariableLeader const& msg = *reinterpret_cast<raw::VariableLeader const*>(buffer);

    mDeviceState.seq  = static_cast<uint32_t>(msg.seq_low) + (static_cast<uint32_t>(msg.seq_high) << 16);
    {
        // Get current UTC time broken down in day, h, m, s
        time_t utc_epoch = ::time(NULL);
        tm utc_hms;
        gmtime_r(&utc_epoch, &utc_hms);

        // Replace hours minutes and seconds by the values reported by the
        // device
        utc_hms.tm_mon   = msg.rtc_month;
        utc_hms.tm_hour  = msg.rtc_hour;
        utc_hms.tm_min   = msg.rtc_min;
        utc_hms.tm_sec   = msg.rtc_sec;

        // And convert it back to seconds since epoch
        time_t since_epoch = timegm(&utc_hms);
        mDeviceState.time = base::Time::fromSeconds(since_epoch, static_cast<uint64_t>(msg.rtc_hundredth) * 10000);
    }

    mDeviceState.orientation =
        Eigen::AngleAxisd(M_PI / 180 * 0.01 * le16toh(msg.roll),    Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(M_PI / 180 * 0.01 * le16toh(msg.pitch),   Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(M_PI / 180 * 0.01 * le16toh(msg.yaw), Eigen::Vector3d::UnitZ());
    mDeviceState.stddev_orientation[0] = M_PI / 180.0f * msg.stddev_yaw;
    mDeviceState.stddev_orientation[1] = M_PI / 180.0f * 0.1f * msg.stddev_pitch;
    mDeviceState.stddev_orientation[2] = M_PI / 180.0f * 0.1f * msg.stddev_roll;
    mDeviceState.speed_of_sound = 1.0f * le16toh(msg.speed_of_sound);
    mDeviceState.salinity    = 1e-3f * le16toh(msg.salinity_at_transducer);
    mDeviceState.depth       = 1e-1f * le16toh(msg.depth_of_transducer);
    mDeviceState.temperature = 1e-2f * le16toh(msg.temperature_at_transducer);
    mDeviceState.pressure    = 100.0f + 10.0f * le32toh(msg.pressure_at_transducer);
    mDeviceState.pressure_variance = 100.0f + 10.0f * le32toh(msg.pressure_variance_at_transducer);
    uint64_t milliseconds =
        static_cast<uint32_t>(msg.min_preping_wait_duration_min * 60 * 1000) +
        static_cast<uint32_t>(msg.min_preping_wait_duration_sec * 1000) +
        static_cast<uint32_t>(msg.min_preping_wait_duration_hundredth * 10);
    mDeviceState.min_preping_wait = base::Time::fromMicroseconds(milliseconds * 1000);

    for (int i = 0; i < 8; ++i)
        mDeviceState.adc_channels[i] = msg.adc_channels[i];
}

void PD0Parser::parseVelocityReadings(uint8_t const* buffer, size_t size)
{
    if (size < sizeof(raw::VelocityMessage) + mAcqConf.cell_count * sizeof(raw::CellVelocity))
        throw std::runtime_error("parseVelocityReadings: buffer size too small");

    raw::VelocityMessage const& msg = *reinterpret_cast<raw::VelocityMessage const*>(buffer);
    for (int cell_idx = 0; cell_idx < mAcqConf.cell_count; ++cell_idx)
    {
        // This is pre-sized as soon as we know the number of cells in the
        // acquisition process
        CellReading& cell = mCellReadings.readings[cell_idx];
        
        for (int beam_idx = 0; beam_idx < 4; ++beam_idx)
        {
            int16_t value = le16toh(msg.velocities[beam_idx].velocity[beam_idx]);
            if (value == -32768)
                cell.velocity[beam_idx] = base::unknown<float>();
            else
                cell.velocity[beam_idx] = 1e-3f * value;
        }
    }
}

void PD0Parser::parseCorrelationReadings(uint8_t const* buffer, size_t size)
{
    if (size < sizeof(raw::CorrelationMessage) + mAcqConf.cell_count * sizeof(raw::CellCorrelation))
        throw std::runtime_error("parseCorrelationReadings: buffer size too small");

    raw::CorrelationMessage const& msg = *reinterpret_cast<raw::CorrelationMessage const*>(buffer);
    for (int cell_idx = 0; cell_idx < mAcqConf.cell_count; ++cell_idx)
    {
        // This is pre-sized as soon as we know the number of cells in the
        // acquisition process
        CellReading& cell = mCellReadings.readings[cell_idx];
        
        for (int beam_idx = 0; beam_idx < 4; ++beam_idx)
        {
            uint8_t value = msg.correlations[beam_idx].correlation[beam_idx];
            cell.correlation[beam_idx] = 1.0f / 255 * value;
        }
    }
}

void PD0Parser::parseIntensityReadings(uint8_t const* buffer, size_t size)
{
    if (size < sizeof(raw::IntensityMessage) + mAcqConf.cell_count * sizeof(raw::CellIntensity))
        throw std::runtime_error("parseIntensityReadings: buffer size too small");

    raw::IntensityMessage const& msg = *reinterpret_cast<raw::IntensityMessage const*>(buffer);
    for (int cell_idx = 0; cell_idx < mAcqConf.cell_count; ++cell_idx)
    {
        // This is pre-sized as soon as we know the number of cells in the
        // acquisition process
        CellReading& cell = mCellReadings.readings[cell_idx];
        
        for (int beam_idx = 0; beam_idx < 4; ++beam_idx)
        {
            uint8_t value = msg.intensities[beam_idx].intensity[beam_idx];
            cell.intensity[beam_idx] = 0.45 * value;
        }
    }
}

void PD0Parser::parseQualityReadings(uint8_t const* buffer, size_t size)
{
    if (size < sizeof(raw::QualityMessage) + mAcqConf.cell_count * sizeof(raw::CellQuality))
        throw std::runtime_error("parseQualityReadings: buffer size too small");

    raw::QualityMessage const& msg = *reinterpret_cast<raw::QualityMessage const*>(buffer);
    for (int cell_idx = 0; cell_idx < mAcqConf.cell_count; ++cell_idx)
    {
        // This is pre-sized as soon as we know the number of cells in the
        // acquisition process
        CellReading& cell = mCellReadings.readings[cell_idx];
        
        for (int beam_idx = 0; beam_idx < 4; ++beam_idx)
        {
            uint8_t value = msg.quality[beam_idx].quality[beam_idx];
            cell.quality[beam_idx] = 1.0f / 255 * value;
        }
    }
}

void PD0Parser::parseBottomTrackingReadings(uint8_t const* buffer, size_t size)
{
    if (size < sizeof(raw::BottomTrackingMessage))
        throw std::runtime_error("parseBottomTrackingReadings: buffer size too small");

    raw::BottomTrackingMessage const& msg = *reinterpret_cast<raw::BottomTrackingMessage const*>(buffer);

    mBottomTrackingConf.ping_per_ensemble = le16toh(msg.bottom_ping_per_ensemble);
    mBottomTrackingConf.delay_before_reacquiring = le16toh(msg.bottom_delay_before_reacquiring);
    mBottomTrackingConf.correlation_threshold = 1.0f / 255 * msg.bottom_correlation_threshold;
    mBottomTrackingConf.evaluation_threshold  = 1.0f / 255 * msg.bottom_evaluation_threshold;
    mBottomTrackingConf.good_ping_threshold = 0.01f * msg.bottom_good_ping_threshold;
    mBottomTrackingConf.mode = msg.bottom_mode;
    mBottomTrackingConf.max_velocity_error = 1e-3f * le16toh(msg.bottom_max_velocity_error);
    mBottomTrackingConf.max_tracking_depth = 1e-1f * le16toh(msg.max_tracking_depth);

    mBottomTracking.time = mDeviceState.time;
    for (int beam = 0; beam < 4; ++beam)
    {
        uint32_t value = static_cast<uint32_t>(le16toh(msg.bottom_range_low[beam])) +
            (static_cast<uint32_t>(msg.bottom_range_high[beam]) << 16);
        if (value)
            mBottomTracking.range[beam]           = 1e-2f * value;
        else
            mBottomTracking.range[beam]           = base::unknown<float>();

        mBottomTracking.velocity[beam]        = 1e-3f * le16toh(msg.bottom_velocity[beam]);
        mBottomTracking.correlation[beam]     = 1.0f / 255 * msg.bottom_correlation[beam];
        mBottomTracking.evaluation[beam]      = 1.0f / 255 * msg.bottom_evaluation[beam];
        mBottomTracking.good_ping_ratio[beam] = 1.0f / 255 * msg.bottom_good_ping_ratio[beam];
        mBottomTracking.rssi[beam]            = 0.45f * msg.rssi[beam];
    }
}

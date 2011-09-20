#include <dvl_teledyne/PD0Parser.hpp>
#include <dvl_teledyne/PD0Raw.hpp>
#include <endian.h>
#include <stdexcept>
#include <base/float.h>

#include <boost/lexical_cast.hpp>
#include <string>
using boost::lexical_cast;
using std::string;

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
        return -packet_start;
    }
    else if (size > 1 && buffer[1] != raw::Header::DATA_SOURCE_ID)
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
    if (max_size && max_size < total_size)
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
    if (checksum != msg_checksum)
    {
        // Not a valid message. Drop the message IDs and let IODriver call us
        // back to find the start of the actual packet
        return -2;
    }

    if (sizeof(raw::Header) + header.msg_count * 2 > ensemble_size)
        return -2;
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
    return total_size;
}

int PD0Parser::getSizeOfMessage(uint16_t msg_id) const
{
    return 0;
}

void PD0Parser::parseEnsemble(uint8_t const* buffer, size_t size)
{
    // Validate the message sizes
    raw::Header const& header = *reinterpret_cast<raw::Header const*>(buffer);

    if (sizeof(raw::Header) + header.msg_count * 2 > size)
        throw std::runtime_error("not enough bytes for " + lexical_cast<string>((int)header.msg_count) + " messages");

    uint32_t offsets[256];
    for (int i = 0; i < header.msg_count; ++i)
        offsets[i] = le16toh(header.offsets[i]);

    invalidateCellReadings();
    for (int i = 0; i < header.msg_count; ++i)
        parseMessage(buffer + offsets[i], size - offsets[i]);
}

void PD0Parser::invalidateCellReadings()
{
    for (size_t i = 0; i < cellReadings.readings.size(); ++i)
    {
        for (int beam = 0; beam < 4; ++beam)
        {
            cellReadings.readings[i].velocity[beam]  = base::unset<float>();
            cellReadings.readings[i].correlation[beam] = base::unset<float>();
            cellReadings.readings[i].intensity[beam] = base::unset<float>();
            cellReadings.readings[i].quality[beam]   = base::unset<float>();
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
        if (cellReadings.readings.size() != acqConf.cell_count)
        {
            cellReadings.readings.resize(acqConf.cell_count);
            invalidateCellReadings();
        }
        break;
    case raw::VariableLeader::ID:
        parseVariableLeader(buffer, size);
        break;
    case raw::VelocityMessage::ID:
        cellReadings.time = status.time;
        parseVelocityReadings(buffer, size);
        break;
    case raw::CorrelationMessage::ID:
        cellReadings.time = status.time;
        parseCorrelationReadings(buffer, size);
        break;
    case raw::IntensityMessage::ID:
        cellReadings.time = status.time;
        parseIntensityReadings(buffer, size);
        break;
    case raw::QualityMessage::ID:
        cellReadings.time = status.time;
        parseQualityReadings(buffer, size);
        break;
    case raw::BottomTrackingMessage::ID:
        bottomTracking.time = status.time;
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
    deviceInfo.fw_version           = leader.fw_version;
    deviceInfo.fw_revision          = leader.fw_revision;
    deviceInfo.cpu_board_serno      = le64toh(leader.cpu_board_serno);
    deviceInfo.system_configuration = le16toh(leader.system_configuration);
    deviceInfo.beam_count           = leader.beam_count;
    deviceInfo.available_sensors    = parseSensors(leader.available_sensors);

    acqConf.lag_duration                  = leader.lag_duration;
    acqConf.cell_count                    = leader.cell_count;
    acqConf.pings_per_ensemble            = le16toh(leader.pings_per_ensemble);
    acqConf.cell_length                   = 0.01f * le16toh(leader.cell_length);
    acqConf.blank_after_transmit_distance = 0.01f * le16toh(leader.blank_after_transmit_distance);
    acqConf.profiling_mode                = leader.profiling_mode;
    acqConf.low_correlation_threshold     = leader.low_correlation_threshold;
    acqConf.code_repetition_count         = leader.code_repetition_count;
    acqConf.water_layer_min_ping_threshold = 1.0f / 255 * leader.water_layer_min_ping_threshold;
    acqConf.water_layer_velocity_threshold = 0.001 * le16toh(leader.water_layer_velocity_threshold);
    uint64_t milliseconds =
        static_cast<uint32_t>(leader.time_between_ping_groups_min * 60 * 1000) +
        static_cast<uint32_t>(leader.time_between_ping_groups_sec * 1000) +
        static_cast<uint32_t>(leader.time_between_ping_groups_hundredth * 10);
    acqConf.time_between_ping_groups = base::Time::fromMicroseconds(milliseconds * 1000);
    acqConf.yaw_alignment     = M_PI / 180 * 0.01 * le16toh(leader.yaw_alignment);
    acqConf.yaw_bias          = M_PI / 180 * 0.01 * le16toh(leader.yaw_bias);
    acqConf.first_cell_distance   = 0.01f * le16toh(leader.first_cell_distance);
    acqConf.transmit_pulse_length  = 0.01f * le16toh(leader.transmit_pulse_length);
    acqConf.water_layer_start      = leader.water_layer_start;
    acqConf.water_layer_end        = leader.water_layer_end;
    acqConf.false_target_threshold = leader.false_target_threshold;
    acqConf.low_latency_trigger    = leader.low_latency_trigger;
    acqConf.transmit_lag_distance  = 0.01f * le16toh(leader.transmit_lag_distance);
    acqConf.narrow_bandwidth_mode  = le16toh(leader.narrow_bandwidth_mode);
    acqConf.used_sensors           = parseSensors(leader.used_sensors);

    uint8_t mode = leader.coordinate_transformation_mode;
    switch(mode & raw::PD0_COORDINATE_SYSTEM_MASK)
    {
    case raw::PD0_COORD_BEAM:
        outputConf.coordinate_system = BEAM;
        break;
    case raw::PD0_COORD_INSTRUMENT:
        outputConf.coordinate_system = INSTRUMENT;
        break;
    case raw::PD0_COORD_SHIP:
        outputConf.coordinate_system = SHIP;
        break;
    case raw::PD0_COORD_EARTH:
        outputConf.coordinate_system = EARTH;
        break;
    default: throw std::runtime_error("unexpected value for coordinate_transformation_mode & raw::PD0_COORDINATE_SYSTEM_MASK");
    }
    outputConf.use_attitude = mode & raw::PD0_USE_ATTITUDE;
    outputConf.use_3beam_solution = mode & raw::PD0_USE_3BEAM_SOLUTION;
    outputConf.use_bin_mapping = mode & raw::PD0_USE_BIN_MAPPING;
}

void PD0Parser::parseVariableLeader(uint8_t const* buffer, size_t size)
{
    if (size < sizeof(raw::VariableLeader))
        throw std::runtime_error("parseVariableLeader: error in stream, buffer size too small");

    raw::VariableLeader const& msg = *reinterpret_cast<raw::VariableLeader const*>(buffer);

    status.seq  = static_cast<uint32_t>(msg.seq_low) + (static_cast<uint32_t>(msg.seq_high) << 16);
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
        status.time = base::Time::fromSeconds(since_epoch, static_cast<uint64_t>(msg.rtc_hundredth) * 10000);
    }

    status.orientation =
        Eigen::AngleAxisd(M_PI / 180 * 0.01 * le16toh(msg.roll),    Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(M_PI / 180 * 0.01 * le16toh(msg.pitch),   Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(M_PI / 180 * 0.01 * le16toh(msg.yaw), Eigen::Vector3d::UnitZ());
    status.stddev_orientation[0] = M_PI / 180.0f * msg.stddev_yaw;
    status.stddev_orientation[1] = M_PI / 180.0f * 0.1f * msg.stddev_pitch;
    status.stddev_orientation[2] = M_PI / 180.0f * 0.1f * msg.stddev_roll;
    status.speed_of_sound = 1.0f * le16toh(msg.speed_of_sound);
    status.salinity    = 1e-3f * le16toh(msg.salinity_at_transducer);
    status.depth       = 1e-1f * le16toh(msg.depth_of_transducer);
    status.temperature = 1e-2f * le16toh(msg.temperature_at_transducer);
    status.pressure    = 100.0f + 10.0f * le32toh(msg.pressure_at_transducer);
    status.pressure_variance = 100.0f + 10.0f * le32toh(msg.pressure_variance_at_transducer);
    uint64_t milliseconds =
        static_cast<uint32_t>(msg.min_preping_wait_duration_min * 60 * 1000) +
        static_cast<uint32_t>(msg.min_preping_wait_duration_sec * 1000) +
        static_cast<uint32_t>(msg.min_preping_wait_duration_hundredth * 10);
    status.min_preping_wait = base::Time::fromMicroseconds(milliseconds * 1000);

    for (int i = 0; i < 8; ++i)
        status.adc_channels[i] = msg.adc_channels[i];
}

void PD0Parser::parseVelocityReadings(uint8_t const* buffer, size_t size)
{
    if (size < sizeof(raw::VelocityMessage) + acqConf.cell_count * sizeof(raw::CellVelocity))
        throw std::runtime_error("parseVelocityReadings: buffer size too small");

    raw::VelocityMessage const& msg = *reinterpret_cast<raw::VelocityMessage const*>(buffer);
    for (int cell_idx = 0; cell_idx < acqConf.cell_count; ++cell_idx)
    {
        // This is pre-sized as soon as we know the number of cells in the
        // acquisition process
        CellReading& cell = cellReadings.readings[cell_idx];
        
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
    if (size < sizeof(raw::CorrelationMessage) + acqConf.cell_count * sizeof(raw::CellCorrelation))
        throw std::runtime_error("parseCorrelationReadings: buffer size too small");

    raw::CorrelationMessage const& msg = *reinterpret_cast<raw::CorrelationMessage const*>(buffer);
    for (int cell_idx = 0; cell_idx < acqConf.cell_count; ++cell_idx)
    {
        // This is pre-sized as soon as we know the number of cells in the
        // acquisition process
        CellReading& cell = cellReadings.readings[cell_idx];
        
        for (int beam_idx = 0; beam_idx < 4; ++beam_idx)
        {
            uint8_t value = msg.correlations[beam_idx].correlation[beam_idx];
            cell.correlation[beam_idx] = 1.0f / 255 * value;
        }
    }
}

void PD0Parser::parseIntensityReadings(uint8_t const* buffer, size_t size)
{
    if (size < sizeof(raw::IntensityMessage) + acqConf.cell_count * sizeof(raw::CellIntensity))
        throw std::runtime_error("parseIntensityReadings: buffer size too small");

    raw::IntensityMessage const& msg = *reinterpret_cast<raw::IntensityMessage const*>(buffer);
    for (int cell_idx = 0; cell_idx < acqConf.cell_count; ++cell_idx)
    {
        // This is pre-sized as soon as we know the number of cells in the
        // acquisition process
        CellReading& cell = cellReadings.readings[cell_idx];
        
        for (int beam_idx = 0; beam_idx < 4; ++beam_idx)
        {
            uint8_t value = msg.intensities[beam_idx].intensity[beam_idx];
            cell.intensity[beam_idx] = 0.45 * value;
        }
    }
}

void PD0Parser::parseQualityReadings(uint8_t const* buffer, size_t size)
{
    if (size < sizeof(raw::QualityMessage) + acqConf.cell_count * sizeof(raw::CellQuality))
        throw std::runtime_error("parseQualityReadings: buffer size too small");

    raw::QualityMessage const& msg = *reinterpret_cast<raw::QualityMessage const*>(buffer);
    for (int cell_idx = 0; cell_idx < acqConf.cell_count; ++cell_idx)
    {
        // This is pre-sized as soon as we know the number of cells in the
        // acquisition process
        CellReading& cell = cellReadings.readings[cell_idx];
        
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

    bottomTrackingConf.ping_per_ensemble = le16toh(msg.bottom_ping_per_ensemble);
    bottomTrackingConf.delay_before_reacquiring = le16toh(msg.bottom_delay_before_reacquiring);
    bottomTrackingConf.correlation_threshold = 1.0f / 255 * msg.bottom_correlation_threshold;
    bottomTrackingConf.evaluation_threshold  = 1.0f / 255 * msg.bottom_evaluation_threshold;
    bottomTrackingConf.good_ping_threshold = 0.01f * msg.bottom_good_ping_threshold;
    bottomTrackingConf.mode = msg.bottom_mode;
    bottomTrackingConf.max_velocity_error = 1e-3f * le16toh(msg.bottom_max_velocity_error);
    bottomTrackingConf.max_tracking_depth = 1e-1f * le16toh(msg.max_tracking_depth);

    bottomTracking.time = status.time;
    for (int beam = 0; beam < 4; ++beam)
    {
        uint32_t value = static_cast<uint32_t>(le16toh(msg.bottom_range_low[beam])) +
            (static_cast<uint32_t>(msg.bottom_range_high[beam]) << 16);
        if (value)
            bottomTracking.range[beam]           = 1e-2f * value;
        else
            bottomTracking.range[beam]           = base::unknown<float>();

        int16_t velocity = le16toh(msg.bottom_velocity[beam]);
        if (velocity == -32768)
            bottomTracking.velocity[beam]        = base::unknown<float>();
        else

            bottomTracking.velocity[beam]        = 1e-3f * velocity;
        bottomTracking.correlation[beam]     = 1.0f / 255 * msg.bottom_correlation[beam];
        bottomTracking.evaluation[beam]      = 1.0f / 255 * msg.bottom_evaluation[beam];
        bottomTracking.good_ping_ratio[beam] = 1.0f / 255 * msg.bottom_good_ping_ratio[beam];
        bottomTracking.rssi[beam]            = 0.45f * msg.rssi[beam];
    }
}

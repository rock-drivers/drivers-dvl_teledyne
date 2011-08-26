#ifndef DVL_TELEDYNE_PD0_HPP
#define DVL_TELEDYNE_PD0_HPP

#include <stdint.h>
#include <base/time.h>
#include <base/eigen.h>
#include <boost/static_assert.hpp>
#include <vector>

namespace dvl_teledyne
{
    namespace pd0_raw_messages
    {
        struct Header
        {
            enum MSG_ID { ID = 0x7f, DATA_SOURCE_ID = 0x7f };
            uint8_t id;
            uint8_t data_source_id;
            uint16_t size;
            uint8_t  spare;
            uint16_t msg_count;
            uint16_t offsets[0];
        } __attribute__((packed));

        /** Flags of the coordinate_transformation_mode field in the FixedLeader
         * structure
         */
        enum PD0_COORDINATE_TRANSFORMATION_MODES
        {
            PD0_COORDINATE_SYSTEM_MASK = 0x18,
            PD0_COORD_BEAM        = 0x00,
            PD0_COORD_INSTRUMENT  = 0x08,
            PD0_COORD_SHIP        = 0x10,
            PD0_COORD_EARTH       = 0x18,

            PD0_USE_ATTITUDE      = 0x04,
            PD0_USE_3BEAM_SOLUTION = 0x02,
            PD0_USE_BINARY_MAPPING = 0x01
        };

        /** List of sensors */
        enum PD0_SENSORS
        {
            PD0_CALCULATE_SPEED_OF_SOUND = 0x40,
            PD0_DEPTH_SENSOR             = 0x20,
            PD0_YAW_SENSOR               = 0x10,
            PD0_PITCH_SENSOR             = 0x08,
            PD0_ROLL_SENSOR              = 0x04,
            PD0_SALINITY_SENSOR          = 0x02,
            PD0_TEMPERATURE_SENSOR       = 0x01
        };

        struct FixedLeader
        {
            enum MSG_ID { ID = 0x0000 };
            uint16_t id;
            uint8_t  fw_version;
            uint8_t  fw_revision;
            uint16_t system_configuration;
            uint8_t  simulated_data;
            uint8_t  lag_duration;
            uint8_t  beam_count;
            uint8_t  cell_count;
            uint16_t pings_per_ensemble;
            uint16_t cell_length;
            uint16_t blank_after_transmit_distance;
            uint8_t  profiling_mode;
            uint8_t  low_correlation_threshold;
            uint8_t  code_repetition_count;
            uint8_t  water_layer_min_ping_threshold;
            uint16_t water_layer_velocity_threshold;
            uint8_t  time_between_ping_groups_min;
            uint8_t  time_between_ping_groups_sec;
            uint8_t  time_between_ping_groups_hundredth;
            uint8_t  coordinate_transformation_mode;
            uint16_t yaw_alignment;
            uint16_t yaw_bias;
            uint8_t  used_sensors;
            uint8_t  available_sensors;
            uint16_t first_cell_distance;
            uint16_t transmit_pulse_length;
            uint8_t  water_layer_start;
            uint8_t  water_layer_end;
            uint8_t  false_target_threshold;
            uint8_t  low_latency_trigger;
            uint16_t transmit_lag_distance;
            uint64_t cpu_board_serno;
            uint16_t narrow_bandwidth_mode;
            uint8_t  spare0;
            uint8_t  base_frequency_index;
            uint8_t  spare1[5];
        } __attribute__((packed));
        BOOST_STATIC_ASSERT(sizeof(FixedLeader) == 59);

        /** Flags of the status field in the VariableLeader structure */
        enum PD0_STATUS_FLAGS
        {
            PD0_STATUS_BUS_ERROR           = 0x00000001,
            PD0_STATUS_ADDRESS_ERROR       = 0x00000002,
            PD0_STATUS_ILLEGAL_INSTRUCTION = 0x00000004,
            PD0_STATUS_ZERO_DIVIDE         = 0x00000008,
            PD0_STATUS_EMULATOR_ERROR      = 0x00000010,
            PD0_STATUS_UNASSIGNED_ERROR    = 0x00000020,
            PD0_STATUS_WATCHDOG_RESTART    = 0x00000040,
            PD0_STATUS_BATTERY_SAVER_POWER = 0x00000080,

            PD0_STATUS_PINGING             = 0x00000100,
            PD0_STATUS_COLD_WAKEUP         = 0x00000400,
            PD0_STATUS_UNKNOWN_WAKEUP      = 0x00000800,

            PD0_STATUS_CLOCK_READ_ERROR    = 0x010000,
            PD0_STATUS_UNEXPECTED_ALARM    = 0x020000,
            PD0_STATUS_CLOCK_JUMP_FORWARD  = 0x040000,
            PD0_STATUS_CLOCK_JUMP_BACKWARD = 0x080000,

            PD0_STATUS_POWER_FAIL          = 0x08000000,
            PD0_STATUS_SPURIOUS_DSP_INTR   = 0x10000000,
            PD0_STATUS_SPURIOUS_UART_INTR  = 0x20000000,
            PD0_STATUS_SPURIOUS_CLOCK_INTR = 0x40000000,
            PD0_STATUS_LEVEL7_INTR         = 0x80000000,
        } __attribute__((packed));

        struct VariableLeader
        {
            enum MSG_ID { ID = 0x8000 };
            uint16_t id;
            uint16_t seq_low;
            uint8_t  rtc_year;
            uint8_t  rtc_month;
            uint8_t  rtc_day;
            uint8_t  rtc_hour;
            uint8_t  rtc_min;
            uint8_t  rtc_sec;
            uint8_t  rtc_hundredth;
            uint8_t  seq_high;

            uint16_t self_test_result;
            uint16_t speed_of_sound;
            uint16_t depth_of_transducer;
            uint16_t yaw;
            uint16_t pitch;
            uint16_t roll;
            uint16_t salinity_at_transducer;
            uint16_t temperature_at_transducer;
            uint8_t min_preping_wait_duration_min;
            uint8_t min_preping_wait_duration_sec;
            uint8_t min_preping_wait_duration_hundredth;
            uint8_t stddev_yaw;
            uint8_t stddev_pitch;
            uint8_t stddev_roll;
            uint8_t adc_channels[8];
            uint32_t status_word;
            uint16_t reserved;
            uint32_t pressure_at_transducer;
            uint32_t pressure_variance_at_transducer;
            uint8_t  spare;
            uint8_t  y2k_rtc_century;
            uint8_t  y2k_rtc_year;
            uint8_t  y2k_rtc_month;
            uint8_t  y2k_rtc_day;
            uint8_t  y2k_rtc_hour;
            uint8_t  y2k_rtc_min;
            uint8_t  y2k_rtc_sec;
            uint8_t  y2k_rtc_hundredth;
        } __attribute__((packed));
        BOOST_STATIC_ASSERT(sizeof(VariableLeader) == 65);


        /** Format of a velocity for one depth cell
         */
        struct CellVelocity
        {
            uint16_t velocity[4];
        };

        /** Variable-size message that contain water velocity w.r.t. the DVL per
         * depth-cell */
        struct VelocityMessage
        {
            enum MSG_ID { ID = 0x001 };
            int16_t id;
            CellVelocity velocities[0];
        } __attribute__((packed));

        struct CellCorrelation
        {
            uint8_t correlation[4];
        };

        struct CorrelationMessage
        {
            enum MSG_ID { ID = 0x002 };
            uint16_t id;
            CellCorrelation correlations[0];
        } __attribute__((packed));

        struct CellIntensity
        {
            uint8_t intensity[4];
        };

        struct IntensityMessage
        {
            enum MSG_ID { ID = 0x003 };
            uint16_t id;
            CellIntensity intensities[0];
        } __attribute__((packed));

        struct CellQuality
        {
            uint8_t quality[4];
        };

        struct QualityMessage
        {
            enum MSG_ID { ID = 0x004 };
            uint16_t id;
            CellQuality quality[0];
        } __attribute__((packed));

        struct BottomTrackingMessage
        {
            enum MSG_ID { ID = 0x006 };
            uint16_t id;

            uint16_t bottom_ping_per_ensemble;
            uint16_t bottom_delay_before_reacquiring;
            uint8_t  bottom_correlation_threshold;
            uint8_t  bottom_evaluation_threshold;
            uint8_t  bottom_good_ping_threshold;
            uint8_t  bottom_mode;
            uint16_t bottom_max_velocity_error;
            uint32_t reserved;
            uint16_t bottom_range_low[4];

            uint16_t bottom_velocity[4];
            uint8_t  bottom_correlation[4];
            uint8_t  bottom_evaluation[4];
            uint8_t  bottom_good_ping_ratio[4];

            uint16_t water_layer_min_size;
            uint16_t water_layer_near_boundary;
            uint16_t water_layer_far_boundary;
            uint16_t water_layer_velocity[4];
            uint8_t  water_layer_correlation_magnitude[4];
            uint8_t  water_layer_intensity[4];
            uint8_t  water_layer_quality[4];

            uint16_t max_tracking_depth;
            uint8_t  rssi[4];
            uint8_t  gain;
            uint8_t  bottom_range_high[4];
            uint8_t  reserved1[4];
        } __attribute__((packed));
        BOOST_STATIC_ASSERT(sizeof(BottomTrackingMessage) == 85);
    };

    namespace pd0_messages
    {

        struct Sensors
        {
            bool calculates_speed_of_sound;
            bool depth;
            bool yaw;
            bool pitch;
            bool roll;
            bool salinity;
            bool temperature;
        };

        /** General device information */
        struct DeviceInfo
        {
            uint8_t fw_version;
            uint8_t fw_revision;
            uint64_t cpu_board_serno;
            uint16_t system_configuration;
            uint8_t  beam_count;

            /* Set of sensors available on the device */
            Sensors available_sensors;
        };

        /** Acquisition information as reported by the device */
        struct AcquisitionConfiguration
        {
            /** Set of sensors used during the acquisition */
            Sensors  used_sensors;

            uint8_t  lag_duration;
            /** How many depth cells are sampled */
            uint8_t  cell_count;
            uint8_t  profiling_mode;
            uint8_t  low_correlation_threshold;
            uint8_t  code_repetition_count;
            /** Total count of pings per ensemble */
            uint16_t pings_per_ensemble;
            /** Length of a depth cell in meters */
            float    cell_length;
            /** ??? Is it the minimum ground distance allowed ? */
            float    blank_after_transmit_distance;
            /** Minimum percentage of good pings in the water profiling data
             * required to output a velocity */
            float    water_layer_min_ping_threshold;
            /** Reject water velocity estimate if the error is above this
             * threshold (in m/s) */
            float    water_layer_velocity_threshold;
            /** Time between two consecutive ping groups */
            base::Time time_between_ping_groups;
            /** Angular correction between beam 3 and the yaw sensor, in
             * radians
             */
            float yaw_alignment;
            /** Magnetic correction: angular correction applied to the yaw
             * sensor reading, in radians
             */
            float yaw_bias;
            /** Distance to the first depth cell, in meters
             */
            float first_cell_distance;
            /** Length, in meters, of a pulse
             */
            float transmit_pulse_length;
            /** Start of the water layer in depth cells */
            uint8_t  water_layer_start;
            /** End of the water layer in depth cells */
            uint8_t  water_layer_end;
            /** Intensity threshold above which a cell or a beam are discarded
             * during processing
             */
            uint8_t  false_target_threshold;
            /** If true, the low latency hardware trigger is enabled
             */
            bool  low_latency_trigger;
            /** Distance between pulse repetitions, in meters */
            float transmit_lag_distance;
            /** If true, the device is in narrow bandwidth mode (higher range
             * but lower accuracy)
             */
            bool  narrow_bandwidth_mode;
            /** ? */
            uint8_t  base_frequency_index;
        };

        enum COORDINATE_SYSTEM
        {
            BEAM, INSTRUMENT, SHIP, EARTH
        };

        /** Configuration of the device estimation */
        struct OutputConfiguration
        {
            COORDINATE_SYSTEM coordinate_system;
            bool use_attitude;
            bool use_3beam_solution;
            bool use_binary_mapping;
        };

        /** Realtime device state */
        struct DeviceState
        {
            uint32_t seq;
            base::Time time;

            base::Quaterniond orientation;
            /** Standard deviation of orientation in yaw, pitch and roll */
            float stddev_orientation[3];

            float depth;
            float speed_of_sound;
            float salinity;
            float temperature;
            float pressure;
            float pressure_variance;

            uint8_t adc_channels[8];

            base::Time min_preping_wait;

            uint16_t self_test_result;
            uint32_t status_word;
        };

        struct CellReading
        {
            /** Velocity. The meaning of the four values depend on the
             * coordinate transformation mode:
             *
             * <ul>
             *   <li>BEAM: velocities on each beam
             *   <li>INSTRUMENT: Beam2-Beam1, Beam4-Beam3, to transducer, Error
             *   <li>SHIP: left->right, back->front, to surface, error
             *   <li>EARTH: to east, to north, to surface, Error
             * </ul>
             */
            float velocity[4];
            /** Correlation magnitude in a [0, 1] scale */
            float correlation[4];
            /** Correlation intensity in dB */
            float intensity[4];
            /** Quality indicator. The interpretation of it depends on the
             * coordinate mode setting
             *
             * <ul>
             *   <li>BEAM: percentage of good pings for each bins
             *   <li>any other setting:
             *      <ul>
             *      <li>ratio of 3-beam solutions (correlation threshold not exceeded)
             *      <li>ratio of rejected transformations (error velocity above water_layer_velocity_threshold)
             *      <li>ratio of rejection because not enough bins had good data
             *      <li>ratio of 4-beam solutions
             *      </ul>
             *   </li>
             * </ul>
             */
            float quality[4];
        };

        /** Depth cell information */
        struct CellReadings
        {
            base::Time time;
            std::vector<CellReading> readings;
        };

        struct BottomTrackingConfiguration
        {
            /** Number of pings to average in one ensemble */
            uint16_t ping_per_ensemble;
            /** Delay in ensemble to wait before trying to reacquire */
            uint16_t delay_before_reacquiring;
            /** Minimum acceptable correlation magnitude (between 0 and 1) */
            float    correlation_threshold;
            /** Minimum acceptable evaluation magnitude (between 0 and 1) */
            float    evaluation_threshold;
            /** Minimum ratio of good pings needed to output a velocity (between
             * 0 and 1)
             */
            float    good_ping_threshold;
            /** Bottom tracking mode (see BM command) */
            uint8_t  mode;
            /** Maximum acceptable velocity error (in m/s) */
            float  max_velocity_error;

            /** Maximum configured tracking depth */
            float max_tracking_depth;

            /** Gain level for shallow water
             * See the WJ command
             */
            uint8_t  gain;
        };

        /** Bottom tracking information */
        struct BottomTracking
        {
            /** Acquisition timestamp */
            base::Time time;

            /** Ranges to the bottom, in meters
             */
            float range[4];
            /** Velocities. The reported information depends on the coordinate
             * transformation setting. See CellReading documentation for more
             * information
             */
            float velocity[4];
            /** Correlation at the bottom cell (between 0 and 1) */
            float correlation[4];
            /** Magnitude in the evaluation filter, for each beam */
            float evaluation[4];
            /** Ratio of good bottom tracking pings */
            float good_ping_ratio[4];

            /** RSSI at the center of the bottom ping (in dB)
             */
            float rssi[4];
        };
    };

    class PD0Parser
    {
    protected:
        int extractPacket(uint8_t const* buffer, size_t size, size_t max_size = 0) const;
        int getSizeOfMessage(uint16_t msg_id) const;
        void invalidateCellReadings();
        void parseMessage(uint8_t const* buffer, size_t size);
        void parseFixedLeader(uint8_t const* buffer, size_t size);
        void parseVariableLeader(uint8_t const* buffer, size_t size);
        void parseQualityReadings(uint8_t const* buffer, size_t size);
        void parseCorrelationReadings(uint8_t const* buffer, size_t size);
        void parseIntensityReadings(uint8_t const* buffer, size_t size);
        void parseVelocityReadings(uint8_t const* buffer, size_t size);
        void parseBottomTrackingReadings(uint8_t const* buffer, size_t size);

    public:
        pd0_messages::DeviceInfo mDeviceInfo;
        pd0_messages::AcquisitionConfiguration mAcqConf;
        pd0_messages::OutputConfiguration mOutputConf;
        pd0_messages::DeviceState mDeviceState;
        pd0_messages::CellReadings mCellReadings;
        pd0_messages::BottomTrackingConfiguration mBTConf;
        pd0_messages::BottomTracking mBT;

        void parseEnsemble(uint8_t const* data, size_t size);
    };
}

#endif

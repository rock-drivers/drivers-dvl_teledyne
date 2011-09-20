#ifndef DVL_TELEDYNE_PD0RAW_HPP
#define DVL_TELEDYNE_PD0RAW_HPP

#include <stdint.h>
#include <boost/static_assert.hpp>
#include <vector>

namespace dvl_teledyne
{
    namespace raw
    {
        struct Header
        {
            enum MSG_ID { ID = 0x7f, DATA_SOURCE_ID = 0x7f };
            uint8_t id;
            uint8_t data_source_id;
            uint16_t size;
            uint8_t  spare;
            uint8_t  msg_count;
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
            PD0_USE_BIN_MAPPING = 0x01
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
            enum MSG_ID { ID = 0x0080 };
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
            enum MSG_ID { ID = 0x0100 };
            int16_t id;
            CellVelocity velocities[0];
        } __attribute__((packed));

        struct CellCorrelation
        {
            uint8_t correlation[4];
        };

        struct CorrelationMessage
        {
            enum MSG_ID { ID = 0x0200 };
            uint16_t id;
            CellCorrelation correlations[0];
        } __attribute__((packed));

        struct CellIntensity
        {
            uint8_t intensity[4];
        };

        struct IntensityMessage
        {
            enum MSG_ID { ID = 0x0300 };
            uint16_t id;
            CellIntensity intensities[0];
        } __attribute__((packed));

        struct CellQuality
        {
            uint8_t quality[4];
        };

        struct QualityMessage
        {
            enum MSG_ID { ID = 0x0400 };
            uint16_t id;
            CellQuality quality[0];
        } __attribute__((packed));

        struct BottomTrackingMessage
        {
            enum MSG_ID { ID = 0x0600 };
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
            // the Explorer DVL does not have these 4 Navigator-specific bytes: uint8_t  reserved1[4];
        } __attribute__((packed));
        BOOST_STATIC_ASSERT(sizeof(BottomTrackingMessage) == 81);
    };
}

#endif

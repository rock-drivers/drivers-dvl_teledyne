#ifndef DVL_TELEDYNE_PD0MESSAGES_HPP
#define DVL_TELEDYNE_PD0MESSAGES_HPP

#include <stdint.h>
#include <base/time.h>
#include <base/eigen.h>
#include <vector>

namespace dvl_teledyne
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
}

#endif

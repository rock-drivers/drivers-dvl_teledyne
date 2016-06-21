#ifndef DVL_TELEDYNE_PD0PARSER_HPP
#define DVL_TELEDYNE_PD0PARSER_HPP

#include <stdint.h>
#include <base/Time.hpp>
#include <base/Eigen.hpp>
#include <vector>

#include <dvl_teledyne/PD0Messages.hpp>

namespace dvl_teledyne
{
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

        DeviceInfo deviceInfo;
        AcquisitionConfiguration acqConf;
        OutputConfiguration outputConf;
        Status status;
        CellReadings cellReadings;
        BottomTrackingConfiguration bottomTrackingConf;
        BottomTracking bottomTracking;

        void parseEnsemble(uint8_t const* data, size_t size);
    };
}

#endif

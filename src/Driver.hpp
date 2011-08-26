#ifndef DVL_TELEDYNE_DRIVER_HPP
#define DVL_TELEDYNE_DRIVER_HPP

#include <iodrivers_base/Driver.hpp>
#include <dvl_teledyne/PD0Parser.hpp>

namespace dvl_teledyne
{
    class Driver : public iodrivers_base::Driver, public PD0Parser
    {
        std::vector<uint8_t> buffer;
        int extractPacket (uint8_t const *buffer, size_t buffer_size) const;

    public:
        Driver();
        void read();
    };
}

#endif


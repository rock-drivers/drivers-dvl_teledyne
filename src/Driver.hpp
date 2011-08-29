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

        bool mConfMode;
        int mDesiredBaudrate;

        /** Tells the DVL to switch to the desired rate */
        void setDeviceBaudrate(int rate);

    public:
        Driver();
        void open(std::string const& uri);

        /** Once open using the baudrate specified in the URI, configures the
         * device to output at a different baud rate, and modifies the driver's
         * configuration accordingly
         */
        void setDesiredBaudrate(int rate);
        
        /** Sends a text file that contains commands to the device
         *
         * The device is guaranteed to be in configuration mode afterwards
         * (regardless of whether the configuration file contains a CS
         * command). Use startAcquisition() to put it in acquisition mode
         */
        void sendConfigurationFile(std::string const& file_name);

        void setConfigurationMode();
        void startAcquisition();
        void read();
        void readConfigurationAck(base::Time const& timeout = base::Time::fromSeconds(1));
    };
}

#endif


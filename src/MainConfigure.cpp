#include <dvl_teledyne/Driver.hpp>
#include <iostream>
#include <fstream>

using namespace dvl_teledyne;

void usage()
{
    std::cerr << "dvl_teledyne_configure DEVICE [FILE]" << std::endl;
}

int main(int argc, char const* argv[])
{
    if (argc < 2 || argc > 3)
    {
        usage();
        return 1;
    }

    dvl_teledyne::Driver driver;
    driver.openSerial(argv[1], 9600);
    driver.setWriteTimeout(base::Time::fromSeconds(5));
    driver.setReadTimeout(base::Time::fromSeconds(5));

    if (argc == 3)
        driver.sendConfigurationFile(argv[2]);

    // The file might contain a CS command. Make sure we are still in
    // configuration mode
    driver.setConfigurationMode();
    driver.startAcquisition();
}




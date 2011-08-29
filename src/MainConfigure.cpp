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

    driver.setConfigurationMode();
    if (argc == 3)
    {
        std::ifstream file(argv[2]);
        std::string line;

        char line_buffer[2000];
        while (!file.eof())
        {
            if (!file.getline(line_buffer, 2000) && !file.eof())
                throw std::runtime_error("lines longer than 2000 characters");

            std::string line(line_buffer);
            line += "\n";
            std::cout << iodrivers_base::Driver::printable_com(line) << std::endl;
            driver.writePacket(reinterpret_cast<uint8_t const*>(line.c_str()), line.length());
            driver.readConfigurationAck();
        }
        driver.setConfigurationMode();
    }
    driver.startAcquisition();
}




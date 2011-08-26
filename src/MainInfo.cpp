#include <dvl_teledyne/Driver.hpp>
#include <iostream>

void usage()
{
    std::cerr << "dvl_teledyne_info DEVICE" << std::endl;
}

int main(int argc, char const* argv[])
{
    if (argc != 2)
    {
        usage();
        return 1;
    }

    dvl_teledyne::Driver driver;
    driver.openSerial(argv[1], 9600);
    driver.setReadTimeout(base::Time::fromSeconds(5));
    driver.read();

    std::cout << "Found device" << std::endl;
    std::cout << "  fw: " << (int)driver.mDeviceInfo.fw_version << "." << (int)driver.mDeviceInfo.fw_revision << std::endl;
    std::cout << "  serno: " << std::hex << driver.mDeviceInfo.cpu_board_serno << std::dec << std::endl;
    std::cout << "  beam count: " << (int)driver.mDeviceInfo.beam_count << std::endl;
    std::cout << "  available sensors:" << std::endl;
    dvl_teledyne::Sensors const& sensors = driver.mDeviceInfo.available_sensors;
    std::cout << "    depth: " << (sensors.depth ? "yes" : "no") << std::endl;
    std::cout << "    yaw: "   << (sensors.yaw ? "yes" : "no") << std::endl;
    std::cout << "    pitch: " << (sensors.pitch ? "yes" : "no") << std::endl;
    std::cout << "    roll: "  << (sensors.roll ? "yes" : "no") << std::endl;
    std::cout << "    salinity: "  << (sensors.salinity ? "yes" : "no") << std::endl;
    std::cout << "    temperature: "  << (sensors.temperature ? "yes" : "no") << std::endl;
    std::cout << std::endl;
    return 0;
}


#include <dvl_teledyne/Driver.hpp>
#include <iostream>

using namespace dvl_teledyne;

void usage()
{
    std::cerr << "dvl_teledyne_read DEVICE" << std::endl;
}

int main(int argc, char const* argv[])
{
    if (argc != 2)
    {
        usage();
        return 1;
    }

    dvl_teledyne::Driver driver;
    driver.openSerial(argv[1], 115200);
    driver.setReadTimeout(base::Time::fromSeconds(5));

    while(true)
    {
        driver.read();

        BottomTracking const& tracking = mDeviceInfo.mBottomTracking;
        std::cout << tracking.time.toString();
        for (int beam = 0; beam < 4; ++beam)
            std::cout << " " << tracking.range[beam] << " " << tracking.velocity[beam] << " " << tracking.evaluation[beam];
        std::cout << std::endl;
    }
}



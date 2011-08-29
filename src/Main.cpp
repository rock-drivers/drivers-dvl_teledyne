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
    driver.open(argv[1]);
    driver.setReadTimeout(base::Time::fromSeconds(5));
    driver.read();

    char const* coord_systems[4] = { "BEAM", "INSTRUMENT", "SHIP", "EARTH" };
    std::cout << "Device outputs its data in the " << coord_systems[driver.mOutputConf.coordinate_system] << " coordinate system" << std::endl;


    std::cout << "Time Seq ";
    for (int beam = 0; beam < 4; ++beam)
        std::cout << " range[" << beam << "] velocity[" << beam << "] evaluation[" << beam << "]";
    std::cout << std::endl;

    while(true)
    {
        driver.read();

        BottomTracking const& tracking = driver.mBottomTracking;
        std::cout << tracking.time.toString() << " " << driver.mStatus.seq;
        for (int beam = 0; beam < 4; ++beam)
            std::cout << " " << tracking.range[beam] << " " << tracking.velocity[beam] << " " << tracking.evaluation[beam];
        std::cout << std::endl;
    }
}



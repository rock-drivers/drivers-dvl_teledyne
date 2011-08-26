#include <dvl_teledyne/Driver.hpp>

using namespace dvl_teledyne;

Driver::Driver()
    : iodrivers_base::Driver(1000000)
{
    buffer.resize(1000000);
}

void Driver::read()
{
    int packet_size = readPacket(&buffer[0], buffer.size());
    if (packet_size)
        parseEnsemble(&buffer[0], packet_size);
}

int Driver::extractPacket (uint8_t const *buffer, size_t buffer_size) const
{
    return PD0Parser::extractPacket(buffer, buffer_size);
}


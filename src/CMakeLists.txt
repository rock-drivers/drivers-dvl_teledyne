rock_library(dvl_teledyne
    SOURCES PD0Parser.cpp Driver.cpp
    HEADERS PD0Messages.hpp PD0Raw.hpp PD0Parser.hpp Driver.hpp
    DEPS_PKGCONFIG base-types iodrivers_base)

rock_executable(dvl_teledyne_info
    MainInfo.cpp
    DEPS dvl_teledyne)
rock_executable(dvl_teledyne_read
    Main.cpp
    DEPS dvl_teledyne)

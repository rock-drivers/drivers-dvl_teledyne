rock_library(dvl_teledyne
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(dvl_teledyne_bin Main.cpp
    DEPS dvl_teledyne)


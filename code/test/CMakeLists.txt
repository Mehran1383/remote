cmake_minimum_required(VERSION 3.10)
project(ublox_rtk)

set(CMAKE_CXX_STANDARD 17)

find_package(serial REQUIRED)

add_executable(ublox_rtk main.cpp)
target_link_libraries(ublox_rtk serial)
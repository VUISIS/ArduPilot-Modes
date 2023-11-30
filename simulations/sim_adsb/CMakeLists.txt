cmake_minimum_required(VERSION 3.13)

project (sim_adsb)

set(CMAKE_C_STANDARD 11)

find_package(MAVLink REQUIRED)

add_executable(sim_adsb sim_adsb.c)

target_link_libraries(sim_adsb PRIVATE MAVLink::mavlink)
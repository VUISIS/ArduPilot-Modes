cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(avoid)

add_executable(avoid
    avoid.cpp
)

find_package(MAVSDK REQUIRED)

target_link_libraries(avoid
    MAVSDK::mavsdk
)

if(NOT MSVC)
    add_compile_options(avoid PRIVATE -Wall -Wextra)
else()
    add_compile_options(avoid PRIVATE -WX -W2)
endif()

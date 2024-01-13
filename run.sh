#!/bin/bash

git submodule update --init --recursive

# Install MAVSDK
(cd modules/MAVSDK &&
cmake -Bbuild -H. -DCMAKE_INSTALL_PREFIX=install -DMAVLINK_DIALECT=ardupilotmega -DMAVLINK_VERSION=2.0 &&
cmake --build build --target install)

# Run the following command in a separate terminal
# sim_vehicle.py -v ArduCopter --console --map 
# Run the following command for or ADSB simulation and may need change the serial port number.
# sim_vehicle.py -v ArduCopter -A "--uartC uart:/dev/ttyS3:57600"

if [ "$1" = AUTO ] ; then
    MODE_DIR="auto_mode/c"
    EXECUTABLE_NAME="waypoints"
elif [ "$1" = GUIDED ] ; then
    MODE_DIR=guided_mode/c
    EXECUTABLE_NAME="guided"
elif [ "$1" = AVOID ] ; then
    MODE_DIR=avoid_obstacle/c
    EXECUTABLE_NAME="avoid"
elif [ "$1" = GEOFENCE ]; then
    MODE_DIR=geofence/c
    EXECUTABLE_NAME="geofence"
elif [ "$1" = MONITOR ]; then
    MODE_DIR=traffic_monitor/c
    EXECUTABLE_NAME="traffic_monitor"
else
    echo "Please specify mode: AUTO, GUIDED, AVOID, MONITOR"
    return
fi

(cd $MODE_DIR && 
cmake -Bbuild -H. -DCMAKE_PREFIX_PATH=$(pwd)/../../modules/MAVSDK/install && 
cmake --build build -j8 &&
# You may need to change the port number according to your simulation environment
./build/$EXECUTABLE_NAME tcp://:5763)

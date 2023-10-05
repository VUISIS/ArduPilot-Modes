#!/bin/bash

git submodule update --init --recursive

# sim_vehicle.py -v ArduCopter --console --map 

if [ "$1" = AUTO ] ; then
    MODE_DIR="auto_mode/c"
elif [ "$1" = GUIDED ] ; then
    MODE_DIR=guided_mode/c
elif [ "$1" = AVOID ] ; then
    MODE_DIR=avoid_obstacle/c
else
    echo "Please specify mode: AUTO, GUIDED, AVOID"
    return
fi

(cd $MODE_DIR && 
cmake -Bbuild -H. -DCMAKE_PREFIX_PATH=$(pwd)/../../modules/MAVSDK/install && 
cmake --build build -j8 &&
./build/waypoints tcp://:5762)

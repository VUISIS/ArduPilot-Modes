#!/bin/bash

cmake -Bbuild -H. -DCMAKE_PREFIX_PATH=$(pwd)/../../../modules/mavlink/install

cmake --build build

./build/sim_adsb
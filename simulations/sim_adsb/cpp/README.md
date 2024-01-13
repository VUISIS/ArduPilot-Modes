# ADS-B signal generation with mavlink
First of all, make sure mavlink is installed correctly as a submodule under root directory. The mavlink fork from Ardupilot organization does not work and it has to be the original [mavlink](https://github.com/mavlink/mavlink) from Mavlink organization.

## Build and compile
```bash
cmake -Bbuild -H. -DCMAKE_PREFIX_PATH=$(pwd)/../../../modules/mavlink/install

cmake --build build
```

## Run executable
```bash
./build/sim_adsb
```
This repository contains Ardupilot applications written in Python. We used both Arducopter and ArduPlane targets to build and run different modes on SITL board. 

## Install Ardupilot

Follow [Ardupilot instructions](https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md) to install for the configurations listed below: 

- Board: sitl 
- Vehicle types: copter, plane

## Install ArduPilot DroneKit Python

Install [DroneKit](https://github.com/dronekit/dronekit-sitl#dronekit-sitl) API. 

```
pip install dronekit-sitl
dronekit-sitl <copter(-version)> [parameters]
```

## Running Ardupilot modes
Run the `simvehicle.py` tool on one terminal and use another terminal for the next step.
```
sim_vehicle.py -v ArduCopter --console --map
```
This will spawn a drone, which is ready to be connected in this TCP address: 127.0.0.1:14550

Now, use another terminal to clone this repository and run different modes: 

```
cd auto_mode
python waypoints.py --connect 127.0.0.1:14550
```

This will connect to the drone launched by `simvehicle.py` and run the waypoints mission. Similarly, other modes could be tested.

![Drone Simulation](simulation.gif)
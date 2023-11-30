#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
import csv

def read_csv_file(file_name):
    values = []
    with open(file_name, 'r') as csv_file:
        csv_reader = csv.reader(csv_file)
        for row in csv_reader:
            values.append(row)
    return values

# Example usage
file_name = 'file-counterexample.txt'
data = read_csv_file(file_name)
# print(data[0])

X = []
Y = []
Z = []
B = []

for row in data:
    valX = row[0][14:]
    valY = row[1][3:]
    valZ = row[2][5:]
    valB = row[3][9:]
    X.append(int(valX))
    Y.append(int(valY))
    Z.append(int(valZ))
    B.append(int(valB))

# Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None


# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


arm_and_takeoff(10)

print("Set default/target airspeed to 3")
vehicle.airspeed = 10

# Get the current location
current_location = vehicle.location.global_frame

# Retrieve the longitude and latitude
longitude = current_location.lon
latitude = current_location.lat

for i in range(len(X)):
    print("Going towards point: %s, Battery level: %s" % (i, B[i]))
    point = LocationGlobalRelative(latitude + (X[i]*2/100000), longitude + (Y[i]*2/100000), 20)
    vehicle.simple_goto(point)
    time.sleep(1)

# print("Going towards first point for 30 seconds ...")

# point1 = LocationGlobalRelative(-35.361354, 149.165218, 20)
# vehicle.simple_goto(point1)

# # sleep so we can see the change in map
# time.sleep(30)

# print("Going towards second point for 30 seconds (groundspeed set to 10 m/s) ...")
# point2 = LocationGlobalRelative(-35.363244, 149.168801, 20)
# vehicle.simple_goto(point2, groundspeed=10)

# # sleep so we can see the change in map
# time.sleep(30)

# print("Returning to Launch")
# vehicle.mode = VehicleMode("RTL")

print("****************\nMISSION COMPLETED\n Final location: X: %s, Y: %s, and Battery: %s" % (X[i], Y[i], Z[i]))

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl:
    sitl.stop()

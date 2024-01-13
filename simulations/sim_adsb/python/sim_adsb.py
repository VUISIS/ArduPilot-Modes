#!/usr/bin/env python3.9
# -*- coding: utf-8 -*-

"""
Before executing this script make sure to set the following parameters in mavproxy
param set ADSB_TYPE 1 # enable ADSB signal listener
param set AVD_F_ACTION 3 # move horizontally

refer to https://ardupilot.org/copter/docs/parameters.html#avd-f-action

mission_basic.py: Example demonstrating basic mission operations including creating, clearing and monitoring missions.
Full documentation is provided at https://dronekit-python.readthedocs.io/en/latest/examples/mission_basic.html
"""

# TODO
# - set ADSB_TYPE parameter to 1
# - generate ADSB signals (replicate sim_adsb.c in Python)


from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil


#Set up option parsing to get connection string
import argparse  
parser = argparse.ArgumentParser(description='Demonstrates basic mission operations.')
parser.add_argument('--connect', 
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None


#Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


# Connect to the Vehicle
connection_string = 'udp:127.0.0.1:14550'
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=False)


def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned Location has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt)


def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5



def distance_to_current_waypoint():
    """
    Gets distance in metres to the current waypoint. 
    It returns None for the first waypoint (Home location).
    """
    nextwaypoint = vehicle.commands.next
    if nextwaypoint==0:
        return None
    missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint


def download_mission():
    """
    Download the current mission from the vehicle.
    """
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready() # wait until download is complete.



def adds_square_mission(aLocation, aSize):
    """
    Adds a takeoff command and four waypoint commands to the current mission. 
    The waypoints are positioned to form a square of side length 2*aSize around the specified LocationGlobal (aLocation).

    The function assumes vehicle.commands matches the vehicle mission state 
    (you must have called download at least once in the session and after clearing the mission)
    """	

    cmds = vehicle.commands

    print(" Clear any existing commands")
    cmds.clear() 
    
    print(" Define/add new commands.")
    # Add new commands. The meaning/order of the parameters is documented in the Command class. 
     
    #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))

    #Define the four MAV_CMD_NAV_WAYPOINT locations and add the commands
    point1 = get_location_metres(aLocation, aSize, -aSize)
    point2 = get_location_metres(aLocation, aSize, aSize)
    point3 = get_location_metres(aLocation, -aSize, aSize)
    point4 = get_location_metres(aLocation, -aSize, -aSize)

    # hard code the 4 coordinates
    # point1 = {}
    point1.lat = -35.36281294235794
    point1.lon = 149.16468672359426

    # point2 = {}
    point2.lat = -35.36281294235794
    point2.lon = 149.16578827640572

    # point3 = {}
    point3.lat = -35.36371125764206
    point3.lon = 149.16578827640572

    # point4 = {}
    point4.lat = -35.36371125764206
    point4.lon = 149.16468672359426


    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point1.lat, point1.lon, 11))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point2.lat, point2.lon, 12))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point3.lat, point3.lon, 13))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point4.lat, point4.lon, 14))
    #add dummy waypoint "5" at point 4 (lets us know when have reached destination)
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point4.lat, point4.lon, 14))    

    print(" Upload new commands to vehicle")
    cmds.upload()


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

        
    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:      
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)

# message documentation refer to
# https://mavlink.io/en/messages/common.html#ADSB_VEHICLE
def send_adsb_signal():
    # MAVLink_adsb_vehicle_message(ICAO_address, lat, lon, altitude_type, altitude, heading, hor_velocity, ver_velocity, callsign, emitter_type, tslc, flags, squawk)

    msg = vehicle.message_factory.adsb_vehicle_encode(
        # ICAO_address = 
        4000, 

        # lat = 
        int(-35.36277334 * 10000000.0),

        # lon =
        int(149.16536671 * 10000000.0),

        # altitude_type = 
        mavutil.mavlink.ADSB_ALTITUDE_TYPE_PRESSURE_QNH,

        # altitude
        604000,

        # heading
        0,

        # hor_velocity = 
        10,

        # ver_velocity = 
        0,

        # callsign: need to be bytes
        b'BOEING 737 MAX 9',

        # emitter type
        mavutil.mavlink.ADSB_EMITTER_TYPE_UAV,

        # tslc
        1,

        # flags 
        mavutil.mavlink.ADSB_FLAGS_VALID_COORDS |
        mavutil.mavlink.ADSB_FLAGS_VALID_ALTITUDE |
        mavutil.mavlink.ADSB_FLAGS_VALID_HEADING |
        mavutil.mavlink.ADSB_FLAGS_VALID_VELOCITY |
        mavutil.mavlink.ADSB_FLAGS_VALID_CALLSIGN |
        mavutil.mavlink.ADSB_FLAGS_VALID_SQUAWK |
        mavutil.mavlink.ADSB_FLAGS_SIMULATED |
        mavutil.mavlink.ADSB_FLAGS_VERTICAL_VELOCITY_VALID |
        mavutil.mavlink.ADSB_FLAGS_BARO_VALID,

        # squawk
        5000,
    ) # end of ADSB message transmitted
    vehicle.send_mavlink(msg)


# enable the drone to listen to the ADSB signal for surrounding intruding aircrafts

# simulate an intruding aircraft
send_adsb_signal()

print('Create a new mission (for current location)')
adds_square_mission(vehicle.location.global_frame,50)


# From Copter 3.3 you will be able to take off using a mission item. Plane must take off using a mission item (currently).
arm_and_takeoff(10)

print("Starting mission")
# Reset mission set to first (0) waypoint
vehicle.commands.next=0

# Set mode to AUTO to start mission
vehicle.mode = VehicleMode("AUTO")

# Monitor mission. 
# Demonstrates getting and setting the command number 
# Uses distance_to_current_waypoint(), a convenience function for finding the 
#   distance to the next waypoint.

# I attempted to set ADSB_TYPE parameter here
# however, it simply hangs and doesn't work
# Details refer to https://dronekit.netlify.app/guide/vehicle_state_and_parameters.html
# 
# Alternatively, you can set the parameters manually by using the commands below
# `param set ADSB_TYPE 1`
# vehicle.parameters['ADSB_TYPE'] = 1

while True:
    nextwaypoint=vehicle.commands.next
    print('Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint()))
    send_adsb_signal()
    if nextwaypoint==5: #Dummy waypoint - as soon as we reach waypoint 4 this is true and we exit.
        print("Exit 'standard' mission when start heading to final waypoint (5)")
        break;
    time.sleep(0.5)
    

print('Return to launch')
vehicle.mode = VehicleMode("RTL")
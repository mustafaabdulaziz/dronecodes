####DEPENDENCIES####
from dronekit import connect,VehicleMode,LocationGlobalRelative,APIException
import time
import socket
import exceptions
import math
import argparse
from pymavlink import mavutil

####Functions#####

def connectMyCopter():
	parser = argparse.ArgumentParser(description='commands')
	parser.add_argument('--connect')
	args = parser.parse_args()

	connection_string = args.connect


	if not connection_string:
		import dronekit_sitl
		sitl = dronekit_sitl.start_default()
		connection_string = sitl.connection_string


	vehicle = connect(connection_string,wait_ready=True)
	return vehicle


def arm_and_takeoff(targetHeight):
    while vehicle.is_armable!=True:
        print("wating for vehicle to become armable")
        time.sleep(1)

    print("vehicle is now armable")

    vehicle.mode = VehicleMode("GUIDED")


    while vehicle.mode!='GUIDED':
        print("wating for drone to enter GUIDED flight mode")
        time.sleep(1)
        
    print("vehicle enterd GUIDED mode")

    vehicle.armed = True

    while vehicle.armed==False:
        print("wating for vehicle to become armed")
        time.sleep(1)
        
    print("our drone armed now . Have fun!!")


    vehicle.simple_takeoff(targetHeight)            #metters
    
    while True:
        print("Current altitude: %d"%vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt>=.95*targetHeight:
            break
        time.sleep(1)
    print("Target alttitude reached!!! ")
    return None
    
    
def condition_yaw(degrees,relative):
    if relative:
        is_relative = 1
    else:
        is_relative = 0
    msg = vehicle.message_factory.command_long_encode(0,0,mavutil.mavlink.MAV_CMD_CONDITION_YAW,0,degrees,0,1,is_relative,0,0,0)
    vehicle.send_mavlink(msg)
    vehicle.flush()
    
def dummy_yaw_initilazer():
    lat = vehicle.location.global_relative_frame.lat
    lon = vehicle.location.global_relative_frame.lon
    alt = vehicle.location.global_relative_frame.alt
    
    aLocation = LocationGlobalRelative(lat,lon,alt)
    
    action= LocationGlobalRelative(lat,lon,alt)
    msg = vehicle.message_factory.set_position_target_global_int_encode(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,0b0000111111111000,aLocation.lat*1e7,aLocation.lon*1e7,aLocation.alt*1e7,0,0,0,0,0,0,0,0)
    vehicle.send_mavlink(msg)
    vehicle.flush()
    












####Main EXECUTABLE#####
    
vehicle = connectMyCopter()
arm_and_takeoff(10)
dummy_yaw_initilazer()

time.sleep(2)
print("intilaizing")
time.sleep(5)

condition_yaw(30,1)
print("yawing 30 degrees relative to current position")
time.sleep(7)


print("yawing true north")
condition_yaw(0,0) #yaw to true north
time.sleep(7)


print("yawing true west")
condition_yaw(270,0)
time.sleep(7)


while True:
    time.sleep(1)























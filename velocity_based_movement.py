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
    
def send_local_ned_velocity(vx,vy,vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(0,0,0,mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,0b0000111111000111,0,0,0,vx,vy,vz,0,0,0,0,0)
    vehicle.send_mavlink(msg)
    vehicle.flush()
    
    
def send_global_ned_velocity(vx,vy,vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(0,0,0,mavutil.mavlink.MAV_FRAME_LOCAL_NED,0b0000111111000111,0,0,0,vx,vy,vz,0,0,0,0,0)
    vehicle.send_mavlink(msg)
    vehicle.flush()

    
####Main EXECUTABLE#####
    
vehicle = connectMyCopter()
arm_and_takeoff(10)

counter = 0
while counter<5:
    send_local_ned_velocity(5,0,0)
    time.sleep(1)
    print("moving north relative to front of drone")
    counter = counter + 1
    
time.sleep(2)

counter = 0
while counter<5:
    send_local_ned_velocity(0,-5,0)
    time.sleep(1)
    print("moving west relative to front of drone")
    counter = counter + 1
    
time.sleep(2)

counter = 0
while counter<5:
    send_global_ned_velocity(5,0,0)
    time.sleep(1)
    print("moving true north ")
    counter = counter + 1
    
time.sleep(2)

counter = 0
while counter<5:
    send_global_ned_velocity(0,-5,0)
    time.sleep(1)
    print("moving true west")
    counter = counter + 1
    
time.sleep(2)

##### UP AND DOWN ######
counter = 0
while counter<5:
    send_local_ned_velocity(0,0,-5)
    time.sleep(1)
    print("moving UP")
    counter = counter + 1
    
time.sleep(2)


counter = 0
while counter<5:
    send_global_ned_velocity(0,0,5)
    time.sleep(1)
    print("moving DOWN")
    counter = counter + 1
    
time.sleep(2)


while True:
    time.sleep(1)






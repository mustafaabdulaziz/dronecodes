####DEPENDENCIES####
from dronekit import connect,VehicleMode,LocationGlobalRelative,APIException
import time
import socket
import exceptions
import math
import argparse

####Functions#####


	

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
    
    
####Main EXECUTABLE#####
    
vehicle = connectMyCopter()
arm_and_takeoff(10)
vehicle.mode = VehicleMode("RTL")
while vehicle.mode != 'RTL' :
    print("wating for drone to enter RTL mode")
    time.sleep(1)
    
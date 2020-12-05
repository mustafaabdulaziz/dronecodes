####DEPENDENCIES####
from dronekit import connect,VehicleMode,LocationGlobalRelative,APIException
import time
import socket
import exceptions
import math
import argparse

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
    



def get_distance_meters(targetLocation,currentLocation):
    dLat = targetLocation.lat - currentLocation.lat
    dLon = targetLocation.lon - currentLocation.lon
    
    return math.sqrt((dLon*dLon)+(dLat*dLat))*1.113195e5
    


def goto(targetLocation):
    distanceToTargetLocation = get_distance_meters(targetLocation,vehicle.location.global_relative_frame)
    vehicle.simple_goto(targetLocation)
    while vehicle.mode.name=='GUIDED':
        currentDistance = get_distance_meters(targetLocation,vehicle.location.global_relative_frame)
        if currentDistance<distanceToTargetLocation*0.01:
            print("Reached target waypoint ")
            time.sleep(2)
            break
        time.sleep(1)
    return None
    

####Main EXECUTABLE#####

wp1 = LocationGlobalRelative(44.50202,-88.060316,10)

 
vehicle = connectMyCopter()
arm_and_takeoff(10)


goto(wp1)

vehicle.mode = VehicleMode("RTL")
while vehicle.mode != 'RTL' :
    print("wating for drone to enter RTL mode")
    time.sleep(1)
    
while True:
    time.sleep(1)
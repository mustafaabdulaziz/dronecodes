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


vehicle = connectMyCopter()

#version and attributes

vehicle.wait_ready('autopilot_version')
print('autopilot_version: %s'%vehicle.version)

print('supports set attitude from companion: %s'%vehicle.capabilities.set_attitude_target_local_ned)

print('position: %s'%vehicle.location.global_relative_frame)

print('Attitude: %s'%vehicle.attitude)

print('velocity: %s'%vehicle.velocity)

print('Last Heartbeat: %s'%vehicle.last_heartbeat)

print('Is the vehicle armable: %s'%vehicle.is_armable)

print('Groundspeed: %s'%vehicle.groundspeed)

print('Mode: %s'%vehicle.mode.name)

print('Armed: %s'%vehicle.armed)

print('EKF ok: %s'%vehicle.ekf_ok)


print('battary: %s'%vehicle.battery)
vehicle.close()
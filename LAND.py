from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time

import argparse  
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14550')
args = parser.parse_args()

# Connect to the Vehicle
print 'Connecting to vehicle on: %s' % args.connect
vehicle = connect(args.connect, baud=115200, wait_ready=True)

while vehicle.armed = True and vehicle.mode != VehicleMode("GUIDED"):
    time.sleep(1)

if vehicle.mode == VehicleMode("GUIDED"):
    #Hover untuk 5 detik
    time.sleep(5)
    print("Now Let's Land")
    vehicle.mode = VehicleMode("LAND")

#close vehicle object
vehicle.close()
    

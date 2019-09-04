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

while vehicle.mode != VehicleMode("LOITER"):
    time.sleep(1)

if vehicle.mode == VehicleMode("LOITER"):
    vehicle.mode = VehicleMode("GUIDED")
    print"Change mode to Guided"
    time.sleep(5)
    vehicle.mode = VehicleMode("RTL")
    print"Change mode to RTL"
#close vehicle object
vehicle.close()

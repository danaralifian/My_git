
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time

# Parse connection args
import argparse  
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14550')
args = parser.parse_args()

# Connect to vehicle
print 'Connecting to vehicle on: %s' % args.connect
vehicle = connect(args.connect, baud=115200, wait_ready=True)

        
print "Arming motors"
    # Copter should arm in GUIDED mode
vehicle.mode = VehicleMode("STABILIZE")
vehicle.armed = True

#if vehicle.mode == VehicleMode("GUIDED"):
 #   vehicle.mode = VehicleMode("RTL")
  #  print "change mode LAND"

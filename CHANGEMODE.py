from dronekit import connect, VehicleMode

# Parse connection args
import argparse  
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14550')
args = parser.parse_args()

# Connect to vehicle
print 'Connecting to vehicle on: %s' % args.connect
vehicle = connect(args.connect, baud=57600, wait_ready=True)

# Change vehicle flight mode
print("Changing flight mode to stabilize")
vehicle.mode = VehicleMode("STABILIZE")
import setup_path
import airsim
import sys
import time
import os

from functools import partial
from mySensorData import mySensorData

print("""Fly on the streets of the Neighborhood environment while collecting sensor data""")
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
print("arming the drone...")
client.armDisarm(True)

# Initialize the sensor data collection module
os.chdir('/home/navlab-admin/AirSim-3D-MOT/data')
path = os.getcwd()
cam_folder = path + '/camera/'
lidar_folder = path + '/lidar/'
pose_folder = path + '/pose/'
sensors = mySensorData(client, 
                 compress_img = False, cam_folder = cam_folder, lidar_folder = lidar_folder, 
                 pose_folder = pose_folder)

state = client.getMultirotorState()
if state.landed_state == airsim.LandedState.Landed:
    print("taking off...")
    client.takeoffAsync().join()
else:
    client.hoverAsync().join()
time.sleep(1)
state = client.getMultirotorState()
if state.landed_state == airsim.LandedState.Landed:
    print("take off failed...")
    sys.exit(1)

# AirSim uses NED coordinates so negative axis is up.
# z of -7 is 7 meters above the original launch point.
z = -7
print("make sure we are hovering at 7 meters...")
client.moveToZAsync(z, 1).join()

# see https://github.com/Microsoft/AirSim/wiki/moveOnPath-demo
# this method is async and we are not waiting for the result since we are passing timeout_sec=0.
print("flying on path...")
result = client.moveOnPathAsync([airsim.Vector3r(125,0,z),
                                airsim.Vector3r(125,-130,z),
                                airsim.Vector3r(0,-130,z),
                                airsim.Vector3r(0,0,z)],
                        12, 120,
                        airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False,0), 20, 1)


for i in range(10):
    sensors.collectData("Drone0", get_cam_data = True, get_lidar_data = True, cam_num = i, lidar_num = i, pose_num = i)
    print(i)
    time.sleep(1)
    i += 1

result.join()


print("disarming...")
client.armDisarm(False)
client.reset()
client.enableApiControl(False)
print("done.")
import sys 
sys.path.append('/home/UE4/AirSim/PythonClient/cs231code/')

import setup_path 
import airsim

import os
from mySensorData import mySensorData

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
#client.reset()
client.armDisarm(True)
airsim.wait_key('Press any key to take an image and point cloud data')


os.chdir('/home/navlab-admin/AirSim-MOT/data')
datapath = os.getcwd()
base_folder = '/test/'
cam_folder = datapath + base_folder + '/camera/'
lidar_folder = datapath + base_folder + '/lidar/'
pose_folder = datapath + base_folder + '/pose/'
calib_folder = datapath + base_folder + '/processed/calib/'
try:
    os.mkdir(datapath + base_folder)
    os.mkdir(cam_folder)
    os.mkdir(lidar_folder)
    os.mkdir(pose_folder)
except OSError:
    print ("Creation of the directory failed")
else:
    print ("Successfully created the directory")
sensors = mySensorData(client, 
                 compress_img = False, cam_folder = cam_folder, lidar_folder = lidar_folder, 
                 pose_folder = pose_folder, calib_folder = calib_folder)

sensors.collectData(vehicle_name = 'Drone0', get_cam_data = True, get_lidar_data = True, get_calib_data = True)
sensors.collectData(vehicle_name = 'Drone1')

airsim.wait_key('Press any key to reset to original state')

client.armDisarm(False)
client.reset()

# that's enough fun for now. let's quit cleanly
client.enableApiControl(False)

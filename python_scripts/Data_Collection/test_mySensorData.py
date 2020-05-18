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


os.chdir('../CS231N_data')
path = os.getcwd()
cam_folder = path + '/camera/'
lidar_folder = path + '/lidar/'
pose_folder = path + '/pose/'
sensors = mySensorData(client, get_cam_data = True, get_lidar_data = True,  
                 compress_img = False, cam_folder = cam_folder, lidar_folder = lidar_folder, 
                 pose_folder = pose_folder)

sensors.collectData()

airsim.wait_key('Press any key to reset to original state')

client.armDisarm(False)
client.reset()

# that's enough fun for now. let's quit cleanly
client.enableApiControl(False)

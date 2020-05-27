import setup_path
import airsim
import pprint
import math
import random
import os

import sys
import time

from mySensorData import mySensorData

# Flies multiple drones through the streets of neighborhood
# assume all drones start in CC

num_drones = 10
z_list = [-20.2, -20.4, -20.6, -20.8, -21, -19.8, -19.6, -19.4, -19.2, -19]
start_locs = [[-4,0], [0,0], [2,0], [2,-2], [2,2], [0,-2], [0,2], [-2,-2], [-2,0], [-2,2]]
path_length = 15
timeout = 600

adj = {'NW': ['CW', 'NC'],
       'NC': ['NW', 'CC', 'NE'],
       'NE': ['NC', 'CE'],
       'CW': ['NW', 'CC', 'SW'],
       'CC': ['CW', 'NC', 'CE', 'SC'],
       'CE': ['CC', 'NE', 'SE'],
       'SW': ['CW', 'SC'],
       'SC': ['SW', 'CC', 'SE'],
       'SE': ['SC', 'CE']}

gridlen = 127.5

coords = {'NW': [gridlen, -gridlen],
          'NC': [gridlen, 0.0],
          'NE': [gridlen, gridlen],
          'CW': [0.0, -gridlen],
          'CC': [0.0, 0.0],
          'CE': [0.0, gridlen],
          'SW': [-gridlen, -gridlen],
          'SC': [-gridlen, 0.0],
          'SE': [-gridlen, gridlen]}

pathwidth = 1

preset = ['NC']

def gen_waypt(loc):
    coord = coords[loc]
    coord[0] += random.uniform(-pathwidth, pathwidth)
    coord[1] += random.uniform(-pathwidth, pathwidth)
    return coord

def gen_path(drone):
    state = ['CC', '']
    path = []
    for i in range(path_length):
        # prevent backtracking
        cands = adj[state[0]]
        if state[1] in cands:
            cands.remove(state[1])
        # choose new segment randomly
        state[1] = state[0]
        state[0] = random.choice(cands)
        #print(state[0], state[1])
        waypt = gen_waypt(state[0])
        path.append(airsim.Vector3r(waypt[0],waypt[1],z_list[drone]))
    return path

if __name__ == "__main__":

    
    client = airsim.MultirotorClient()
    client.confirmConnection()
    print("arming drones...")
    for i in range(num_drones):
        client.enableApiControl(True, "Drone"+str(i))
        client.armDisarm(True, "Drone"+str(i))

    print("initializing data collection...")
    os.chdir('/home/navlab-admin/AirSim-3D-MOT/data')
    path = os.getcwd()
    cam_folder = path + '/camera/'
    lidar_folder = path + '/lidar/'
    pose_folder = path + '/pose/'
    sensors = mySensorData(client, 
                    compress_img = False, cam_folder = cam_folder, lidar_folder = lidar_folder, 
                    pose_folder = pose_folder)

    print("taking off...")
    cmds = []
    for i in range(num_drones):
        cmd = client.takeoffAsync(vehicle_name="Drone"+str(i))
        cmds.append(cmd)
    for cmd in cmds:
        cmd.join()

    print("ascending to hover altitude")
    cmds = []
    for i in range(num_drones):
        cmd = client.moveToZAsync(z_list[i], 1, vehicle_name="Drone"+str(i))
        cmds.append(cmd)
    for cmd in cmds:
        cmd.join()
    
    print("flying routes")
    speed = 9
    cmds = []
    # generate path
    path = gen_path(0)
    for i in reversed(range(num_drones)):
        
        # move command
        cmd = client.moveOnPathAsync(path, speed, timeout, airsim.DrivetrainType.ForwardOnly, 
                        airsim.YawMode(False,0), 20, 1, vehicle_name="Drone"+str(i))
        cmds.append(cmd)
        # time.sleep(0.2)

    for i in range(100):
        sensors.collectData("Drone0", get_cam_data = True, get_lidar_data = True, cam_num = i, lidar_num = i, pose_num = i)
        print(i)
        time.sleep(1)
        i += 1
    
    for cmd in cmds:
        cmd.join()


    print("disarming...")
    for i in range(num_drones):
        client.armDisarm(False, "Drone"+str(i+1))

    print("resetting")
    client.reset()

    for i in range(num_drones):
        client.enableApiControl(False, "Drone"+str(i+1))
    print("done.")
    
    




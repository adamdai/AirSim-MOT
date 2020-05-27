import setup_path
import airsim
import pprint
import math
import random

import sys
import time

# Flies multiple drones through the streets of neighborhood
# assume all drones start in CC

num_drones = 9
#z_list = [-8, -7, -6, -5, -4, -9, -10, -11, -12]
z_list = [-20, -19, -18, -17, -16, -21, -22, -23, -24]
start_locs = [[0,0], [2,0], [2,-2], [2,2], [0,-2], [0,2], [-2,-2], [-2,0], [-2,2]]
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
        waypt[0] -= start_locs[drone][0]
        waypt[1] -= start_locs[drone][1]
        path.append(airsim.Vector3r(waypt[0],waypt[1],z_list[drone]))
    return path

if __name__ == "__main__":

    #path = gen_path(-5)

    
    client = airsim.MultirotorClient()
    client.confirmConnection()
    print("arming drones...")
    for i in range(num_drones):
        client.enableApiControl(True, "Drone"+str(i+1))
        client.armDisarm(True, "Drone"+str(i+1))

    print("taking off...")
    cmds = []
    for i in range(num_drones):
        cmd = client.takeoffAsync(vehicle_name="Drone"+str(i+1))
        cmds.append(cmd)
    for cmd in cmds:
        cmd.join()

    print("ascending to hover altitude")
    cmds = []
    for i in range(num_drones):
        cmd = client.moveToZAsync(z_list[i], 1, vehicle_name="Drone"+str(i+1))
        cmds.append(cmd)
    for cmd in cmds:
        cmd.join()
    
    print("flying routes")
    speed = 9
    cmds = []
    for i in reversed(range(num_drones)):
        # generate path
        path = gen_path(i)
        # move command
        cmd = client.moveOnPathAsync(path, speed, timeout, airsim.DrivetrainType.ForwardOnly, 
                        airsim.YawMode(False,0), 20, 1, vehicle_name="Drone"+str(i+1))
        cmds.append(cmd)
        time.sleep(0.2)
    
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
    
    




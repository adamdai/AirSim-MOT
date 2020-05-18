import setup_path 
import airsim

import numpy as np
import os
import tempfile
import pprint
import cv2
import time
import math
import pickle

class mySensorData:

    def __init__(self, client, get_cam_data = True, get_lidar_data = True,  
                 compress_img = False, cam_folder = None, lidar_folder = None, 
                 pose_folder = None):
        self.client = client
        self.cam_folder = cam_folder # folder to store camera data
        self.lidar_folder = lidar_folder # folder to store lidar data
        self.pose_folder = pose_folder # folder to store the pose data
        self.compress_img = compress_img
        self.get_cam_data = get_cam_data
        self.get_lidar_data = get_lidar_data

    def collectData(self, cam_num = 0, lidar_num = 0, pose_num = 0):
        if self.get_cam_data:
            self.genImg(cam_num)
        if self.get_lidar_data:
            self.genPC(lidar_num)
        self.genPose(pose_num)

    def genImg(self, im_num): # stores the camera data in folder cam_folder, with name having idx
        responses = self.getImg()
        filename_img = self.cam_folder + ("img%.6d.png" % im_num)
        filename_ts = self.cam_folder + ("ts%.6d,txt" % im_num)
        for idx, response in enumerate(responses):
            if self.compress_img: #png format
                airsim.write_file(os.path.normpath(filename_img), response.image_data_uint8)
            else: #uncompressed array
                img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8)
                img_rgb = img1d.reshape(response.height, response.width, 3)
                cv2.imwrite(os.path.normpath(filename_img), img_rgb)
            output = open(os.path.normpath(filename_ts), 'wb')
            pickle.dump(response.time_stamp, output)
            output.close()
    
    def genPC(self, im_num): # stores the lidar data in folder lidar_folder, with name having idx
        points, time_stamp = self.getPC()
        filename_pc = self.lidar_folder + ("scan%.6d.txt" % im_num)
        filename_ts = self.lidar_folder + ("ts%.6d.txt" % im_num)
        op1 = open(os.path.normpath(filename_pc), 'wb')
        op2 = open(os.path.normpath(filename_ts), 'wb')
        pickle.dump(points, op1)
        pickle.dump(time_stamp,op2)
        op1.close()
        op2.close()
    
    def genPose(self, im_num):
        state = self.client.getMultirotorState()
        filename_pose = self.pose_folder + ("pose%.6d.txt" % im_num)
        op1 = open(os.path.normpath(filename_pose), 'wb')
        pickle.dump(state,op1)
        op1.close()

    def getImg(self): # utility for genImg
        if self.compress_img:
            responses = self.client.simGetImages([
                            airsim.ImageRequest("front_cam", airsim.ImageType.Scene)]) 
        else:
            responses = self.client.simGetImages([
                            airsim.ImageRequest("front_cam", airsim.ImageType.Scene, False, False)])
        return responses

    def getPC(self): # utility for genPC
        lidar_data = self.client.getLidarData()
        time_stamp = lidar_data.time_stamp
        lidar_pose = lidar_data.pose
        if (len(lidar_data.point_cloud) >= 3):
            points = np.array(lidar_data.point_cloud, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0]/3), 3))

        return points, time_stamp


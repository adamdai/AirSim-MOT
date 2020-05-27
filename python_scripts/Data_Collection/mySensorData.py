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

from convert_calib import write_to_label_file
from convert_pose import get_transform


class mySensorData:

    def __init__(self, client,  
                 compress_img = False, cam_folder = None, lidar_folder = None, 
                 pose_folder = None, calib_folder = None):
        self.client = client
        self.cam_folder = cam_folder # folder to store camera data
        self.lidar_folder = lidar_folder # folder to store lidar data
        self.pose_folder = pose_folder # folder to store the pose data
        self.calib_folder = calib_folder # folder to store the calib data
        self.compress_img = compress_img

    def collectData(self, vehicle_name, 
			get_cam_data = False, get_lidar_data = False, get_calib_data = False,
			 cam_num = 0, lidar_num = 0, pose_num = 0, calib_num = 0):
        lidar_pose = [None]
        camera_info = [None]
        if get_cam_data:
            camera_info = self.genImg(cam_num, vehicle_name)
        if get_lidar_data:
            lidar_pose = self.genPC(lidar_num, vehicle_name)
        if get_calib_data:
            self.genCalib(calib_num, vehicle_name, lidar_pose, camera_info)
        self.genPose(pose_num, vehicle_name)

    def genImg(self, im_num, vehicle_name): # stores the camera data in folder cam_folder, with name having idx
        responses = self.getImg(vehicle_name)
        pprint.pprint(responses[0].camera_position)
        pprint.pprint(responses[0].camera_orientation)
        filename_img = self.cam_folder + ("%.6d.png" % im_num)
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
        camera_info = self.client.simGetCameraInfo(vehicle_name=vehicle_name)
        return camera_info
    
    def genPC(self, im_num, vehicle_name): # stores the lidar data in folder lidar_folder, with name having idx
        points, lidar_pose, time_stamp = self.getPC(vehicle_name)
        filename_pc = self.lidar_folder + ("scan%.6d.txt" % im_num)
        filename_ts = self.lidar_folder + ("ts%.6d.txt" % im_num)
        op1 = open(os.path.normpath(filename_pc), 'wb')
        op2 = open(os.path.normpath(filename_ts), 'wb')
        pickle.dump(points, op1)
        pickle.dump(time_stamp,op2)
        op1.close()
        op2.close()
        return lidar_pose
    
    def genPose(self, im_num, vehicle_name):
        state = self.client.getMultirotorState(vehicle_name = vehicle_name)
        filename_pose = self.pose_folder + vehicle_name + ("pose%.6d.txt" % im_num)
        op1 = open(os.path.normpath(filename_pose), 'wb')
        pickle.dump(state,op1)
        op1.close()

    def getImg(self, vehicle_name): # utility for genImg
        if self.compress_img:
            responses = self.client.simGetImages([
                            airsim.ImageRequest("front_cam", airsim.ImageType.Scene)], vehicle_name = vehicle_name) 
        else:
            responses = self.client.simGetImages([
                            airsim.ImageRequest("front_cam", airsim.ImageType.Scene, False, False)], vehicle_name=vehicle_name)
        return responses

    def getPC(self, vehicle_name): # utility for genPC
        k = 50
        points_list = []
        for i in range(k):
            lidar_data = self.client.getLidarData(vehicle_name = vehicle_name)
            time_stamp = lidar_data.time_stamp
            lidar_pose = lidar_data.pose

            if (len(lidar_data.point_cloud) >= 3):
                points = np.array(lidar_data.point_cloud, dtype=np.dtype('f4'))
                points = np.reshape(points, (int(points.shape[0]/3), 3))
                points_list.append(points)
            time.sleep(0.001)
        
        points_all = np.vstack((points_list[0],points_list[1]))
        for i in range(2,k):
            points_all = np.vstack((points_all,points_list[i]))
        
        pprint.pprint(points_all.shape)

        return points_all, lidar_pose, time_stamp

    def genCalib(self, calib_num, vehicle_name, lidar_pose, camera_info):
        lt = get_transform(lidar_pose)
        ct = get_transform(camera_info.pose, True)
        v2c = np.dot(ct,lt)
        i2v = np.linalg.inv(lt)
        R0 = np.eye(3)
        P2 = np.asarray(camera_info.proj_mat)
        P0 = P2
        P1 = P2
        P3 = P2
        write_to_label_file(P0, P1, P2, P3, R0, v2c, i2v, self.calib_folder+("%.6d.txt" % calib_num))

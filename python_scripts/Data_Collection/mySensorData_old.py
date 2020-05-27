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
from convert_pose import get_transform, quaternion_to_eul, eul_to_rotmat


class mySensorData:

    def __init__(self, client,  
                 compress_img = False, cam_folder = None, lidar_folder = None, 
                 pose_folder = None, calib_folder = None, label_folder = None):
        self.client = client
        self.cam_folder = cam_folder # folder to store camera data
        self.lidar_folder = lidar_folder # folder to store lidar data
        self.pose_folder = pose_folder # folder to store the pose data
        self.calib_folder = calib_folder # folder to store the calib data
        self.compress_img = compress_img
        self.label_folder = label_folder

    def collectData(self, vehicle_name, 
			get_cam_data = False, get_lidar_data = False, get_calib_data = False,
			 cam_num = 0, lidar_num = 0, pose_num = 0, calib_num = 0):
        lidar_pose = [None]
        camera_info = [None]
        if get_cam_data:
            camera_info = self.genImg(cam_num, vehicle_name)
            # pprint.pprint(camera_info)
        if get_lidar_data:
            lidar_pose,  lidar_points = self.genPC(lidar_num, vehicle_name)
            # pprint.pprint(lidar_pose)
        if get_calib_data:
            P2, R0, v2c = self.genCalib(calib_num, vehicle_name, lidar_pose, camera_info)
        # self.test_mats(P2,R0,v2c,lidar_points)
        self.genPose(pose_num, vehicle_name)

    def genImg(self, im_num, vehicle_name): # stores the camera data in folder cam_folder, with name having idx
        responses = self.getImg(vehicle_name)
        # pprint.pprint(responses[0].camera_position)
        # pprint.pprint(responses[0].camera_orientation)
        filename_img = self.cam_folder + ("%.6d.png" % im_num)
        filename_ts = self.pose_folder + ("cam_ts%.6d,txt" % im_num)
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
        camera_info = self.client.simGetCameraInfo("front_cam", vehicle_name=vehicle_name)
        filename_cam_pose = self.pose_folder + vehicle_name + ("campose%.6d.txt" % im_num)
        op1 = open(os.path.normpath(filename_cam_pose), 'wb')
        pickle.dump(camera_info,op1)
        op1.close()
        return camera_info
    
    def genPC(self, im_num, vehicle_name): # stores the lidar data in folder lidar_folder, with name having idx
        points, lidar_pose, time_stamp = self.getPC(vehicle_name)
        filename_pc = self.lidar_folder + ("%.6d.bin" % im_num)
        filename_ts = self.pose_folder + ("velo_ts%.6d.txt" % im_num)
        # op1 = open(os.path.normpath(filename_pc), 'wb')
        op2 = open(os.path.normpath(filename_ts), 'wb')
        # pickle.dump(points, op1)
        pickle.dump(time_stamp,op2)
        # op1.close()
        op2.close()
        n_points = points.shape[0]
        min_points = 1e10
        if n_points < min_points:
            min_points = n_points
        final_points = np.hstack((points, np.ones((n_points,1), dtype=np.dtype('f4'))))
        # lt = get_transform(lidar_pose, True)
        # final_points_t = np.dot(final_points, ) 
        final_points.tofile(filename_pc)
        return lidar_pose, final_points
    
    def genPose(self, im_num, vehicle_name):
        state = self.client.getMultirotorState(vehicle_name = vehicle_name)
        pprint.pprint(vehicle_name + " position: " + str(state.kinematics_estimated.position))
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
        k = 1
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
        
        if k == 1:
            points_all = points_list[0]
        else:
            points_all = np.vstack((points_list[0],points_list[1]))
            for i in range(2,k):
                points_all = np.vstack((points_all,points_list[i]))
        
        pprint.pprint('lidar points: ' + str(points_all.shape))

        return points_all, lidar_pose, time_stamp

    def genCalib(self, calib_num, vehicle_name, lidar_pose, camera_info):
        lt = get_transform(lidar_pose)
        ct = get_transform(camera_info.pose, True)
        # pprint.pprint(camera_info)
        trans_possible = np.array([[0,1,0,0],[0,0,1,0],[1,0,0,0],[0,0,0,1]])
        # ct = np.dot(trans_possible,ct)
        v2c = np.dot(trans_possible,np.dot(ct,lt))[0:3,:]
        # v2c = np.dot(ct,lt)[0:3,:]
        # v2c = np.dot(ct,lt)
        # print(v2c)
        i2v = np.linalg.inv(lt)[0:3,:] # this is wrong
        R0 = np.eye(3)
        # R0 = np.array([[0,1,0,0],[0,0,1,0],[1,0,0,0],[0,0,0,1]])
        # print(camera_info.proj_mat.matrix)
        P2 = np.asarray(camera_info.proj_mat.matrix)[0:3,:]
        # set camera center to image center (W/2, H/2)
        P2[0,3] = 960
        P2[1,3] = 540
        P2[1,2] = np.abs(P2[1,2])
        # P2 = np.hstack((np.asarray(camera_info.proj_mat.matrix)[0:3,1:4],np.zeros((3,1))))
        # print(P2)
        P0 = P2
        P1 = P2
        P3 = P2
        write_to_label_file(P0, P1, P2, P3, R0, v2c, i2v, self.calib_folder+("%.6d.txt" % calib_num))
        return P2, R0, v2c

    def saveLabel(self, start_locs, id, camera_info, pos2, label_num, obj_name):
        ct = get_transform(camera_info.pose, inv = True)

        trans_possible = np.array([[0,1,0,0],[0,0,1,0],[1,0,0,0],[0,0,0,1]])
        ct = np.dot(trans_possible,ct)

        ## T = [R | t
        ##      0 | 1]

        # T1 = get_transform(pos1, inv = True, isDronePose = True)
        T2 = get_transform(pos2, inv = False, isDronePose = True)
        t1 = np.append((start_locs[id,:] - start_locs[0,:]),0)
        init_transformmat = np.vstack((np.hstack((np.eye(3),np.reshape(t1,(3,1)))),[0,0,0,1]))

        
        # transformi = make_transform(np.eye(3),t1)
        final_mat = np.dot(ct, np.dot(init_transformmat,T2))

        data = np.zeros((14,))
        data[0] = 0  # truncation goes from 0 to 1
        data[1] = int(0)  # integer (0,1,2,3) indicating occlusion state:
                        # 0 = fully visible, 1 = partly occluded
                        # 2 = largely occluded, 3 = unknown
        data[3:7] = [0.0,0.0,0.0,0.0]    # bbox 2D bounding box of object in the image (0-based index):
                        # contains left, top, right, bottom pixel coordinates
        h = 0.5
        w = 1.5
        l = 1.5
        data[7:10] = [h,w,l]                 # dimensions 3D object dimensions: height, width, length (in meters)
        # location 3D object location x,y,z in camera coordinates (in meters) 
        # and rotation_y Rotation ry around Y-axis in camera coordinates [-pi..pi]
        
        # data[10:13] = final_mat[0:3,3]
        data[10:13] = np.dot(final_mat, np.reshape(np.array([0,0,h/2,1]),(4,1)))[0:3,0]

        z_b2 = final_mat[2,3]
        x_b2 = final_mat[0,3]
        data[13] = - np.arctan2(final_mat[2,0], final_mat[0,0])
        heading1 = np.arctan2(z_b2,x_b2)
        alpha = heading1 - data[13] - np.pi/2
        data[2] = alpha    # alpha Observation angle of object, ranging [-pi..pi]
        
        if np.linalg.norm(data[10:13]) <= 25:
            f=open(self.label_folder+("%.6d.txt" % label_num),'w')
            f.write(obj_name + ' ')
            np.savetxt(f,np.reshape(data,(1,14)),fmt='%0.4f')
            f.close()
        else:
            f=open(self.label_folder+("%.6d.txt" % label_num),'w')
            f.close()

    def read_pickle_file(self,fname):
        drone0 = open(fname, 'rb')
        content = pickle.load(drone0)
        drone0.close()
        return content

    def test_mats(self,P2,R0,v2c,lidar_points):
        temp1 = np.dot(lidar_points,np.transpose(v2c))
        temp2 = np.transpose(np.dot(R0,np.transpose(temp1)))
        temp3 = np.hstack((temp2,np.ones((temp2.shape[0],1))))

        # P2 = np.hstack((P2[0:3,1:4],np.zeros((3,1))))

        final = np.dot(temp3,np.transpose(P2))

        final[:, 0] /= final[:, 2]
        final[:, 1] /= final[:, 2]
        
        fov_inds = (lidar_points[:, 0] > 1.0)
        final_2d = final[fov_inds, 0:2]
        np.savetxt('test.txt', final_2d, delimiter=',') 
        print('Did it!!!')
        # pts = np.transpose(lidar_points)
        # temp1 = np.dot(v2c,pts)
        # temp2 = np.dot(R0,temp1)
        # # for i in range(temp2.shape[1]):
        # #     if temp2[2,i] < 0:
        # #         temp2[:,i] = np.zeros((3,))
        
        # P2[0,2] = 1920/2
        # P2[1,2] = 1080/2
        # P2[1,1] = np.abs(P2[1,1])
        # print(P2)
        # op = np.dot(P2, temp2)

        # fov_inds = (pts[0, :] > 2.0)
        # op_fov = op[:,fov_inds]
                
        # # for i in range(op.shape[1]):
        # #     op[0,i] /= op[2,i]
        # #     op[1,i] /= op[2,i]
        # #     op[2,i] /= op[2,i] 
        # np.savetxt('test.txt', op_fov, delimiter=',') 






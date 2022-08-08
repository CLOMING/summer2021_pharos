#!/usr/bin/env python3
# [RAW] : PKG Dependency
from __future__ import print_function
import rospy
from std_msgs.msg import String

# SYSTEM
import argparse
from math import log10, ceil
from os.path import join, exists, isfile, realpath, dirname
from os import makedirs, remove, chdir, environ
import random, shutil, json, time

# PyTorch
# import torch
# import torch.nn as nn
# import torch.nn.functional as F
# import torch.optim as optim
# import torch.onnx
# from torch.autograd import Variable
# from torch.utils.data import DataLoader, SubsetRandomSampler
# from torch.utils.data.dataset import Subset
# import torchvision.transforms as transforms
# import torchvision.datasets as datasets
# import torchvision.models as models

# THB
import cv2
# import pcl
# import h5py
import time
import math
import faiss
import numpy as np
from PIL import Image
from datetime import datetime
import matplotlib.pyplot as plt
import matplotlib as mpl
from matplotlib import cm
from collections import OrderedDict
from skimage.transform import resize
# from tensorboardX import SummaryWriter
from cv_bridge import CvBridge, CvBridgeError
import open3d as o3d
import open3d.core as o3c
# from open3d_ros_helper import open3d_ros_helper as orh

# ROS
import ros_numpy
from ros_numpy import numpy_msg
import message_filters
from nav_msgs.msg import Odometry 
from sensor_msgs.msg import Image 
from sensor_msgs.msg import PointCloud2 
from std_msgs.msg import Float32,Float32MultiArray,Float64,Float64MultiArray,MultiArrayLayout,MultiArrayDimension
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
# [RAW] : PARSER Argument
class Localization_PARAM():
    def __init__(self):
        self.mode =                      'test'
        self.batchSize =                 4
        self.cacheBatchSize =            24
        self.cacheRefreshRate =          1000
        self.nEpochs =                   30
        self.start_epoch =               0
        self.nGPU =                      1
        self.optim =                     'ADAM'
        self.lr =                        0.0001
        self.lrStep =                    5
        self.lrGamma =                   0.5
        self.weightDecay =               0.001
        self.momentum=                   0.9
        self.nocuda =                    False
        self.threads =                   8
        self.seed =                      123
        self.savePath =                  'checkpoints'
        self.ckpt =                      'best'
        self.evalEvery =                 1
        self.patience =                  10
        self.dataset =                   'urban'
        self.arch =                      'resnet18'
        self.vladv2 =                    False
        self.pooling =                   'netvlad'
        self.num_clusters =              64
        self.margin =                    0.1
        self.split =                     'val'
        self.fromscratch =               True

        # HARD CODED -> [Future Works] : Change with ROS Param
        self.dataPath =                  '/home/iismn/WorkSpace/CU11_DL/ROS/src/RESEARCH_PACK/OSM_NetVLAD/src/data/'
        self.runsPath =                  '/home/iismn/WorkSpace/CU11_DL/ROS/src/RESEARCH_PACK/OSM_NetVLAD/src/runs/'
        self.cachePath =                 '/home/iismn/WorkSpace/CU11_DL/ROS/src/RESEARCH_PACK/OSM_NetVLAD/src/cache/'
        self.resume =                    '/home/iismn/WorkSpace/CU11_DL/ROS/src/RESEARCH_PACK/OSM_NetVLAD/src/runs/8K_1K_ResNet18_NetVLAD'

        self.MAP_range =                 rospy.get_param('AGV_Local_Module/ACC_DIST', 100)
        self.iter =                      0

class Localization_MAIN():
    def __init__(self):

        self.rviz_linelist = Marker()
        self.rviz_linelist.ns = "bounding_Box_Py"
        self.rviz_linelist.id = 1

        self.rviz_linelist.type = Marker.LINE_LIST   #Marker.POINTS
        self.rviz_linelist.action = Marker.ADD    #Marker.ADD

        self.rviz_linelist.color = ColorRGBA(0, 1, 0, 1)
        self.rviz_linelist.scale.x = 0.05

        # [ROS] ROS Set Initializeing -------------------------------------------------
        print('[  ROS  ] >> START')

        self.ros_sub_obstacle = rospy.get_param('~sub_map_topic', '/lidar_pcl/obstacle/filteredPCL')
        self.ros_pub_obstacle = rospy.get_param('~pub_map_topic', '/lidar_pcl/obstacle/detectedPCL_Py')
        self.ros_pub_obstacle_marker = rospy.get_param('~pub_map_topic', '/lidar_pcl/obstacle/detectedBOX_Py')
        self.init_sub = rospy.Subscriber(self.ros_sub_obstacle, PointCloud2, self.obstacle_CB, queue_size = 5)
        self.init_pub = rospy.Publisher(self.ros_pub_obstacle, PointCloud2, queue_size = 1)
        self.marker_pub = rospy.Publisher(self.ros_pub_obstacle_marker, Marker, queue_size=1)

        print('[  ROS  ] >> SPIN')
        rospy.spin()

    def obstacle_CB(self, obstacle_PCL):
        
        obstacle_np = ros_numpy.numpify(obstacle_PCL)

        points=np.zeros((obstacle_np.shape[0],3))
        points[:,0]=obstacle_np['x'].reshape(1,-1)
        points[:,1]=obstacle_np['y'].reshape(1,-1)
        points[:,2]=obstacle_np['z'].reshape(1,-1)

        idx = np.where(points[:,2] <= 1.5)
        points = points[idx][:]
        # Open 3D - GPU-Processing
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        downpcd = pcd.voxel_down_sample(voxel_size=0.3)

        labels = np.array(downpcd.cluster_dbscan(eps=1.0, min_points=5, print_progress=False))
        try:
            max_label = labels.max()
            print("[",obstacle_PCL.header.stamp.to_sec(), f"] Cluster : {max_label + 1} ")
            isLabel = True
        except:
            print("[",obstacle_PCL.header.stamp.to_sec(), f"] DBSCAN Cluster None ")  
            isLabel = False

        if isLabel == True:
            colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
            colors[labels < 0] = 0
            pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])

            xyz_load = np.asarray(downpcd.points)
            rgb_load = np.asarray(pcd.colors)

            rgb_load = rgb_load*255
            rgb_load = rgb_load.astype(np.uint32)
            # print(rgb_load)
            ROS_temp = np.zeros(xyz_load.shape[0], dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('rgb', np.uint32)])

            ROS_temp['x'] = xyz_load[:, 0]
            ROS_temp['y'] = xyz_load[:, 1]
            ROS_temp['z'] = xyz_load[:, 2]
            r = rgb_load[:, 0]
            g = rgb_load[:, 1]
            b = rgb_load[:, 2]
            rgb_arr = np.array((r << 16) | (g << 8) | (b << 0), dtype=np.uint32)

            ROS_temp['rgb'] = rgb_arr
            ROS_MCL_Msg = ros_numpy.msgify(PointCloud2, ROS_temp)
            ROS_MCL_Msg.header = obstacle_PCL.header
            self.init_pub.publish(ROS_MCL_Msg)
            
            rviz_linelist_Reset = Marker()

            self.rviz_linelist.header.frame_id = obstacle_PCL.header.frame_id
            self.rviz_linelist.points = rviz_linelist_Reset.points
            self.rviz_linelist.pose.orientation.w = 1.0;
            for i in range(max_label+1):
                
                idx = np.where(labels == i)
                obstacleD = ROS_temp[idx][:]

                obstacle_pt=np.zeros((obstacleD.shape[0],3))
                obstacle_pt[:,0]=obstacleD['x'].reshape(1,-1)
                obstacle_pt[:,1]=obstacleD['y'].reshape(1,-1)
                obstacle_pt[:,2]=obstacleD['z'].reshape(1,-1)

                # Open 3D - GPU-Processing ------------------------------------------------------------
                pcdt = o3d.geometry.PointCloud()
                pcdt.points = o3d.utility.Vector3dVector(obstacle_pt)

                aabb = pcdt.get_axis_aligned_bounding_box()
                min_PT = aabb.get_max_bound()
                max_PT = aabb.get_min_bound()

                # print(min_PT[2]-max_PT[2])

                if min_PT[2]-max_PT[2] > 0.3 and np.linalg.norm(max_PT-min_PT) < 5:
                    self.boundbox_FN(min_PT, max_PT)
                
                # if i == max_label-1:
                #     break
            
            self.marker_pub.publish(self.rviz_linelist)



    def boundbox_FN(self, min_PT, max_PT):
            boudingBox = np.empty([8,3])
            boudingBox[0,0:3] = np.array([min_PT[0], min_PT[1], min_PT[2]])
            boudingBox[1,0:3] = np.array([min_PT[0], min_PT[1], max_PT[2]])
            boudingBox[2,0:3] = np.array([max_PT[0], min_PT[1], max_PT[2]])
            boudingBox[3,0:3] = np.array([max_PT[0], min_PT[1], min_PT[2]])

            boudingBox[4,0:3] = np.array([min_PT[0], max_PT[1], min_PT[2]])
            boudingBox[5,0:3] = np.array([min_PT[0], max_PT[1], max_PT[2]])
            boudingBox[6,0:3] = np.array([max_PT[0], max_PT[1], max_PT[2]])
            boudingBox[7,0:3] = np.array([max_PT[0], max_PT[1], min_PT[2]])
            

            for i in range(3):
                self.rviz_linelist.points.append(Point(boudingBox[i,0], boudingBox[i,1], boudingBox[i,2]))    
                self.rviz_linelist.points.append(Point(boudingBox[i+1,0], boudingBox[i+1,1], boudingBox[i+1,2]))  

            self.rviz_linelist.points.append(Point(boudingBox[3,0], boudingBox[3,1], boudingBox[3,2]))    
            self.rviz_linelist.points.append(Point(boudingBox[0,0], boudingBox[0,1], boudingBox[0,2]))  

            for i in range(3):
                i = i+4
                self.rviz_linelist.points.append(Point(boudingBox[i,0], boudingBox[i,1], boudingBox[i,2]))    
                self.rviz_linelist.points.append(Point(boudingBox[i+1,0], boudingBox[i+1,1], boudingBox[i+1,2]))  

            self.rviz_linelist.points.append(Point(boudingBox[7,0], boudingBox[7,1], boudingBox[7,2]))    
            self.rviz_linelist.points.append(Point(boudingBox[4,0], boudingBox[4,1], boudingBox[4,2])) 

            for i in range(4):
                self.rviz_linelist.points.append(Point(boudingBox[i,0], boudingBox[i,1], boudingBox[i,2]))    
                self.rviz_linelist.points.append(Point(boudingBox[i+4,0], boudingBox[i+4,1], boudingBox[i+4,2]))  


if __name__ == '__main__':
    rospy.init_node("obstacle_filter_o3d")
    localizer_PARM = Localization_PARAM()
    localizer_DL = Localization_MAIN()

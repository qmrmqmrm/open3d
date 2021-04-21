#!/home/j/.pyenv/versions/ros_py36/bin/python3

import rospy
from cv_bridge import CvBridge, CvBridgeError
import pyrealsense2 as rs
import cv2
import tf
import numpy as np
# from std_msgs.msg import Int32
from sensor_msgs.msg import Image, PointCloud2
import open3d as o3d
# import numpy
import matplotlib.pyplot as plt
from open3d_ros_helper import open3d_ros_helper as orh


print("Read Redwood dataset")
color_raw = o3d.io.read_image("/home/j/catkin_ws/src/Open3D/examples/test_data/RGBD/color/00000.jpg")
print(type(color_raw))
print(color_raw.dimension())

depth_raw = o3d.io.read_image("/home/j/catkin_ws/src/Open3D/examples/test_data/RGBD/depth/00000.png")
print(type(depth_raw))
rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
    color_raw, depth_raw)
print(rgbd_image)
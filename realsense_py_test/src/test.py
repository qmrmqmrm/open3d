#!/home/j/.pyenv/versions/ros_py36/bin/python3

import rospy
from cv_bridge import CvBridge, CvBridgeError
import pyrealsense2 as rs
import cv2
import tf
import numpy as np
from std_msgs.msg import Int32
from sensor_msgs.msg import Image, PointCloud2, Imu
import open3d as o3d
import matplotlib.pyplot as plt
from open3d_ros_helper import open3d_ros_helper as orh
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped
import math


class Merge_IMU:
    def __init__(self, pre_time):
        rospy.Subscriber("/camera/gyro/sample", Imu, self.gyro_callback)
        rospy.Subscriber("/camera/accel/sample", Imu, self.accel_callback)
        self.gyro = Imu()
        self.accel = Imu()
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.x = 0
        self.y = 0
        self.z = 0

        self.pre_time = pre_time

    def gyro_callback(self, msgs):
        self.gyro = msgs

    def accel_callback(self, msgs):
        self.accel = msgs

    def merge_gyro_accel(self):
        br = tf.TransformBroadcaster()
        rospy.Subscriber("/camera/gyro/sample", Imu, self.gyro_callback)
        rospy.Subscriber("/camera/accel/sample", Imu, self.accel_callback)
        # print("------------------------------------------")
        # print(self.gyro,"\n\n", self.accel)
        d_t = rospy.Time.now().to_time() - self.pre_time.to_time()
        self.pre_time = rospy.Time.now()
        # print(d_t)
        self.roll += self.gyro.angular_velocity.x * d_t
        self.pitch += self.gyro.angular_velocity.y * d_t
        self.yaw += self.gyro.angular_velocity.z * d_t
        self.x += self.accel.linear_acceleration.x * d_t
        self.y += (self.accel.linear_acceleration.y+9.7478) * d_t
        self.z += self.accel.linear_acceleration.z * d_t

        x, y, z, w = tf.transformations.quaternion_from_euler(self.roll, self.pitch, self.yaw)
        print(f"self.x : {self.x :0.4f}, self.y : {self.y:.4f},self.z : {self.z:.4f}")
        print(f"self.x : {self.accel.linear_acceleration.x :0.4f}, self.y : {self.accel.linear_acceleration.y:.4f},"
              f"self.z : {self.accel.linear_acceleration.z:.4f}")

        br.sendTransform((self.x , self.y,self.z),
                         (x, y, z, w),
                         rospy.Time.now(),
                         "camera_depth_cutting",
                         "camera_link")
        x, y, z, w = tf.transformations.quaternion_from_euler(-np.pi / 2, 0, -np.pi / 2)
        br.sendTransform((0, 0, 0),
                         (x, y, z, w),
                         rospy.Time.now(),
                         "camera_link",
                         "cam")


def main():
    rospy.init_node("draw_aruco_axis")
    pre_time = rospy.Time.now()
    imu = Merge_IMU(pre_time)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        imu.merge_gyro_accel()

        rate.sleep()


if __name__ == '__main__':
    main()

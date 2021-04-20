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


# # class Call_back:
# #     def __init__(self):
# #         self.depth_img = None


def depth_callback(point):
    ros_transform = orh.rospc_to_o3dpc(point)
    # br = tf.TransformBroadcaster()
    # br.sendTransform((0, 0, 0),
    #                  (0, 0, 0, 1),
    #                  rospy.Time.now(),
    #                  "camera_depth_cutting",
    #                  "camera_link")

    # print(f"\nheight : {point.height}\n"
    #       f"width  : {point.width}\n"
    #       f"point_step : {point.point_step}\n"
    #       f"header : {point.header}\n")
    # ind = np.where(dists > 0.01)[0]
    # o3d.visualization.draw_geometries([ros_transform])
    z = np.asarray([[0.0, 0.0, 0.0]])

    zero = o3d.geometry.PointCloud()
    zero.points = o3d.utility.Vector3dVector(z)
    zero.paint_uniform_color([1, 0, 0])
    dists = ros_transform.compute_point_cloud_distance(zero)
    dists = np.asarray(dists)
    ind = np.where(dists < 4.01)[0]
    pcd_without_chair = ros_transform.select_by_index(ind)

    # o3d.io.write_point_cloud("/home/j/catkin_ws/src/test_point2.pcd",ros_transform)
    # o3d.visualization.draw_geometries([ros_transform])
    # o3d.visualization.draw_geometries([pcd_without_chair, zero])
    # print(pcd_without_chair.points)
    ros_msg = orh.o3dpc_to_rospc(pcd_without_chair)
    ros_msg.header.frame_id = "camera_depth_cutting"
    pub = rospy.Publisher('test_point2',PointCloud2,queue_size=1)
    # print(f"\nros_msg\nheight : {ros_msg.height}\n"
    #       f"width  : {ros_msg.width}\n"
    #       f"point_step : {ros_msg.point_step}\n"
    #       f"header : {ros_msg.header}\n")
    pub.publish(ros_msg)



def main():
    rospy.init_node("real_depth")
    # callback = Call_back()
    pub_right = rospy.Subscriber('/camera/depth/color/points', PointCloud2, depth_callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

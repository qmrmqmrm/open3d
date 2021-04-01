#!/home/j/.pyenv/versions/ros_py36/bin/python3

import rospy
from cv_bridge import CvBridge, CvBridgeError
import pyrealsense2 as rs
import cv2
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
    # ind = np.where(dists > 0.01)[0]
    #o3d.visualization.draw_geometries([ros_transform])
    z = np.asarray([[0.0, 0.0, 0.0], [1.0, 1.0, 1.0], [0.0, 1.0, 1.0], [1.0, 0.0, 1.0], [1.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]])

    dec_filter = rs.decimation_filter()  # Decimation - reduces depth frame density
    spat_filter = rs.spatial_filter()  # Spatial    - edge-preserving spatial smoothing
    temp_filter = rs.temporal_filter()  # Temporal   - reduces temporal noise
    # frames = pipe.wait_for_frames()
    # depth_frame = frames.get_depth_frame()
    filtered = dec_filter.process(depth_frame)
    filtered = spat_filter.process(filtered)
    filtered = temp_filter.process(filtered)
    zero = o3d.geometry.PointCloud()
    zero.points = o3d.utility.Vector3dVector(z)
    zero.paint_uniform_color([1, 0, 0])
    # o3d.io.write_point_cloud("/home/j/catkin_ws/src/test_point2.pcd",ros_transform)
    # o3d.visualization.draw_geometries([ros_transform,zero])
def main():
    rospy.init_node("real_depth")
    # callback = Call_back()

    pub_right = rospy.Subscriber('/camera/depth/color/points', PointCloud2,depth_callback)
    rospy.spin()
        
if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

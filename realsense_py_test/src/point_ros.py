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

class Ros_Point:
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.config.enable_stream(rs.stream.depth, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, rs.format.bgr8, 30)

        # Start streaming
        self.pipeline.start(self.config)

        # Get stream profile and camera intrinsics
        profile = self.pipeline.get_active_profile()
        depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
        depth_intrinsics = depth_profile.get_intrinsics()
        w, h = depth_intrinsics.width, depth_intrinsics.height

        # Processing blocks
        self.pc = rs.pointcloud()
        self.decimate = rs.decimation_filter()
        self.decimate.set_option(rs.option.filter_magnitude, 2 ** 1)
        self.colorizer = rs.colorizer()

    def make_ros_point(self):
        # Grab camera data

        # Wait for a coherent pair of frames: depth and color
        frames = self.pipeline.wait_for_frames()

        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        depth_frame = self.decimate.process(depth_frame)

        # Grab new intrinsics (may be changed by decimation)
        depth_intrinsics = rs.video_stream_profile(
            depth_frame.profile).get_intrinsics()
        w, h = depth_intrinsics.width, depth_intrinsics.height

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        depth_colormap = np.asanyarray(
            self.colorizer.colorize(depth_frame).get_data())

        mapped_frame, color_source = color_frame, color_image

        points = self.pc.calculate(depth_frame)
        self.pc.map_to(mapped_frame)

        # Pointcloud data to arrays
        v, t = points.get_vertices(), points.get_texture_coordinates()
        # print(v, t)
        verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)  # xyz
        texcoords = np.asanyarray(t).view(np.float32).reshape(-1, 2)  # uv
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(verts)

        # test_img = o3d.cpu.pybind.geometry.Image(verts)
        z = np.asarray([[0.0, 0.0, 0.0]])
        zero = o3d.geometry.PointCloud()
        zero.points = o3d.utility.Vector3dVector(z)
        zero.paint_uniform_color([1, 0, 0])

        dists = pcd.compute_point_cloud_distance(zero)
        dists = np.asarray(dists)
        ind = np.where(dists < 4.01)[0]
        pcd = pcd.select_by_index(ind)

        print(pcd)
        # test_dep = o3d.cpu.pybind.geometry.Image(texcoords)
        # print(test_img)
        ros_msg = orh.o3dpc_to_rospc(pcd)
        ros_msg.header.frame_id = "camera_depth_cutting"
        pub = rospy.Publisher('test_point2', PointCloud2, queue_size=1)
        pub.publish(ros_msg)

def main():
    rospy.init_node("real_depth")
    point = Ros_Point()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        point.make_ros_point()
        rate.sleep()
    # callback = Call_back()
    # pub_right = rospy.Subscriber('/camera/depth/color/points', PointCloud2, make_ros_point)
    # rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pipeline.stop()

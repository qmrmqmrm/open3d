#!/home/j/.pyenv/versions/ros_py36/bin/python3
import math

import cv2
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
import pyrealsense2 as rs
import rospy
from open3d_ros_helper import open3d_ros_helper as orh
from sensor_msgs.msg import PointCloud2


class AppState:
    def __init__(self):
        self.WIN_NAME = 'RealSense'
        self.pitch, self.yaw = math.radians(-10), math.radians(-15)
        self.translation = np.array([0, 0, -1], dtype=np.float32)
        self.distance = 2
        self.spatial = 2
        self.prev_mouse = 0, 0
        self.mouse_btns = [False, False, False]
        self.paused = False
        self.decimate = 1
        self.scale = True
        self.color = True
    def reset(self):
        self.pitch, self.yaw, self.distance = 0, 0, 2
        self.translation[:] = 0, 0, -1

    @property
    def rotation(self):
        Rx, _ = cv2.Rodrigues((self.pitch, 0, 0))
        Ry, _ = cv2.Rodrigues((0, self.yaw, 0))
        return np.dot(Ry, Rx).astype(np.float32)

    @property
    def pivot(self):
        return self.translation + np.array((0, 0, self.distance), dtype=np.float32)

class Ros_Point:
    def __init__(self):
        self.state =AppState()
        self.pipeline = rs.pipeline()
        CONFIG = rs.config()

        CONFIG.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        CONFIG.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        self.pipeline.start(CONFIG)

        # Get stream profile and camera intrinsics
        profile = self.pipeline.get_active_profile()
        depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
        depth_intrinsics = depth_profile.get_intrinsics()

        w, h = depth_intrinsics.width, depth_intrinsics.height
        self.out = np.empty((h, w, 3), dtype=np.uint8)
        # Processing blocks
        self.pc = rs.pointcloud()

        # spatial o ,colorizer,disparity, temporal o ,hole_filling o ,decimation o ,hdr_merge
        self.decimate = rs.decimation_filter()
        self.spatial = rs.spatial_filter()
        self.temporal = rs.temporal_filter()
        self.threshold = rs.threshold_filter()
        hold = rs.hole_filling_filter()

        self.decimate.set_option(rs.option.filter_magnitude, 2 ** self.state.decimate)

        self.spatial.set_option(rs.option.filter_magnitude, 2 ** self.state.spatial)
        self.spatial.set_option(rs.option.filter_smooth_alpha, 0.30)
        self.spatial.set_option(rs.option.filter_smooth_delta, 40)

        self.temporal.set_option(rs.option.filter_smooth_alpha, 0.2)
        self.temporal.set_option(rs.option.filter_smooth_delta, 25)

        self.threshold.set_option(rs.option.min_distance, 0.01)
        self.threshold.set_option(rs.option.max_distance, 5.0)

        self.colorizer = rs.colorizer()

    def make_ros_point(self):
        # Grab camera data

        # Wait for a coherent pair of frames: depth and color
        frames = self.pipeline.wait_for_frames()

        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        depth_frame = self.threshold.process(depth_frame)
        depth_frame = self.decimate.process(depth_frame)
        depth_frame = self.temporal.process(depth_frame)
        # depth_frame = self.hold.process(depth_frame)
        depth_frame = self.spatial.process(depth_frame)
        # Grab new intrinsics (may be changed by decimation)
        depth_intrinsics = rs.video_stream_profile(
            depth_frame.profile).get_intrinsics()
        w, h = depth_intrinsics.width, depth_intrinsics.height

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        # print(f"depth_image.shape{depth_image.shape}")
        # print(f"color_image.shape{color_image.shape}")

        depth_colormap = np.asanyarray(
            self.colorizer.colorize(depth_frame).get_data())

        if self.state.color:
            mapped_frame, color_source = color_frame, color_image
        else:
            mapped_frame, color_source = depth_frame, depth_colormap
        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_source.shape

        resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]),
                                         interpolation=cv2.INTER_AREA)
        # print(resized_color_image.shape)
        points = self.pc.calculate(depth_frame)
        self.pc.map_to(mapped_frame)

        # Pointcloud data to arrays
        v, t = points.get_vertices(), points.get_texture_coordinates()
        verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)  # xyz
        texcoords = np.asanyarray(t).view(np.float32).reshape(-1, 2)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(verts)
        depth_reshape = np.reshape(resized_color_image, (-1, 3))
        # pcd.colors = o3d.utility.Vector3dVector(depth_reshape)
        # print(pcd.has_colors())

        with o3d.utility.VerbosityContextManager(
                o3d.utility.VerbosityLevel.Debug) as cm:
            labels = np.array(
                pcd.cluster_dbscan(eps=0.02, min_points=10, print_progress=True))

        max_label = labels.max()
        print(f"point cloud has {max_label + 1} clusters")
        colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
        colors[labels < 0] = 0
        pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])

        plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,
                                                 ransac_n=3,
                                                 num_iterations=1000)
        [a, b, c, d] = plane_model
        print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

        inlier_cloud = pcd.select_by_index(inliers)
        inlier_cloud.paint_uniform_color([1.0, 0, 0])
        # outlier_cloud = pcd.select_by_index(inliers, invert=True)
        # print(pcd.colors)
        # o3d.visualization.draw_geometries([pcd])

        # o3d.visualization.draw_geometries([pcd])

        ros_msg = orh.o3dpc_to_rospc(pcd)
        ros_msg.header.frame_id = "map"
        pub = rospy.Publisher('test_point2', PointCloud2, queue_size=1)
        pub.publish(ros_msg)

    def view(self, v):
        """apply view transformation on vector array"""
        return np.dot(v - self.state.pivot, self.state.rotation) + self.state.pivot - self.state.translation

    def project(self, v):
        """project 3d vector array to 2d"""
        h, w = self.out.shape[:2]
        view_aspect = float(h) / w

        # ignore divide by zero for invalid depth
        with np.errstate(divide='ignore', invalid='ignore'):
            proj = v[:, :-1] / v[:, -1, np.newaxis] * \
                   (w * view_aspect, h) + (w / 2.0, h / 2.0)

        # near clipping
        znear = 0.03
        proj[v[:, 2] < znear] = np.nan
        return proj

    def pointcloud(self, out, verts, texcoords, color, painter=True):
        """draw point cloud with optional painter's algorithm"""
        if painter:

            # Painter's algo, sort points from back to front

            # get reverse sorted indices by z (in view-space)
            # https://gist.github.com/stevenvo/e3dad127598842459b68
            v = self.view(verts)
            s = v[:, 2].argsort()[::-1]
            proj = self.project(v[s])
        else:
            proj = self.project(self.view(verts))

        if self.state.scale:
            proj *= 0.5 ** self.state.decimate

        h, w = out.shape[:2]

        # proj now contains 2d image coordinates
        j, i = proj.astype(np.uint32).T

        # create a mask to ignore out-of-bound indices
        im = (i >= 0) & (i < h)
        jm = (j >= 0) & (j < w)
        m = im & jm

        cw, ch = color.shape[:2][::-1]
        if painter:
            # sort texcoord with same indices as above
            # texcoords are [0..1] and relative to top-left pixel corner,
            # multiply by size and add 0.5 to center
            v, u = (texcoords[s] * (cw, ch) + 0.5).astype(np.uint32).T
        else:
            v, u = (texcoords * (cw, ch) + 0.5).astype(np.uint32).T
        # clip texcoords to image
        np.clip(u, 0, ch - 1, out=u)
        np.clip(v, 0, cw - 1, out=v)

        # perform uv-mapping
        out[i[m], j[m]] = color[u[m], v[m]]


def main():
    rospy.init_node("real_depth")
    point = Ros_Point()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        point.make_ros_point()
        rate.sleep()
    point.pipeline.stop()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

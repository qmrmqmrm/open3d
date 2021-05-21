import cv2
import numpy as np
import open3d as o3d
import pyrealsense2 as rs

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

config.enable_stream(rs.stream.depth, rs.format.z16, 30)
config.enable_stream(rs.stream.color, rs.format.bgr8, 30)
pipeline.start(config)
decimate = rs.decimation_filter()
spatial = rs.spatial_filter()
temporal = rs.temporal_filter()
threshold = rs.threshold_filter()
hold = rs.hole_filling_filter()

decimate.set_option(rs.option.filter_magnitude, 6)

spatial.set_option(rs.option.filter_magnitude, 2)
spatial.set_option(rs.option.filter_smooth_alpha, 0.30)
spatial.set_option(rs.option.filter_smooth_delta, 40)

temporal.set_option(rs.option.filter_smooth_alpha, 0.2)
temporal.set_option(rs.option.filter_smooth_delta, 25)

threshold.set_option(rs.option.min_distance, 0.01)
threshold.set_option(rs.option.max_distance, 2.0)
try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        depth_frame = threshold.process(depth_frame)
        depth_frame = decimate.process(depth_frame)
        depth_frame = temporal.process(depth_frame)
        # depth_frame = self.hold.process(depth_frame)
        depth_frame = spatial.process(depth_frame)

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape
        print(color_colormap_dim)
        print(depth_colormap_dim)

        # If depth and color resolutions are different, resize color image to match depth image for display
        if depth_colormap_dim != color_colormap_dim:
            resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]),
                                             interpolation=cv2.INTER_AREA)
            images = np.hstack((resized_color_image, depth_colormap))
            test_img = o3d.cpu.pybind.geometry.Image(resized_color_image)
            test_dep = o3d.cpu.pybind.geometry.Image(depth_colormap)
            # print(type(depth_colormap),depth_colormap.shape)
        else:
            images = np.hstack((color_image, depth_colormap))
            test_img = o3d.cpu.pybind.geometry.Image(color_image)
            test_dep = o3d.cpu.pybind.geometry.Image(depth_colormap)

        # print(images.shape)

        # print(test_dep)
        # #
        # # # color_raw = o3d.io.read_image(depth_colormap)
        # # # depth_raw = o3d.io.read_image(images)
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(test_img, test_dep)
        #

        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd_image,
            o3d.camera.PinholeCameraIntrinsic(
                o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))

        o3d.visualization.draw_geometries([pcd])
        # # # Show images
        # # plt.subplot(1, 2, 1)
        # plt.title('Redwood grayscale image')
        # plt.imshow(images)
        # # plt.subplot(1, 2, 2)
        # # plt.title('Redwood depth image')
        # # plt.imshow(test_dep)
        # plt.show()

finally:

    # Stop streaming
    pipeline.stop()

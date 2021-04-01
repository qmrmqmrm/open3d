import pyrealsense2 as rs
pipe = rs.pipeline()
# cfg = rs.config()
# cfg.enable_stream(rs.stream.Depth, 640, 480, rs.format.Z16, 30);
pipe.start()
# Declare filters
dec_filter = rs.decimation_filter()   # Decimation - reduces depth frame density
spat_filter = rs.spatial_filter()          # Spatial    - edge-preserving spatial smoothing
temp_filter = rs.temporal_filter()    # Temporal   - reduces temporal noise
frames = pipe.wait_for_frames()
depth_frame = frames.get_depth_frame()
filtered = dec_filter.process(depth_frame)
filtered = spat_filter.process(filtered)
filtered = temp_filter.process(filtered)
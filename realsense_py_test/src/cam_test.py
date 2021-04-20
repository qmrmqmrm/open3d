#!/home/j/.pyenv/versions/ros_py36/bin/python3
# -*- coding: utf-8 -*-
import sys, os
import open3d as o3d
# from open3d import open3d_tutorial
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import matplotlib.pyplot as plt


# only needed for tutorial, monkey patches visualization
sys.path.append('/home/j/catkin_ws/src/Open3D/examples/python/')
import open3d_tutorial as o3dtut
o3dtut.interactive = not "CI" in os.environ

def main():
    print("Load a ply point cloud, print it, and render it")
    pcd = o3d.io.read_point_cloud("/home/j/catkin_ws/src/Open3D/examples/test_data/fragment.ply")
    print(pcd)
    print(np.asarray(pcd.points))
    o3d.visualization.draw_geometries([pcd],
                                    zoom=0.3412,
                                    front=[0.4257, -0.2125, -0.8795],
                                    lookat=[2.6172, 2.0475, 1.532],
                                    up=[-0.0694, -0.9768, 0.2024])

    print("Downsample the point cloud with a voxel of 0.05")
    downpcd = pcd.voxel_down_sample(voxel_size=0.05)
    o3d.visualization.draw_geometries([downpcd],
                                    zoom=0.3412,
                                    front=[0.4257, -0.2125, -0.8795],
                                    lookat=[2.6172, 2.0475, 1.532],
                                    up=[-0.0694, -0.9768, 0.2024])


    print("Recompute the normal of the downsampled point cloud")
    downpcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    o3d.visualization.draw_geometries([downpcd],
                                    zoom=0.3412,
                                    front=[0.4257, -0.2125, -0.8795],
                                    lookat=[2.6172, 2.0475, 1.532],
                                    up=[-0.0694, -0.9768, 0.2024],
                                    point_show_normal=True)
    print("Print a normal vector of the 0th point")
    print(downpcd.normals[0])   
    print("Print the normal vectors of the first 10 points")
    print(np.asarray(downpcd.normals)[:10, :])

    print("Load a polygon volume and use it to crop the original point cloud")
    vol = o3d.visualization.read_selection_polygon_volume(
        "/home/j/catkin_ws/src/Open3D/examples/test_data/Crop/cropped.json")
    chair = vol.crop_point_cloud(pcd)
    o3d.visualization.draw_geometries([chair],
                                    zoom=0.7,
                                    front=[0.5439, -0.2333, -0.8060],
                                    lookat=[2.4615, 2.1331, 1.338],
                                    up=[-0.1781, -0.9708, 0.1608])
    print("Paint chair")
    chair.paint_uniform_color([1, 0.706, 0])
    o3d.visualization.draw_geometries([chair],
                                    zoom=0.7,
                                    front=[0.5439, -0.2333, -0.8060],
                                    lookat=[2.4615, 2.1331, 1.338],
                                    up=[-0.1781, -0.9708, 0.1608])
    pcl = o3dtut.get_bunny_mesh().sample_points_poisson_disk(number_of_points=2000)
    hull, _ = pcl.compute_convex_hull()
    hull_ls = o3d.geometry.LineSet.create_from_triangle_mesh(hull)
    hull_ls.paint_uniform_color((1, 0, 0))
    o3d.visualization.draw_geometries([pcl, hull_ls])
    # Load data
    pcd = o3d.io.read_point_cloud("/home/j/catkin_ws/src/Open3D/examples/test_data/fragment.ply")
    vol = o3d.visualization.read_selection_polygon_volume(
        "/home/j/catkin_ws/src/Open3D/examples/test_data/Crop/cropped.json")
    chair = vol.crop_point_cloud(pcd)

    dists = pcd.compute_point_cloud_distance(chair)
    dists = np.asarray(dists)
    ind = np.where(dists > 0.01)[0]
    pcd_without_chair = pcd.select_by_index(ind)
    o3d.visualization.draw_geometries([pcd_without_chair],
                                    zoom=0.3412,
                                    front=[0.4257, -0.2125, -0.8795],
                                    lookat=[2.6172, 2.0475, 1.532],
                                    up=[-0.0694, -0.9768, 0.2024])
    aabb = chair.get_axis_aligned_bounding_box()
    aabb.color = (1, 0, 0)
    obb = chair.get_oriented_bounding_box()
    obb.color = (0, 1, 0)
    o3d.visualization.draw_geometries([chair, aabb, obb],
                                    zoom=0.7,
                                    front=[0.5439, -0.2333, -0.8060],
                                    lookat=[2.4615, 2.1331, 1.338],
                                    up=[-0.1781, -0.9708, 0.1608])
    # pcl = o3dtut.get_bunny_mesh().sample_points_poisson_disk(number_of_points=2000)
    # hull, _ = pcl.compute_convex_hull()
    # hull_ls = o3d.geometry.LineSet.create_from_triangle_mesh(hull)
    # hull_ls.paint_uniform_color((1, 0, 0))
    # o3d.visualization.draw_geometries([pcl, hull_ls])

    pcd = o3d.io.read_point_cloud("/home/j/catkin_ws/src/Open3D/examples/test_data/fragment.ply")

    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(
            pcd.cluster_dbscan(eps=0.02, min_points=10, print_progress=True))

    max_label = labels.max()
    print(f"point cloud has {max_label + 1} clusters")
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
    o3d.visualization.draw_geometries([pcd],
                                    zoom=0.455,
                                    front=[-0.4999, -0.1659, -0.8499],
                                    lookat=[2.1813, 2.0619, 2.0999],
                                    up=[0.1204, -0.9852, 0.1215])
    

    pcd = o3d.io.read_point_cloud("/home/j/catkin_ws/src/Open3D/examples/test_data/fragment.pcd")
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,
                                            ransac_n=3,
                                            num_iterations=1000)
    [a, b, c, d] = plane_model
    print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

    inlier_cloud = pcd.select_by_index(inliers)
    inlier_cloud.paint_uniform_color([1.0, 0, 0])
    outlier_cloud = pcd.select_by_index(inliers, invert=True)
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud],
                                    zoom=0.8,
                                    front=[-0.4999, -0.1659, -0.8499],
                                    lookat=[2.1813, 2.0619, 2.0999],
                                    up=[0.1204, -0.9852, 0.1215])

    print("Convert mesh to a point cloud and estimate dimensions")

    print("Convert mesh to a point cloud and estimate dimensions")
    pcd = o3dtut.get_armadillo_mesh().sample_points_poisson_disk(5000)
    diameter = np.linalg.norm(
        np.asarray(pcd.get_max_bound()) - np.asarray(pcd.get_min_bound()))
    o3d.visualization.draw_geometries([pcd])
    
    print("Define parameters used for hidden_point_removal")
    camera = [0, 0, diameter]
    radius = diameter * 100

    print("Get all points that are visible from given view point")
    _, pt_map = pcd.hidden_point_removal(camera, radius)

    print("Visualize result")
    pcd = pcd.select_by_index(pt_map)
    o3d.visualization.draw_geometries([pcd])

if __name__ == "__main__":
    main()

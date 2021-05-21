#!/home/j/.pyenv/versions/ros_py36/bin/python3

import copy

import numpy as np
# from std_msgs.msg import Int32
import open3d as o3d
import rospy


# import numpy


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp],
                                      zoom=0.4459,
                                      front=[0.9288, -0.2951, -0.2242],
                                      lookat=[1.6784, 2.0612, 1.4451],
                                      up=[-0.3402, -0.9189, -0.1996])


# rotation matrix 생성
def create_rotation_matrix(euler):
    (yaw, pitch, roll) = euler

    yaw_matrix = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])

    pitch_matrix = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    roll_matrix = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])

    rotation_matrix = yaw_matrix @ pitch_matrix @ roll_matrix

    return rotation_matrix


def main():
    a_ = np.random.rand(2000).reshape(1000, 2) * 10
    a = np.concatenate((a_, np.zeros((1000, 1))), axis=1)
    b = np.random.rand(1500).reshape(500, 3) * 5
    c = np.concatenate((a, b))
    pcd = o3d.io.read_point_cloud("out.ply")
    pcd2 = o3d.io.read_point_cloud("out2.ply")
    z = np.asarray([[0.0, 0.0, 0.0]])

    zero = o3d.geometry.PointCloud()
    zero.points = o3d.utility.Vector3dVector(z)
    zero.paint_uniform_color([1, 0, 0])

    dists = pcd.compute_point_cloud_distance(zero)
    dists = np.asarray(dists)
    ind = np.where(dists < 4.01)[0]
    pcd = pcd.select_by_index(ind)

    dists2 = pcd2.compute_point_cloud_distance(zero)
    dists2 = np.asarray(dists2)
    ind = np.where(dists2 < 4.01)[0]
    pcd2 = pcd2.select_by_index(ind)

    print(pcd.points)
    source = o3d.geometry.PointCloud()
    source.points = o3d.utility.Vector3dVector(c)
    r = [1.55432151, -2.54513158, 1.2530725805869145]
    rotation = create_rotation_matrix(r)

    t = np.array([[2, 3, 4]])
    q = (rotation @ (c.T)).T + t  # + (np.random.rand(12000).reshape(4000, 3) * 0.1)

    z = np.asarray([[0.0, 0.0, 0.0], [10.0, 10.0, 10.0], [0.0, 10.0, 10.0], [10.0, 0.0, 10.0], [10.0, 10.0, 0.0],
                    [0.0, 0.0, 10.0], [0.0, 10.0, 0.0], [10.0, 0.0, 0.0]])

    target = o3d.geometry.PointCloud()
    target.points = o3d.utility.Vector3dVector(q)
    zero = o3d.geometry.PointCloud()
    zero.points = o3d.utility.Vector3dVector(z)
    source.paint_uniform_color([1, 0, 0])
    target.paint_uniform_color([0, 1, 0])
    zero.paint_uniform_color([0, 0, 0])
    o3d.visualization.draw_geometries([pcd, pcd2, zero])

    threshold = 1
    zero_init = np.eye(4)

    d_max = True
    count = 0

    while d_max:
        # for n in range(100):
        # sig = o3d.pipelines.registration.TukeyLoss(k=0.1)
        # print("zero_init",zero_init)
        reg_p2p = o3d.pipelines.registration.registration_icp(pcd, pcd2, threshold, zero_init,
                                                              o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                                                              o3d.pipelines.registration.ICPConvergenceCriteria(
                                                                  max_iteration=2000))
        cor = np.asarray(reg_p2p.correspondence_set).shape
        pcd.transform(reg_p2p.transformation)
        trans = reg_p2p.transformation
        marge_t = zero_init @ reg_p2p.transformation
        zero_init = reg_p2p.transformation

        if cor[0] >= 1000:
            d_max = False
        else:
            count += 1
            if count > 100:
                print("fall")
                d_max = False

    o3d.visualization.draw_geometries([pcd, pcd2, zero])


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

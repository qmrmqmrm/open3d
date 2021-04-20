#!/home/j/.pyenv/versions/ros_py36/bin/python3

import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
np.set_printoptions(formatter={'float_kind': lambda x: "{0:0.3f}".format(x)})

# from std_msgs.msg import Int32
from sensor_msgs.msg import Image, PointCloud2
import open3d as o3d
import copy
# import numpy
import matplotlib.pyplot as plt
from open3d_ros_helper import open3d_ros_helper as orh


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


def make_transformation_matrix(r_matrix, t_matrix):
    print(r_matrix)
    print(t_matrix)
    matrix_3x4 = np.concatenate((r_matrix, t_matrix), axis=1)
    zero_one = np.array([[0., 0., 0., 1.]])
    matrix_4x4 = np.concatenate((matrix_3x4, zero_one), axis=0)
    return matrix_4x4


def main():
    a_ = np.random.rand(2000).reshape(1000, 2) * 100
    a = np.concatenate((a_, np.ones((1000, 1))), axis=1)
    print(a.shape)

    r = [1.55, -2.04, 1.25]
    rotation = create_rotation_matrix(r)

    t = np.array([[5, 7, -5]])
    q = (rotation @ (a.T)).T #+ t # + (np.random.rand(3000).reshape(1000, 3) * 0.5)
    # print(q.shape, type(q))

    target = o3d.geometry.PointCloud()
    target.points = o3d.utility.Vector3dVector(q)
    test_vector = q[0] -q[-1]
    test_v = q[50] - q [-1]

    print(test_vector)
    test_vector_d = np.linalg.norm(test_vector)

    n_ = make_plane(q)
    test_vec = test_vector_d * n_
    print(test_vec)
    n_t = n_.T
    p = n_t
    # print(n_t.shape)
    for i in range(100):
        # print((i+1)*n_t)
        neee =(i+1)*n_t
        p = np.concatenate((p,neee))

        # print(f"p.shape{p.shape}")

    z = np.asarray([[0.0, 0.0, 0.0]])
    z = np.concatenate((z,p))
    # print(z)

    zero = o3d.geometry.PointCloud()
    zero.points = o3d.utility.Vector3dVector(z)
    target.paint_uniform_color([0, 1, 0])
    zero.paint_uniform_color([0, 0, 0])
    o3d.visualization.draw_geometries([target, zero],point_show_normal=True)


def make_plane(point):
    shape = point.shape
    one_vecter = np.ones((shape[0], 1))
    # print(type(one_vecter),type(shape),type(point))
    # print(one_vecter.shape, shape, point.shape,point.T.shape,point.T @ point)
    n = np.linalg.inv(point.T @ point) @ point.T @ one_vecter
    print(f"n : {n}")
    point_norm = np.linalg.norm(n)
    norm_vector = n / point_norm
    return norm_vector


if __name__ == '__main__':
    main()

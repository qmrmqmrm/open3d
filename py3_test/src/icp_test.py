#!/home/j/.pyenv/versions/ros_py36/bin/python3

import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
# from std_msgs.msg import Int32
from sensor_msgs.msg import Image, PointCloud2
import open3d as o3d
import copy
# import numpy
import matplotlib.pyplot as plt
from open3d_ros_helper import open3d_ros_helper as orh


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


def make_transformation_matrix(r_matrix, t_matrix):
    print(r_matrix)
    print(t_matrix)
    matrix_3x4 = np.concatenate((r_matrix, t_matrix), axis=1)
    zero_one = np.array([[0., 0., 0., 1.]])
    matrix_4x4 = np.concatenate((matrix_3x4, zero_one), axis=0)
    return matrix_4x4


def main():
    # rospy.init_node("icp_test")
    # callback = Call_back()
    a_ = np.random.rand(2000).reshape(1000, 2) * 10
    # print(a_,np.zeros((1000,1)))
    a = np.concatenate((a_, np.zeros((1000, 1))), axis=1)
    b = np.random.rand(1500).reshape(500, 3) * 5
    # b = np.concatenate((np.zeros((500, 1)),b_), axis=1)
    c = np.concatenate((a,b))
    print(c)
    source = o3d.geometry.PointCloud()
    source.points = o3d.utility.Vector3dVector(c)
    r = [1.55432151, -2.54513158, 1.2530725805869145]
    rotation = create_rotation_matrix(r)

    t = np.array([[2, 3, 4]])
    m44 = make_transformation_matrix(rotation, t.T)
    print(m44)
    # print(a ,rotation.shape)
    z = np.asarray([[0.0, 0.0, 0.0], [10.0, 10.0, 10.0], [0.0, 10.0, 10.0], [10.0, 0.0, 10.0], [10.0, 10.0, 0.0],
                    [0.0, 0.0, 10.0], [0.0, 10.0, 0.0], [10.0, 0.0, 0.0]])

    q = (rotation @ (c.T)).T + t# + (np.random.rand(12000).reshape(4000, 3) * 0.1)
    # print( (np.random.rand(300).reshape(100, 3) * 0.01))
    target = o3d.geometry.PointCloud()
    target.points = o3d.utility.Vector3dVector(q)
    zero = o3d.geometry.PointCloud()
    zero.points = o3d.utility.Vector3dVector(z)
    source.paint_uniform_color([1, 0, 0])
    target.paint_uniform_color([0, 1, 0])
    zero.paint_uniform_color([0, 0, 0])
    o3d.visualization.draw_geometries([source, target, zero])
    # o3d.visualization.draw_geometries([zero])
    threshold = 1
    zero_init = np.eye(4)
    # print(zero_init)
    # draw_registration_result(source, target, trans_init)
    # print("Initial alignment")
    # Run icp
    d_max = True
    count = 0
    print("start")
    while d_max:
        # for n in range(100):
        # sig = o3d.pipelines.registration.TukeyLoss(k=0.1)
        # print("zero_init",zero_init)
        reg_p2p = o3d.pipelines.registration.registration_icp( target,source, threshold, zero_init,
                                                              o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                                                              o3d.pipelines.registration.ICPConvergenceCriteria(
                                                                  max_iteration=2000))

        # Transform our matrix according to the output transformation matrix
        print(reg_p2p)
        # print(np.asarray(reg_p2p.correspondence_set).shape)
        cor = np.asarray(reg_p2p.correspondence_set).shape

        print("p2p transformation", reg_p2p.transformation)
        target.transform(reg_p2p.transformation)
        # print(np.asarray(source.points[-1]))
        trans = reg_p2p.transformation
        marge_t = zero_init @ reg_p2p.transformation
        zero_init = reg_p2p.transformation
        # print(marge_t)
        print(cor[0])
        if cor[0] >= 1000:
            d_max = False
        else:
            count += 1
            if count > 100:
                print("fall")
                d_max = False
    print("count : ",count)
    o3d.visualization.draw_geometries([source, target, zero])
    # draw_registration_result(source, target, reg_p2p.transformation)
    #
    # def apply_noise(pcd, mu, sigma):
    #     noisy_pcd = copy.deepcopy(pcd)
    #     points = np.asarray(noisy_pcd.points)
    #     points += np.random.normal(mu, sigma, size=points.shape)
    #     noisy_pcd.points = o3d.utility.Vector3dVector(points)
    #     return noisy_pcd
    #
    # mu, sigma = 0, 0.1  # mean and standard deviation
    # source_noisy = apply_noise(source, mu, sigma)
    #
    # print("Source PointCloud + noise:")
    # o3d.visualization.draw_geometries([source_noisy],
    #                                   zoom=0.4459,
    #                                   front=[0.353, -0.469, -0.809],
    #                                   lookat=[2.343, 2.217, 1.809],
    #                                   up=[-0.097, -0.879, 0.467])
    #
    #
    #
    # print("Robust point-to-plane ICP, threshold={}:".format(threshold))
    # loss = o3d.pipelines.registration.TukeyLoss(k=sigma)
    # print("Using robust loss:", loss)
    # p2l = o3d.pipelines.registration.TransformationEstimationPointToPlane(loss)
    # reg_p2l = o3d.pipelines.registration.registration_icp(source_noisy, target,
    #                                                       threshold, trans,
    #                                                       p2l)
    # print(reg_p2l)
    # print("Transformation is:")
    # print(reg_p2l.transformation)
    # draw_registration_result(source, target, reg_p2l.transformation)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

#!/home/j/.pyenv/versions/ros_py36/bin/python3
import numpy as np
import open3d as o3d


def main():
    plane_ = np.random.rand(2000).reshape(1000, 2) * 100
    plane = np.concatenate((plane_, np.ones((1000, 1))), axis=1)

    r = [1.55, -2.04, 1.25]
    rotation = create_rotation_matrix(r)

    t = np.array([[19, 15, -16]])
    test_plane_points = (rotation @ (plane.T)).T + t + (np.random.rand(3000).reshape(1000, 3) * 0.5)

    test_plane = o3d.geometry.PointCloud()
    test_plane.points = o3d.utility.Vector3dVector(test_plane_points)

    normal_vector = make_plane_function(test_plane_points)
    print(normal_vector.shape)
    print(np.linalg.norm(normal_vector))

    normal_vector_t = normal_vector.T
    test_vectors = normal_vector_t
    for i in range(100):
        normal_vector_test = (i + 1) * normal_vector_t
        test_vectors = np.concatenate((test_vectors, normal_vector_test))

    z = np.asarray([[0.0, 0.0, 0.0]])
    z = np.concatenate((z, test_vectors))

    normal_vector_o3d = o3d.geometry.PointCloud()
    normal_vector_o3d.points = o3d.utility.Vector3dVector(z)

    test_plane.paint_uniform_color([0, 1, 0])
    normal_vector_o3d.paint_uniform_color([0, 0, 0])
    o3d.visualization.draw_geometries([test_plane, normal_vector_o3d], point_show_normal=True)


def make_plane_function(points):
    shape = points.shape
    one_vecter = np.ones((shape[0], 1))
    normal_vector_ = np.linalg.inv(points.T @ points) @ points.T @ one_vecter
    dist = 1 / (np.linalg.norm(normal_vector_))
    normal_vector = normal_vector_ * dist
    return normal_vector


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


if __name__ == '__main__':
    main()

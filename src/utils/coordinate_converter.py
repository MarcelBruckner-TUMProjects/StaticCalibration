import numpy as np
from scipy.spatial.transform import Rotation as R


def create_translation(x, y, z=0.):
    return np.array([
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1]
    ], dtype=float)


def create_rotation(axis, angle):
    raw = R.from_euler(axis, angle, degrees=True)
    raw = np.append(raw.as_matrix(), np.array([[0, 0, 0]]), axis=0)
    raw = np.append(raw, np.array([[0], [0], [0], [1]]), axis=1)
    return raw


def euler_from_matrix(mat):
    result = mat[:, :3]
    result = result[:3]
    return R.from_matrix(result).as_euler('zyx', degrees=True)


# S50
T_base = create_translation(0, 0)
# print(T_base)
R_base = create_rotation('z', 16.5)
# print(R_base)

H_base = R_base * T_base
# print(H_base)

T_radar = create_translation(0, 0, 7.23)
# print(T_radar)

T_mount = create_translation(-0.67, -22.076, 1.111)
# print(T_mount)

R_mount = create_rotation('z', 180)
# print(R_mount)

T_near = create_translation(0.1582, -0.865, -0.3074)
# print(T_near)

# See below why no matrix
R_near_x = 11.2
R_near_y = 0.5
R_near_z = create_rotation('z', 0.32)
# print(R_near_z)

T_road = [-7.67, -25.89, 0]
R_road = [-196.5, 0, 0, ]

origin = np.array([0, 0, 0, 1])

H_camera_to_mount = np.linalg.inv(T_near @ R_near_z)
# print('H_camera_to_mount', '\n', H_camera_to_mount)
H_mount_to_base = np.linalg.inv(T_mount @ T_radar @ R_mount)
# print('H_mount_to_base', '\n', H_mount_to_base)
H_base_to_global = np.linalg.inv(T_base @ R_base)
# print('H_base_to_global', '\n', H_base_to_global)
H_global_to_camera = H_base_to_global @ H_mount_to_base @ H_camera_to_mount

H_global_to_camera[2, 3] *= -1
# print('H_camera_to_global', '\n', H_global_to_camera)

camera_translation_global = H_global_to_camera @ origin
print('camera_translation_global', '\t', camera_translation_global)

camera_rotation_global = euler_from_matrix(H_global_to_camera)
camera_rotation_global = [camera_rotation_global[0], R_near_x, R_near_y]
print('camera_rotation_global', '\t\t', camera_rotation_global)

camera_translation_road = camera_translation_global - np.array([*T_road, 0])
camera_translation_road[1] *= -1
print('camera_translation_road', '\t', camera_translation_road)

camera_rotation_road = [(x[0] - x[1]) % 360 for x in zip(camera_rotation_global, np.array([*R_road, 0]))]
print('camera_rotation_road', '\t\t', camera_rotation_road)

# coding = utf-8
# 旋转
import numpy as np
import scipy.linalg as linalg


# 参数分别是旋转轴和旋转弧度值
def rotate_mat(axis, radian):
    return linalg.expm(np.cross(np.eye(3), axis / linalg.norm(axis) * radian))


# 旋转轴
def cal_axis(v1, v2, v3=[0, 0, 0]):
    v1 = np.asarray(v1)
    v2 = np.asarray(v2)
    v3 = np.asarray(v3)
    axis = np.cross(v1 - v3, v2 - v3)
    return axis / np.linalg.norm(axis)


def get_theta(v1, v2):
    v1 = np.asarray(v1)
    v2 = np.asarray(v2)
    return np.arccos(
        (v1 @ v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)))  # /np.pi*180


def R_by_vector(v1, v2):
    v1 = np.asarray(v1)
    v2 = np.asarray(v2)
    theta = get_theta(v1, v2)
    axis = cal_axis(v1, v2)
    print('theta:', theta * 180 / np.pi, '\naxis:', axis)
    R = rotate_mat(axis=axis, radian=theta)
    return theta, axis, R


def rotate(points, axis, theta, center=[0, 0, 0]):
    points = np.asarray(points)
    center = np.asarray(center)
    points = points - np.tile(center, (points.shape[0], 1))


# #点云旋转
# pcd_points_R = np.dot(pcd_points, R.T)

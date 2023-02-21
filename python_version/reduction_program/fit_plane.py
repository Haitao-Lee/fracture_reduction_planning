# coding = utf-8
import numpy as np
from scipy.optimize import leastsq
from sklearn import svm


def fit_func(p, x, y):
    """ 数据拟合函数 """
    a, b, c = p
    return a * x + b * y + c


def residuals(p, x, y, z):
    """ 误差函数 """
    return z - fit_func(p, x, y)


def estimate_plane_with_leastsq(pts):
    """ 根据最小二乘拟合出平面参数 """
    p0 = [1, 0, 1]
    np_pts = np.array(pts)
    plsq = leastsq(residuals,
                   p0,
                   args=(np_pts[:, 0], np_pts[:, 1], np_pts[:, 2]))
    return plsq[0]


def get_proper_plane_params(p, pts):
    """ 根据拟合的平面的参数，得到实际显示的最佳的平面 """
    np_pts = np.array(pts)

    np_pts_mean = np.mean(np_pts, axis=0)
    np_pts_min = np.min(np_pts, axis=0)
    np_pts_max = np.max(np_pts, axis=0)

    plane_center_z = fit_func(p, np_pts_mean[0], np_pts_mean[1])
    plane_center = np.array([np_pts_mean[0], np_pts_mean[1], plane_center_z])

    plane_origin_z = fit_func(p, np_pts_min[0], np_pts_min[1])
    plane_origin = np.array([np_pts_min[0], np_pts_min[1], plane_origin_z])

    if np.linalg.norm(p) < 1e-10:
        print(r'plsq 的 norm 值为 0 {}'.format(p))
    plane_normal = p / np.linalg.norm(p)

    plane_pt_1 = np.array([
        np_pts_max[0], np_pts_min[1],
        fit_func(p, np_pts_max[0], np_pts_min[1])
    ])
    plane_pt_2 = np.array([
        np_pts_min[0], np_pts_max[1],
        fit_func(p, np_pts_min[0], np_pts_max[1])
    ])
    return plane_center, plane_normal, plane_origin, plane_pt_1, plane_pt_2


def fit_plane_by_norm2(points, dir):
    p = estimate_plane_with_leastsq(points)
    center, normal, origin, plane_x_pt, plane_y_pt = get_proper_plane_params(
        p, points)
    print(
        'estimate plane center: {}, \nnormal: {}, \norigin: {}, \nplane_x_pt: {}, \nplane_y_pt: {}'
        .format(center, normal, origin, plane_x_pt, plane_y_pt))
    if np.dot(normal, dir) < 0:
        normal = -normal
    return center, normal, origin, plane_x_pt, plane_y_pt


def fit_plane_by_svm(points1, points2, dir_vec, threshold):
    cls = svm.SVC(kernel="linear", C=threshold)
    labels1 = np.ones(points1.shape[0])
    labels2 = -np.ones(points2.shape[0])
    X = np.concatenate([points1, points2], axis=0)
    Y = np.concatenate([labels1, labels2], axis=0)
    cls.fit(X, Y)
    w = cls.coef_[0]
    b = cls.intercept_[0]
    sv = cls.support_vectors_[-1]
    z = -(w[0] * sv[0] + w[1] * sv[1] + b) / w[2]
    plane_center = [sv[0], sv[1], z]
    w = w / np.linalg.norm(w)
    if np.dot(w, dir_vec) < 0:
        w = -w
    return w, plane_center

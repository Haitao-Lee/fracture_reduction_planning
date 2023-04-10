# coding = utf-8
import geometry
import numpy as np
import open3d as o3d
import scipy.spatial as spatial
import scipy
from tqdm import tqdm
from sklearn.cluster import DBSCAN
from sklearn.decomposition import PCA
import screw_setting
import visualization
import time


def get_screw_dir_by_SVM(points1,
                         points2,
                         svm_threshold=screw_setting.svm_threshold):
    plane_normal, _ = geometry.fit_plane_by_svm(points1, points2,
                                                svm_threshold)
    return plane_normal


def get_screw_dir_by_Center(points1, points2):
    center1 = np.mean(points1, axis=0)
    center2 = np.mean(points2, axis=0)
    normal = center1 - center2
    normal = normal / np.linalg.norm(normal)
    return normal


def get_screw_dir_by_norm2(points1, points2):
    points = np.concatenate([points1, points2], axis=0)
    plane_info = geometry.fit_plane_by_norm2(np.array(points))
    return plane_info[1]


def get_screw_dir_by_ransac(points1, points2):
    points = np.concatenate([points1, points2], axis=0)
    plane_info, _, _ = geometry.ransac_planefit(
        points, ransac_n=3, max_dst=screw_setting.ransac_eps)
    return plane_info[3:6]


# def get_screw_implant_position(points1, points2):
#     center1 = np.mean(points1, axis=0)
#     center2 = np.mean(points2, axis=0)
#     # center = (center1 + center2)/2
#     center = (points2.shape[0] * center1 + points1.shape[0] * center2) / (points1.shape[0] + points2.shape[0])
#     return center


# 近似求取切比雪夫中心
def get_screw_implant_position_by_Chebyshev_center(points1, points2):
    points1 = np.array(points1)
    points2 = np.array(points2)
    all_points = np.concatenate([points1, points2], axis=0)
    pca = PCA()
    pca.fit(all_points)
    vec1 = np.array(pca.components_[0, :])
    vec1 = vec1 / np.linalg.norm(vec1)
    dsp_dsbt = np.dot(all_points, vec1)
    min_val = dsp_dsbt[np.argmin(dsp_dsbt)]
    max_val = dsp_dsbt[np.argmax(dsp_dsbt)]
    res = max_val - min_val
    s_r = get_screw_radius()
    initial_center = np.mean(all_points, axis=0)
    tree = spatial.KDTree(all_points)
    # plane_info, _, _ = geometry.ransac_planefit(all_points, ransac_n=3, max_dst=screw_setting.ransac_eps)
    normal = get_screw_dir_by_SVM(points1, points2)
    normal1 = np.array([normal[2], 0, -normal[0]])
    if np.linalg.norm(normal1) == 0:
        normal1 = np.array([normal[1],  -normal[0], 0])
    normal1 = normal1 / np.linalg.norm(normal1)
    normal2 = np.cross(normal1, normal)
    normal2 = normal2 / np.linalg.norm(normal2)
    normal3 = normal1 - normal2
    normal3 = normal3/np.linalg.norm(normal3)
    normal4 = normal1 + normal2
    normal4 = normal4 / np.linalg.norm(normal4)
    normals = [normal1, normal2, normal3, normal4]
    # initial_res = 0
    # init_length = 0
    # while initial_res == 0:
    #     tmp_center = initial_center + init_length * normals[0]
    #     tmp_norm = np.linalg.norm(all_points - np.expand_dims(tmp_center, 0).repeat(all_points.shape[0], axis=0), axis=1)
    #     indices = np.array(np.where(tmp_norm < 3)).flatten()
    #     if indices.shape[0] == 0:
    #         initial_res = init_length
    #         break
    #     tmp_center = initial_center - init_length * normals[0]
    #     tmp_norm = np.linalg.norm(all_points - np.expand_dims(tmp_center, 0).repeat(all_points.shape[0], axis=0), axis=1)
    #     indices = np.array(np.where(tmp_norm < 3)).flatten()
    #     if indices.shape[0] == 0:
    #         initial_res = init_length
    #         break
    #     tmp_center = initial_center + init_length * normals[1]
    #     tmp_norm = np.linalg.norm(all_points - np.expand_dims(tmp_center, 0).repeat(all_points.shape[0], axis=0), axis=1)
    #     indices = np.array(np.where(tmp_norm < 3)).flatten()
    #     if indices.shape[0] == 0:
    #         initial_res = init_length
    #         break
    #     tmp_center = initial_center - init_length * normals[1]
    #     tmp_norm = np.linalg.norm(all_points - np.expand_dims(tmp_center, 0).repeat(all_points.shape[0], axis=0), axis=1)
    #     indices = np.array(np.where(tmp_norm < 3)).flatten()
    #     if indices.shape[0] == 0:
    #         initial_res = init_length
    #         break
    #     tmp_center = initial_center + init_length * normals[2]
    #     tmp_norm = np.linalg.norm(all_points - np.expand_dims(tmp_center, 0).repeat(all_points.shape[0], axis=0), axis=1)
    #     indices = np.array(np.where(tmp_norm < 3)).flatten()
    #     if indices.shape[0] == 0:
    #         initial_res = init_length
    #         break
    #     tmp_center = initial_center - init_length * normals[2]
    #     tmp_norm = np.linalg.norm(all_points - np.expand_dims(tmp_center, 0).repeat(all_points.shape[0], axis=0), axis=1)
    #     indices = np.array(np.where(tmp_norm < 3)).flatten()
    #     if indices.shape[0] == 0:
    #         initial_res = init_length
    #         break
    #     tmp_center = initial_center + init_length * normals[3]
    #     tmp_norm = np.linalg.norm(all_points - np.expand_dims(tmp_center, 0).repeat(all_points.shape[0], axis=0), axis=1)
    #     indices = np.array(np.where(tmp_norm < 3)).flatten()
    #     if indices.shape[0] == 0:
    #         initial_res = init_length
    #         break
    #     tmp_center = initial_center - init_length * normals[3]
    #     tmp_norm = np.linalg.norm(all_points - np.expand_dims(tmp_center, 0).repeat(all_points.shape[0], axis=0), axis=1)
    #     indices = np.array(np.where(tmp_norm < 3)).flatten()
    #     if indices.shape[0] == 0:
    #         initial_res = init_length
    #         break
    #     init_length = init_length + 0.04
    if res < 12 * s_r:
        # _, all_tmp_points, _ = geometry.ransac_planefit(all_points, ransac_n=3, max_dst=screw_setting.ransac_eps/2)
        # initial_center = np.mean(all_tmp_points, axis=0)
        return initial_center
    else:
        tmp_position = min_val + 6 * s_r
        max_res = 0
        best_center = initial_center
        last_num = 0
        while tmp_position <= max_val - max(6 * s_r, 0.2 * res):
            indices = np.array(
                np.where((dsp_dsbt > tmp_position - 6 * s_r)
                         & (dsp_dsbt < tmp_position + 6 * s_r))).flatten()
            if indices.shape[0] <= 60:
                tmp_position = tmp_position + s_r
                continue
            tmp_points = all_points[indices]
            # plane_info, _, _ = geometry.ransac_planefit(tmp_points, ransac_n=3, max_dst=screw_setting.ransac_eps)
            # normal = plane_info[3:6]
            center = np.mean(tmp_points, axis=0)
            init_length = 2
            tmp_res = 0
            while tmp_res == 0:
                tmp_center = center + init_length * normals[0]
                tmp_norm = np.linalg.norm(all_points - np.expand_dims(tmp_center, 0).repeat(all_points.shape[0], axis=0), axis=1)
                indices = np.array(np.where(tmp_norm < 3)).flatten()
                if indices.shape[0] == 0:
                    tmp_res = init_length
                    break
                tmp_center = center - init_length * normals[0]
                tmp_norm = np.linalg.norm(all_points - np.expand_dims(tmp_center, 0).repeat(all_points.shape[0], axis=0), axis=1)
                indices = np.array(np.where(tmp_norm < 3)).flatten()
                if indices.shape[0] == 0:
                    tmp_res = init_length
                    break
                tmp_center = center + init_length * normals[1]
                tmp_norm = np.linalg.norm(all_points - np.expand_dims(tmp_center, 0).repeat(all_points.shape[0], axis=0), axis=1)
                indices = np.array(np.where(tmp_norm < 3)).flatten()
                if indices.shape[0] == 0:
                    tmp_res = init_length
                    break
                tmp_center = center - init_length * normals[1]
                tmp_norm = np.linalg.norm(all_points - np.expand_dims(tmp_center, 0).repeat(all_points.shape[0], axis=0), axis=1)
                indices = np.array(np.where(tmp_norm < 3)).flatten()
                if indices.shape[0] == 0:
                    tmp_res = init_length
                    break
                tmp_center = center + init_length * normals[2]
                tmp_norm = np.linalg.norm(all_points - np.expand_dims(tmp_center, 0).repeat(all_points.shape[0], axis=0), axis=1)
                indices = np.array(np.where(tmp_norm < 3)).flatten()
                if indices.shape[0] == 0:
                    tmp_res = init_length
                    break
                tmp_center = center - init_length * normals[2]
                tmp_norm = np.linalg.norm(all_points - np.expand_dims(tmp_center, 0).repeat(all_points.shape[0], axis=0), axis=1)
                indices = np.array(np.where(tmp_norm < 3)).flatten()
                if indices.shape[0] == 0:
                    tmp_res = init_length
                    break
                tmp_center = center + init_length * normals[3]
                tmp_norm = np.linalg.norm(all_points - np.expand_dims(tmp_center, 0).repeat(all_points.shape[0], axis=0), axis=1)
                indices = np.array(np.where(tmp_norm < 3)).flatten()
                if indices.shape[0] == 0:
                    tmp_res = init_length
                    break
                tmp_center = center - init_length * normals[3]
                tmp_norm = np.linalg.norm(all_points - np.expand_dims(tmp_center, 0).repeat(all_points.shape[0], axis=0), axis=1)
                indices = np.array(np.where(tmp_norm < 3)).flatten()
                if indices.shape[0] == 0:
                    tmp_res = init_length
                    break
                init_length = init_length + 0.4
            balls = []
            for i in range(4):
                balls.append(center + tmp_res * normals[i])
                balls.append(center - tmp_res * normals[i])
            # tmp_pca = PCA()
            # tmp_pca.fit(tmp_points)
            # tmp_vec1 = np.array(tmp_pca.components_[0, :])
            # tmp_vec1 = tmp_vec1/np.linalg.norm(tmp_vec1)
            # tmp_vec2 = np.array(tmp_pca.components_[1, :])
            # tmp_vec2 = tmp_vec2/np.linalg.norm(tmp_vec2)
            # tmp_dsp_dsbt1 = np.dot(tmp_points, tmp_vec1)
            # tmp_min_val1 = tmp_dsp_dsbt1[np.argmin(tmp_dsp_dsbt1)]
            # tmp_max_val1 = tmp_dsp_dsbt1[np.argmax(tmp_dsp_dsbt1)]
            # tmp_res1 = tmp_max_val1 - tmp_min_val1
            # tmp_dsp_dsbt2 = np.dot(tmp_points, tmp_vec2)
            # tmp_min_val2 = tmp_dsp_dsbt2[np.argmin(tmp_dsp_dsbt2)]
            # tmp_max_val2 = tmp_dsp_dsbt2[np.argmax(tmp_dsp_dsbt2)]
            # tmp_res2 = tmp_max_val2 - tmp_min_val2
            # tmp_res = 0
            # if min(tmp_res1, tmp_res2) < 12*s_r:
            #     tmp_res = min(tmp_res1, tmp_res2)
            # elif min(tmp_res1, tmp_res2) >= 12*s_r:
            #     tmp_res = max(tmp_res1, tmp_res2)
            tmp_position = tmp_position + s_r
            if tmp_res > max_res or (tmp_res == max_res and indices.shape[0] > last_num):
                max_res = tmp_res
                last_num = indices.shape[0]
                # _, tmp_points, _ = geometry.ransac_planefit(tmp_points, ransac_n=3, max_dst=screw_setting.ransac_eps/5)
                best_center = np.mean(tmp_points, axis=0)
                # min_point = tmp_points[np.argmin(tmp_dsp_dsbt)]
                # max_point = tmp_points[np.argmax(tmp_dsp_dsbt)]
                # best_center = (min_point + max_point)/2
            tmp_indices = tree.query_ball_point(tmp_points, 0.5, workers=-1)
            new_indices = np.empty((0, 1))
            for k in range(len(tmp_indices)):
                new_indices = np.concatenate([new_indices, np.array(tmp_indices[k]).reshape(-1, 1)], axis=0)
            new_indices = new_indices.astype(np.int).flatten()
            tmp_all_points = all_points.copy()
            tmp_all_points = np.delete(tmp_all_points, new_indices, axis=0)
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(tmp_all_points)
            tmp_pcd = o3d.geometry.PointCloud()
            tmp_pcd.points = o3d.utility.Vector3dVector(tmp_points)
            visualization.points_visualization_by_vtk([tmp_pcd, pcd], balls, radius=3) #[np.mean(tmp_points, axis=0), best_center])
            # visualization.viz_matplot(tmp_points)
            
        # print('initial center:%.2fmm,  Chebyshev center:%.2fmm'% (initial_res, max_res))
        return best_center
 

def get_2_screw_implant_positions_by_Chebyshev_center(points1, points2, length_rate):
    points1 = np.array(points1)
    points2 = np.array(points2)
    all_points = np.concatenate([points1, points2], axis=0)
    pca = PCA()
    pca.fit(all_points)
    vec1 = np.array(pca.components_[0, :])
    vec1 = vec1 / np.linalg.norm(vec1)
    dsp_dsbt = np.dot(all_points, vec1)
    min_val = dsp_dsbt[np.argmin(dsp_dsbt)]
    max_val = dsp_dsbt[np.argmax(dsp_dsbt)]
    s_r = get_screw_radius()
    tmp_position = min_val + 8 * s_r
    max_res1 = 0
    max_res2 = 0
    best_center1 = np.mean(points1, axis=0)
    best_center2 = np.mean(points2, axis=0)
    max_size1 = 0
    max_size2 = 0
    # tree = spatial.KDTree(all_points)
    # normal = geometry.ransac_planefit(all_points, 3, max_dst=2*screw_setting.ransac_eps)[0][3:6]
    while tmp_position <= max_val - max(8 * s_r, 0.2 * (max_val - min_val)):
        indices = np.array(
            np.where((dsp_dsbt > tmp_position - 8 * s_r)
                     & (dsp_dsbt < tmp_position + 8 * s_r))).flatten()
        if indices.shape[0] <= 60:
            tmp_position = tmp_position + s_r
            continue
        tmp_points = all_points[indices]
        tmp_size = indices.shape[0]
        normal = geometry.ransac_planefit(tmp_points,
                                          3,
                                          max_dst=2 *
                                          screw_setting.ransac_eps)[0][3:6]
        # pcd = o3d.geometry.PointCloud()
        # pcd.points = o3d.utility.Vector3dVector(all_points)
        # tmp_pcd = o3d.geometry.PointCloud()
        # tmp_pcd.points = o3d.utility.Vector3dVector(tmp_points)
        # visualization.points_visualization_by_vtk([tmp_pcd, pcd], [np.mean(tmp_points, axis=0), best_center1, best_center2])
        normal1 = np.array([normal[2], 0, -normal[0]])
        if np.linalg.norm(normal1) == 0:
            normal1 = np.array([normal[1],  -normal[0], 0])
        normal1 = normal1 / np.linalg.norm(normal1)
        normal2 = np.cross(normal1, normal)
        normal2 = normal2 / np.linalg.norm(normal2)
        normal3 = normal1 - normal2
        normal3 = normal3/np.linalg.norm(normal3)
        normal4 = normal1 + normal2
        normal4 = normal4 / np.linalg.norm(normal4)
        normals = [normal1, normal2, normal3, normal4]
        center = np.mean(tmp_points, axis=0)
        init_length = 1.5
        tmp_res = 0
        while tmp_res == 0:
            while tmp_res == 0:
                tmp_center = center + init_length * normals[0]
                tmp_norm = np.linalg.norm(all_points - np.expand_dims(tmp_center, 0).repeat(all_points.shape[0], axis=0), axis=1)
                indices = np.array(np.where(tmp_norm < 3)).flatten()
                if indices.shape[0] == 0:
                    tmp_res = init_length
                    break
                tmp_center = center - init_length * normals[0]
                tmp_norm = np.linalg.norm(all_points - np.expand_dims(tmp_center, 0).repeat(all_points.shape[0], axis=0), axis=1)
                indices = np.array(np.where(tmp_norm < 3)).flatten()
                if indices.shape[0] == 0:
                    tmp_res = init_length
                    break
                tmp_center = center + init_length * normals[1]
                tmp_norm = np.linalg.norm(all_points - np.expand_dims(tmp_center, 0).repeat(all_points.shape[0], axis=0), axis=1)
                indices = np.array(np.where(tmp_norm < 3)).flatten()
                if indices.shape[0] == 0:
                    tmp_res = init_length
                    break
                tmp_center = center - init_length * normals[1]
                tmp_norm = np.linalg.norm(all_points - np.expand_dims(tmp_center, 0).repeat(all_points.shape[0], axis=0), axis=1)
                indices = np.array(np.where(tmp_norm < 3)).flatten()
                if indices.shape[0] == 0:
                    tmp_res = init_length
                    break
                tmp_center = center + init_length * normals[2]
                tmp_norm = np.linalg.norm(all_points - np.expand_dims(tmp_center, 0).repeat(all_points.shape[0], axis=0), axis=1)
                indices = np.array(np.where(tmp_norm < 3)).flatten()
                if indices.shape[0] == 0:
                    tmp_res = init_length
                    break
                tmp_center = center - init_length * normals[2]
                tmp_norm = np.linalg.norm(all_points - np.expand_dims(tmp_center, 0).repeat(all_points.shape[0], axis=0), axis=1)
                indices = np.array(np.where(tmp_norm < 3)).flatten()
                if indices.shape[0] == 0:
                    tmp_res = init_length
                    break
                tmp_center = center + init_length * normals[3]
                tmp_norm = np.linalg.norm(all_points - np.expand_dims(tmp_center, 0).repeat(all_points.shape[0], axis=0), axis=1)
                indices = np.array(np.where(tmp_norm < 3)).flatten()
                if indices.shape[0] == 0:
                    tmp_res = init_length
                    break
                tmp_center = center - init_length * normals[3]
                tmp_norm = np.linalg.norm(all_points - np.expand_dims(tmp_center, 0).repeat(all_points.shape[0], axis=0), axis=1)
                indices = np.array(np.where(tmp_norm < 3)).flatten()
                if indices.shape[0] == 0:
                    tmp_res = init_length
                    break
                init_length = init_length + 0.4
        tmp_position = tmp_position + s_r
        if tmp_res >= max_res1 and tmp_size > max_size1:
            if np.linalg.norm(center - best_center1) > length_rate * s_r:
                max_res2 = max_res1
                best_center2 = best_center1
                max_res1 = tmp_res
                best_center1 = center
                max_size2 = max_size1
                max_size1 = tmp_size
            elif np.linalg.norm(center - best_center2
                                ) > length_rate * s_r and tmp_size > max_size2:
                max_res1 = tmp_res
                best_center1 = center
                max_size2 = tmp_size
            tmp_position = tmp_position + length_rate * s_r
        elif tmp_res >= max_res2 and np.linalg.norm(
                center -
                best_center1) > length_rate * s_r and tmp_size > max_size2:
            max_res2 = tmp_res
            best_center2 = center
            max_size2 = tmp_size
    return best_center1, best_center2


# def get_screw_implant_position(points1, points2):
#     points1 = np.array(points1)
#     points2 = np.array(points2)
#     all_points = np.concatenate([points1, points2], axis=0)
#     pca = PCA()
#     pca.fit(all_points)
#     vec1 = np.array(pca.components_[0, :])
#     vec1 = vec1/np.linalg.norm(vec1)
#     vec2 = np.array(pca.components_[1, :])
#     vec2 = vec2/np.linalg.norm(vec2)
#     dsp_dsbt = np.dot(all_points, vec1)
#     min_val = dsp_dsbt[np.argmin(dsp_dsbt)]
#     max_val = dsp_dsbt[np.argmax(dsp_dsbt)]
#     res = max_val - min_val
#     s_r = get_screw_radius()
#     if res < 3*s_r:
#         return np.mean(all_points, axis=0)
#     else:
#         tmp_position = min_val + s_r
#         max_res = 0
#         best_center = None
#         while tmp_position <= max_val - s_r:
#             indices = np.array(np.where((dsp_dsbt > tmp_position) & (dsp_dsbt < tmp_position + s_r))).flatten()
#             if indices.shape[0] == 0:
#                 tmp_position = tmp_position + s_r
#                 continue
#             tmp_points = all_points[indices]
#             tmp_dsp_dsbt = np.dot(tmp_points, vec2)
#             tmp_min_val = tmp_dsp_dsbt[np.argmin(tmp_dsp_dsbt)]
#             tmp_max_val = tmp_dsp_dsbt[np.argmax(tmp_dsp_dsbt)]
#             min_point = tmp_points[np.argmin(tmp_dsp_dsbt)]
#             max_point = tmp_points[np.argmax(tmp_dsp_dsbt)]
#             tmp_res = tmp_max_val - tmp_min_val
#             tmp_position = tmp_position + s_r
#             if tmp_res > max_res:
#                 max_res = tmp_res
#                 best_center = np.mean(tmp_points, axis=0)
#                 # best_center = (min_point + max_point)/2
#         return best_center


def get_screw_radius(radius=screw_setting.screw_radius):
    return radius


def separate_point(points,
                   radius=screw_setting.sp_radius,
                   eps1=screw_setting.sp_threshold):
    points = np.array(points)
    all_points = []
    y_pred = DBSCAN(eps=eps1).fit_predict(points)
    y_uniq = np.unique(np.array(y_pred))
    for y in y_uniq:
        indices = np.argwhere(y_pred == y).flatten()
        ps = np.array(points[indices])
        if ps.shape[0] / points.shape[0] < 0.1 and ps.shape[0] < 10:
            continue
        # pca = PCA()
        # pca.fit(points)
        # vec = np.array(pca.components_[0, :])
        # vec = vec/np.linalg.norm(vec)
        # dsp_dsbt = np.dot(ps, vec)
        # min_val = dsp_dsbt[np.argmin(dsp_dsbt)]
        # max_val = dsp_dsbt[np.argmax(dsp_dsbt)]
        # res = max_val - min_val

        # vec_1 = np.array(pca.components_[1, :])
        # vec_1 = vec_1/np.linalg.norm(vec_1)
        # dsp_dsbt_1 = np.dot(ps, vec_1)
        # min_val_1 = dsp_dsbt_1[np.argmin(dsp_dsbt_1)]
        # max_val_1 = dsp_dsbt_1[np.argmax(dsp_dsbt_1)]
        # res_1 = max_val_1 - min_val_1

        # # vec_2 = np.array(pca.components_[2, :])
        # # vec_2 = vec_2/np.linalg.norm(vec_2)
        # # dsp_dsbt_2 = np.dot(ps, vec_2)
        # # min_val_2 = dsp_dsbt_2[np.argmin(dsp_dsbt_2)]
        # # max_val_2 = dsp_dsbt_2[np.argmax(dsp_dsbt_2)]
        # # res_2 = max_val_2 - min_val_2
        # if res_1 < 4*get_screw_radius():
        #     continue
        # if res > 2 * radius:
        #     indices1 = np.array(np.where(dsp_dsbt < (min_val + max_val) / 2)).flatten()
        #     indices2 = np.array(np.where(dsp_dsbt > (min_val + max_val) / 2)).flatten()
        #     ps1 = ps[indices1]
        #     ps2 = ps[indices2]

        #     pca1 = PCA()
        #     pca1.fit(ps1)
        #     vec1 = np.array(pca1.components_[1, :])
        #     vec1 = vec1/np.linalg.norm(vec1)
        #     dsp_dsbt1 = np.dot(ps1, vec1)
        #     min_val1 = dsp_dsbt1[np.argmin(dsp_dsbt1)]
        #     max_val1 = dsp_dsbt1[np.argmax(dsp_dsbt1)]
        #     res1 = max_val1 - min_val1

        #     pca2 = PCA()
        #     pca2.fit(ps2)
        #     vec2 = np.array(pca2.components_[1, :])
        #     vec2 = vec2/np.linalg.norm(vec2)
        #     dsp_dsbt2 = np.dot(ps2, vec2)
        #     min_val2 = dsp_dsbt2[np.argmin(dsp_dsbt2)]
        #     max_val2 = dsp_dsbt2[np.argmax(dsp_dsbt2)]
        #     res2 = max_val2 - min_val2
        #     if res1 > 4*get_screw_radius():
        #         all_points.append(ps1)
        #     if res2 > 4*get_screw_radius():
        #         all_points.append(ps2)
        #     continue
        all_points.append(ps)
    return all_points


def get_effect_points(pcds, threshold=screw_setting.gep_threshold):
    all_points = [0]
    tmp_all_points = []
    for pcd in pcds:
        points = np.array(pcd.points)
        tmp_all_points.append(points)
    all_points[0] = tmp_all_points[0].copy()
    for i in range(1, len(tmp_all_points)):
        for j in range(len(all_points)):
            if tmp_all_points[i].shape[0] <= all_points[j].shape[0]:
                all_points.insert(j, tmp_all_points[i])
                break
            elif j == len(all_points) - 1:
                all_points.append(tmp_all_points[i])
                break
    trees = []
    for points in all_points:
        tree = spatial.KDTree(points)
        trees.append(tree)
    finish_indices = []
    match_clusters = []
    # id1 = None
    # id2 = None
    for i in range(0, len(all_points)):
        finish_indices.append(i)
        points1 = np.empty((0, 3))
        points2 = np.empty((0, 3))
        for j in range(0, len(trees)):
            if j not in finish_indices:
                _, indices = trees[j].query(all_points[i], 1, workers=-1)
                indices = np.array(indices)
                dist = np.linalg.norm(all_points[i] - all_points[j][indices],
                                      axis=1)
                dist_idx = np.argwhere(dist < threshold).flatten()
                points1 = np.concatenate([points1, all_points[i][dist_idx]],
                                         axis=0)
                points2 = np.concatenate(
                    [points2, all_points[j][indices[dist_idx]]], axis=0)
        if points1.shape[0] > 50:
            match_clusters.append([points1, points2])
    refine_cluster = []
    # matched_pcds = []
    for match_cluster in match_clusters:
        points1 = match_cluster[0]
        points2 = match_cluster[1]
        all_points = separate_point(points1)
        tree = spatial.KDTree(points2)
        # tmp_cluster = []
        for points in all_points:
            _, indices = tree.query(points, 1, workers=-1)
            refine_cluster.append([points, points2[indices]])
            # pcd_ps = np.concatenate([points, points2[indices]], axis=0)
            # pcd = o3d.geometry.PointCloud()
            # pcd.points = o3d.utility.Vector3dVector(pcd_ps)
            # matched_pcds.append(pcd)
        # refine_cluster.append(tmp_cluster)
    # visualization.points_visualization_by_vtk(matched_pcds)
    return refine_cluster


def get_cone(dire,
             angle=screw_setting.cone_angle,
             r_resolution=screw_setting.r_res,
             c_resolution=screw_setting.c_res):
    orth_dir = np.array([dire[2], 0, -dire[0]])
    orth_dir = orth_dir / np.linalg.norm(orth_dir)
    radius = np.tan(angle)
    rot_mtx = scipy.linalg.expm(
        np.cross(np.eye(3),
                 dire / scipy.linalg.norm(dire) * 2 * np.pi / c_resolution))
    cone = [dire]
    for i in range(c_resolution):
        orth_dir = np.dot(rot_mtx, orth_dir)
        for j in range(r_resolution):
            n_dir = dire + orth_dir * radius * j / r_resolution
            n_dir = n_dir / np.linalg.norm(n_dir)
            cone.append(n_dir)
    return cone


def add_screw_length(path_info,
                     pcds,
                     eps=screw_setting.screw_radius,
                     dist_eps=screw_setting.dist_eps):
    rf_path_info = []
    restPoints = np.empty((0, 3))
    for pcd in pcds:
        points = np.asarray(pcd.points)
        restPoints = np.concatenate([restPoints, points], axis=0)
    # tree = spatial.KDTree(allPoints)
    for info in path_info:
        n_dir = info[0]
        cent = info[1]
        id1 = info[2]
        id2 = info[3]
        cent_var = restPoints - np.expand_dims(cent, 0).repeat(restPoints.shape[0], axis=0)
        norm = np.linalg.norm(cent_var, axis=1)
        r_dist = np.sqrt(norm**2 - np.abs(np.dot(cent_var, n_dir.T))**2)
        indices = np.argwhere(r_dist < eps).flatten()
        if indices.shape[0] == 0:
            continue
        tmp_points = restPoints[indices]
        y_pred = DBSCAN(eps=dist_eps / 2).fit_predict(tmp_points)
        y_uniq = np.unique(np.array(y_pred))
        # center_list = []
        fp_list = []
        cp_list = []
        ps = None
        cone_pcd = []
        explore_points = []
        for y in y_uniq:
            idx = np.argwhere(y_pred == y).flatten()
            ps = np.array(tmp_points[idx])
            # if ps.shape[0] < 200:
            #     continue
            dist = np.dot(ps, n_dir.T)
            # center_list.append(np.mean(ps, axis=0))
            cp_list.append(ps[np.argmin(dist).flatten()[0]])
            fp_list.append(ps[np.argmax(dist).flatten()[0]])
            tmp_pcd = o3d.geometry.PointCloud()
            tmp_pcd.points = o3d.utility.Vector3dVector(ps)
            cone_pcd.append(tmp_pcd)
        # center_list = np.array(center_list)
        fp_list = np.array(fp_list)
        cp_list = np.array(cp_list)
        ori_cent = np.expand_dims(cent, 0).repeat(fp_list.shape[0], axis=0)
        # dif_cent = center_list - ori_cent
        dir_fp = fp_list - ori_cent
        dir_cp = cp_list - ori_cent
        # com_cent = np.linalg.norm(dif_cent, axis=1)
        com_fp = np.linalg.norm(dir_fp, axis=1)
        com_cp = np.linalg.norm(dir_cp, axis=1)
        # index = np.argmin(com_cent).flatten()[0]
        length1 = 0
        length2 = 0
        tmp_length1 = 0
        tmp_length2 = 0
        idx1 = -1
        idx2 = -1
        if com_fp.shape[0] <= 1:
            continue
        for k in range(com_fp.shape[0]):
            if com_fp[k] < com_cp[k]:
                tmp_com = com_cp[k].copy()
                com_cp[k] = com_fp[k]
                com_fp[k] = tmp_com
                tmp_diff = dir_cp[k].copy()
                dir_cp[k] = dir_fp[k]
                dir_fp[k] = tmp_diff
        tmp_com_cp = np.array([com_cp[0]])
        tmp_com_fp = np.array([com_fp[0]])
        tmp_dir_cp = np.array([dir_cp[0]])
        for m in range(1, com_cp.shape[0]):
            for k in range(tmp_com_cp.shape[0]):
                if com_cp[m] <= tmp_com_cp[k]:
                    tmp_com_cp = np.insert(tmp_com_cp,
                                        k,
                                        com_cp[m],
                                        axis=0)
                    tmp_com_fp = np.insert(tmp_com_fp,
                                        k,
                                        com_fp[m],
                                        axis=0)
                    tmp_dir_cp = np.insert(tmp_dir_cp,
                                        k,
                                        dir_cp[m],
                                        axis=0)
                    break
                elif k == tmp_com_cp.shape[0] - 1:
                    tmp_com_cp = np.concatenate([tmp_com_cp, [com_cp[m]]],
                                                axis=0)
                    tmp_com_fp = np.concatenate([tmp_com_fp, [com_fp[m]]],
                                                axis=0)
                    tmp_dir_cp = np.concatenate([tmp_dir_cp, [dir_cp[m]]],
                                                axis=0)
                    break
        com_cp = tmp_com_cp
        com_fp = tmp_com_fp
        dir_cp = tmp_dir_cp
        explore_flag1 = False
        explore_flag2 = False
        for k in range(com_fp.shape[0]):
            pn = np.dot(dir_cp[k], n_dir.T)
            if pn > 0 and not explore_flag1:
                tmp_length1 = com_cp[k]
                # visualization.points_visualization_by_vtk(rest_pcds_for_explore, centers=[cent+(tmp_length1 + com_fp[idx1])/2*n_dir])
                if idx1 == -1:
                    explore_length1 = tmp_length1 / 2
                    if not isExploreV1(pcds, [n_dir, cent, id1, id2, explore_length1, 0]):
                        length1 = tmp_length1
                        idx1 = k
                    else:
                        explore_points.append(cent + n_dir * explore_length1)
                        explore_flag1 = True
                else:
                    explore_length1 = (tmp_length1 + com_fp[idx1]) / 2
                    explore_points.append(cent + n_dir * explore_length1)
                    explore_points.append(cent + n_dir * (com_cp[idx1] + com_fp[idx1]) / 2)
                    if not isExploreV1(pcds, [n_dir, cent, id1, id2, explore_length1, 0]) and not isExploreV1(pcds, [n_dir, cent, id1, id2, (com_cp[idx1] + com_fp[idx1]) / 2, 0]):
                        length1 = tmp_length1
                        idx1 = k
                    else:
                        explore_flag1 = True
            elif pn <= 0 and not explore_flag2:
                tmp_length2 = com_cp[k]
                # visualization.points_visualization_by_vtk(rest_pcds_for_explore, centers=[cent+(tmp_length1 + com_fp[idx1])/2*n_dir])
                if idx2 == -1:
                    explore_length2 = tmp_length2 / 2
                    explore_points.append(cent - n_dir * explore_length2)
                    if not isExploreV2(pcds, [n_dir, cent, id1, id2, 0, explore_length2]):
                        length2 = tmp_length2
                        idx2 = k
                    else:
                        explore_flag2 = True
                else:
                    explore_length2 = (tmp_length2 + com_fp[idx2]) / 2
                    explore_points.append(cent - n_dir * explore_length2)
                    explore_points.append(cent - n_dir * (com_cp[idx2] + com_fp[idx2]) / 2)
                    if not isExploreV2(pcds, [n_dir, cent, id1, id2, 0, explore_length2]) and not isExploreV2(pcds, [n_dir, cent, id1, id2, 0, (com_cp[idx2] + com_fp[idx2]) / 2]):
                        length2 = tmp_length2
                        idx2 = k
                    else:
                        explore_flag2 = True
        rf_path_info.append([n_dir, cent, id1, id2, length1, length2])
    # visualization.points_visualization_by_vtk(matched_pcds)
    return rf_path_info


def estimate_dist(info1, info2):
    dire1 = info1[0]
    cent1 = info1[1]
    f_length1 = info1[4]
    b_length1 = info1[5]
    f_p1 = cent1 + dire1 * f_length1
    b_p1 = cent1 - dire1 * b_length1

    dire2 = info2[0]
    cent2 = info2[1]
    f_length2 = info2[4]
    b_length2 = info2[5]
    f_p2 = cent2 + dire2 * f_length2
    b_p2 = cent2 - dire2 * b_length2
    return geometry.segment_3d_dist(f_p1, b_p1, f_p2, b_p2)


def isInterference(new_info, path_infos, eps=2.5 * screw_setting.screw_radius):
    for info in path_infos:
        dist = estimate_dist(new_info, info)
        if dist <= eps:
            # if np.abs(np.dot(info[0], new_info[0])) <= 0.1:
            #     visualization.screws_vis([new_info, info])
            return True, info
    return False, None


# def isExplore(pcds, info, radius=screw_setting.screw_radius):
#     length1 = info[4]
#     length2 = info[5]
#     length = length1 + length2
#     if length < 20*radius:
#         return False
#     all_points = np.empty((0, 3))
#     all_normals = np.empty((0, 3))
#     for pcd in pcds:
#         all_points = np.concatenate([all_points, np.array(pcd.points)], axis=0)
#         all_normals = np.concatenate([all_normals, np.array(pcd.normals)], axis=0)
#     dire = np.array(info[0])
#     cent = np.array(info[1])

#     test_point1 = cent + (length1 - 4*radius)*dire
#     test_dif1 = np.linalg.norm(all_points - np.expand_dims(test_point1, 0).repeat(all_points.shape[0], axis=0), axis=1)
#     index1 = np.argmin(test_dif1).flatten()[0]
#     c_p1 = all_points[index1].reshape(-1, 3)
#     c_n1 = all_normals[index1].reshape(-1, 3)
#     vec1 = c_p1 - test_point1
#     if np.dot(c_n1, vec1.T).min() < 0:
#         dist1 = np.linalg.norm(vec1, axis=1)
#         return (dist1.max() > radius)

#     test_point2 = cent - length2*dire/2
#     test_dif2 = np.linalg.norm(all_points - np.expand_dims(test_point2, 0).repeat(all_points.shape[0], axis=0), axis=1)
#     index2 = np.argmin(test_dif2).flatten()[0]
#     c_p2 = all_points[index2].reshape(-1, 3)
#     c_n2 = all_normals[index2].reshape(-1, 3)
#     vec2 = c_p2 - test_point2
#     if np.dot(c_n2, vec2.T).min() < 0:
#         dist2 = np.linalg.norm(vec2, axis=1)
#         return (dist2.max() > radius)
#     return False


def isExplore(pcds,
              info,
              res=screw_setting.resolution,
              radius=10 * screw_setting.screw_radius):
    length1 = info[4]
    length2 = info[5]
    all_points = np.empty((0, 3))
    all_normals = np.empty((0, 3))
    for pcd in pcds:
        all_points = np.concatenate([all_points, np.array(pcd.points)], axis=0)
        all_normals = np.concatenate(
            [all_normals, np.array(pcd.normals)], axis=0)
    # tree = spatial.KDTree(all_points)
    dire = np.array(info[0])
    cent = np.array(info[1])
    dire1 = np.array([dire[2], 0, -dire[0]])
    dire1 = dire1 / np.linalg.norm(dire1)
    dire2 = np.cross(dire, dire1)
    dire3 = dire1 - dire2
    dire3 = dire3 / np.linalg.norm(dire3)
    dire4 = dire1 + dire2
    dire4 = dire4 / np.linalg.norm(dire4)
    dires = [dire1, dire2, dire3, dire4]
    for j in range(1, res):
        test_point1 = cent + j * (length1) * dire / res
        for i in range(0, 4):
            tmp_center = test_point1 + radius * dires[i]
            tmp_norm = np.linalg.norm(all_points - np.expand_dims(
                tmp_center, 0).repeat(all_points.shape[0], axis=0),
                                      axis=1)
            indices = np.array(np.where(tmp_norm < radius)).flatten()
            if indices.shape[0] == 0:
                return True
            tmp_center = test_point1 - radius * dires[i]
            tmp_norm = np.linalg.norm(all_points - np.expand_dims(
                tmp_center, 0).repeat(all_points.shape[0], axis=0),
                                      axis=1)
            indices = np.array(np.where(tmp_norm < radius)).flatten()
            if indices.shape[0] == 0:
                return True
        test_point2 = cent - j * (length2) * dire / res
        for i in range(0, 4):
            tmp_center = test_point2 + radius * dires[i]
            tmp_norm = np.linalg.norm(all_points - np.expand_dims(
                tmp_center, 0).repeat(all_points.shape[0], axis=0),
                                      axis=1)
            indices = np.array(np.where(tmp_norm < radius)).flatten()
            if indices.shape[0] == 0:
                return True
            tmp_center = test_point2 - radius * dires[i]
            tmp_norm = np.linalg.norm(all_points - np.expand_dims(
                tmp_center, 0).repeat(all_points.shape[0], axis=0),
                                      axis=1)
            indices = np.array(np.where(tmp_norm < radius)).flatten()
            if indices.shape[0] == 0:
                return True
    return False


# def isExploreV1(pcds, info, radius=8*screw_setting.screw_radius):
#     length1 = info[4]
#     all_points = np.empty((0, 3))
#     all_normals = np.empty((0, 3))
#     for pcd in pcds:
#         all_points = np.concatenate([all_points, np.array(pcd.points)], axis=0)
#         all_normals = np.concatenate([all_normals, np.array(pcd.normals)], axis=0)
#     # tree = spatial.KDTree(all_points)
#     dire = np.array(info[0])
#     cent = np.array(info[1])
#     dire1 = np.array([dire[2], 0, -dire[0]])
#     dire1 = dire1/np.linalg.norm(dire1)
#     dire2 = np.cross(dire, dire1)
#     dire3 = dire1 - dire2
#     dire3 = dire3/np.linalg.norm(dire3)
#     dire4 = dire1 + dire2
#     dire4 = dire4/np.linalg.norm(dire4)
#     dires = [dire1, dire2, dire3, dire4]
#     test_point1 = cent + (length1)*dire
#     # ball_centers = []
#     for i in range(0, 4):
#         tmp_center = test_point1 + radius*dires[i]
#         tmp_norm = np.linalg.norm(all_points - np.expand_dims(tmp_center, 0).repeat(all_points.shape[0], axis=0), axis=1)
#         indices = np.array(np.where(tmp_norm < radius)).flatten()
#         if indices.shape[0] == 0:
#             return True
#         tmp_center = test_point1 - radius*dires[i]
#         tmp_norm = np.linalg.norm(all_points - np.expand_dims(tmp_center, 0).repeat(all_points.shape[0], axis=0), axis=1)
#         indices = np.array(np.where(tmp_norm < radius)).flatten()
#         if indices.shape[0] == 0:
#             return True
#     return False


def isExploreV1(pcds, info, radius=2 * screw_setting.screw_radius, rate=1.2):
    length1 = info[4]
    all_points = np.empty((0, 3))
    for pcd in pcds:
        all_points = np.concatenate([all_points, np.array(pcd.points)], axis=0)
    dire = np.array(info[0])
    dire = dire / np.linalg.norm(dire)
    cent = np.array(info[1])
    dire1 = np.array([dire[2], 0, -dire[0]])
    if np.linalg.norm(dire1) == 0:
        dire1 = np.array([0, -dire[2], dire[1]])
    dire1 = dire1 / np.linalg.norm(dire1)
    dire2 = np.cross(dire, dire1)
    dire2 = dire2 / np.linalg.norm(dire2)
    dire3 = dire1 - dire2
    dire3 = dire3 / np.linalg.norm(dire3)
    dire4 = dire1 + dire2
    dire4 = dire4 / np.linalg.norm(dire4)
    dires = [dire1, dire2, dire3, dire4]
    test_point1 = cent + length1 * dire
    diff = all_points - np.expand_dims(test_point1, 0).repeat(
        all_points.shape[0], axis=0)
    diff_norm = np.linalg.norm(diff, axis=1)
    # ball_centers = []
    # cone_pcd = []
    for i in range(0, len(dires)):
        r_dist = np.sqrt(diff_norm**2 - np.abs(np.dot(diff, dires[i].T))**2)
        indices = np.argwhere(r_dist < radius).flatten()
        # tmp_points = all_points[indices]
        # tmp_pcd = o3d.geometry.PointCloud()
        # tmp_pcd.points = o3d.utility.Vector3dVector(tmp_points)
        # cone_pcd.append(tmp_pcd)
        if indices.shape[0] <= 1:
            return True
        flag1 = False
        flag2 = False
        dists = np.dot(diff[indices], dires[i].T)
        for dist in dists:
            if not flag1:
                if dist < 200 and dist > rate * get_screw_radius():
                    flag1 = True
            if not flag2:
                if dist > -200 and dist < -rate * get_screw_radius():
                    flag2 = True
            if flag1 and flag2:
                break
        if not flag1 or not flag2:
            # visualization.points_visualization_by_vtk(cone_pcd, [test_point1])
            # visualization.points_visualization_by_vtk(pcds, [test_point1])
            return True
    # visualization.points_visualization_by_vtk(cone_pcd, [test_point1])
    # visualization.points_visualization_by_vtk(pcds, [test_point1])
    return False


def isExploreV2(pcds, info, radius=2 * screw_setting.screw_radius, rate=1.2):
    length2 = info[5]
    all_points = np.empty((0, 3))
    for pcd in pcds:
        all_points = np.concatenate([all_points, np.array(pcd.points)], axis=0)
    dire = np.array(info[0])
    cent = np.array(info[1])
    dire1 = np.array([dire[2], 0, -dire[0]])
    if np.linalg.norm(dire1) == 0:
        dire1 = np.array([0, -dire[2], dire[1]])
    dire1 = dire1 / np.linalg.norm(dire1)
    dire2 = np.cross(dire, dire1)
    dire2 = dire2 / np.linalg.norm(dire2)
    dire3 = dire1 - dire2
    dire3 = dire3 / np.linalg.norm(dire3)
    dire4 = dire1 + dire2
    dire4 = dire4 / np.linalg.norm(dire4)
    dires = [dire1, dire2, dire3, dire4]
    test_point2 = cent - (length2) * dire
    diff = all_points - np.expand_dims(test_point2, 0).repeat(
        all_points.shape[0], axis=0)
    diff_norm = np.linalg.norm(diff, axis=1)
    cone_pcd = []
    for i in range(0, len(dires)):
        r_dist = np.sqrt(diff_norm**2 - np.abs(np.dot(diff, dires[i].T))**2)
        indices = np.argwhere(r_dist < radius).flatten()
        tmp_points = all_points[indices]
        tmp_pcd = o3d.geometry.PointCloud()
        tmp_pcd.points = o3d.utility.Vector3dVector(tmp_points)
        cone_pcd.append(tmp_pcd)
        if indices.shape[0] <= 1:
            return True
        flag1 = False
        flag2 = False
        dists = np.dot(diff[indices], dires[i].T)
        for dist in dists:
            if not flag1:
                if dist < 200 and dist > rate * get_screw_radius():
                    flag1 = True
            if not flag2:
                if dist > -200 and dist < -rate * get_screw_radius():
                    flag2 = True
            if flag1 and flag2:
                break
        if not flag1 or not flag2:
            # visualization.points_visualization_by_vtk(cone_pcd, [test_point2])
            # visualization.points_visualization_by_vtk(pcds, [test_point2])
            return True
    # visualization.points_visualization_by_vtk(cone_pcd, [test_point2])
    # visualization.points_visualization_by_vtk(pcds, [test_point2])
    return False


# def isExploreV2(pcds, info, radius=8*screw_setting.screw_radius):
#     length2 = info[5]
#     all_points = np.empty((0, 3))
#     all_normals = np.empty((0, 3))
#     for pcd in pcds:
#         all_points = np.concatenate([all_points, np.array(pcd.points)], axis=0)
#         all_normals = np.concatenate([all_normals, np.array(pcd.normals)], axis=0)
#     # tree = spatial.KDTree(all_points)
#     dire = np.array(info[0])
#     cent = np.array(info[1])
#     dire1 = np.array([dire[2], 0, -dire[0]])
#     dire1 = dire1/np.linalg.norm(dire1)
#     dire2 = np.cross(dire, dire1)
#     dire3 = dire1 - dire2
#     dire3 = dire3/np.linalg.norm(dire3)
#     dire4 = dire1 + dire2
#     dire4 = dire4/np.linalg.norm(dire4)
#     dires = [dire1, dire2, dire3, dire4]

#     test_point2 = cent - (length2)*dire
#     for i in range(0, 4):
#         tmp_center = test_point2 + radius*dires[i]
#         tmp_norm = np.linalg.norm(all_points - np.expand_dims(tmp_center, 0).repeat(all_points.shape[0], axis=0), axis=1)
#         indices = np.array(np.where(tmp_norm < radius)).flatten()
#         if indices.shape[0] == 0:
#             return True
#         tmp_center = test_point2 - radius*dires[i]
#         tmp_norm = np.linalg.norm(all_points - np.expand_dims(tmp_center, 0).repeat(all_points.shape[0], axis=0), axis=1)
#         indices = np.array(np.where(tmp_norm < radius)).flatten()
#         if indices.shape[0] == 0:
#             return True
#     return False


def isExplore_vis(pcds, info, radius=8 * screw_setting.screw_radius):
    length1 = info[4]
    length2 = info[5]
    all_points = np.empty((0, 3))
    all_normals = np.empty((0, 3))
    for pcd in pcds:
        all_points = np.concatenate([all_points, np.array(pcd.points)], axis=0)
        all_normals = np.concatenate(
            [all_normals, np.array(pcd.normals)], axis=0)
    # tree = spatial.KDTree(all_points)
    dire = np.array(info[0])
    cent = np.array(info[1])
    dire1 = np.array([dire[2], 0, -dire[0]])
    dire1 = dire1 / np.linalg.norm(dire1)
    dire2 = np.cross(dire, dire1)
    dire3 = dire1 - dire2
    dire3 = dire3 / np.linalg.norm(dire3)
    dire4 = dire1 + dire2
    dire4 = dire4 / np.linalg.norm(dire4)
    dires = [dire1, dire2, dire3, dire4]
    test_point1 = cent + (length1) * dire
    test_point2 = cent - (length2) * dire
    ball_centers = []
    for i in range(0, 4):
        ball_centers.append(test_point1 + radius * dires[i])
        ball_centers.append(test_point1 - radius * dires[i])
        ball_centers.append(test_point2 + radius * dires[i])
        ball_centers.append(test_point2 - radius * dires[i])
    visualization.points_visualization_by_vtk(pcds,
                                              ball_centers,
                                              radius=radius)


def get_optimal_info(path_info,
                     rest_pcds,
                     rest_pcds_for_explore,
                     eps=screw_setting.screw_radius,
                     dist_eps=screw_setting.dist_eps):
    rf_path_info = []
    # allPoints = np.empty((0, 3))
    # all_points = []
    # for pcd in all_pcds:
    #     points = np.asarray(pcd.points)
    #     allPoints = np.concatenate([allPoints, points], axis=0)
    #     all_points.append(points)
    rest_points = []
    restPoints = np.empty((0, 3))
    for pcd in rest_pcds:
        points = np.asarray(pcd.points)
        restPoints = np.concatenate([restPoints, points], axis=0)
        rest_points.append(points)
    # tree = spatial.KDTree(allPoints)
    matched_pcds = []
    allCenter = np.mean(restPoints, axis=0)
    # print("\n\n\033[31mPath program: there are %d screws needed to be processed.\033[0m" % len(path_info))
    for i in range(len(path_info)):
        start = time.time()
        info = path_info[i]
        dire = info[0]
        cent = info[1]
        id1 = info[2]
        id2 = info[3]
        # path_points = np.concatenate([all_points[id1], all_points[id2]], axis=0)
        cone = get_cone(dire)
        if i != 0 and path_info[i - 1][2] == id1 and path_info[i - 1][3] == id2 and np.linalg.norm(path_info[i - 1][0] - dire) < 1e-2 and len(rf_path_info) != 0:
            # cone = [rf_path_info[len(rf_path_info) - 1][0]]
            # continue
            # cent = -0.8*rf_path_info[len(rf_path_info) - 1][1] + 1.8*cent
            cone = get_cone(rf_path_info[-1][0], angle=np.pi / 3)
        best_dir = dire
        best_length1 = 0
        best_length2 = 0
        # close_length1 = 0
        close_length2 = 0
        # tmp = allPoints - np.expand_dims(cent, 0).repeat(allPoints.shape[0], axis=0)
        # tmp = path_points - np.expand_dims(cent, 0).repeat(path_points.shape[0], axis=0)
        cent_var = restPoints - np.expand_dims(cent, 0).repeat(restPoints.shape[0], axis=0)
        norm = np.linalg.norm(cent_var, axis=1)
        best_cone_pcd = []
        # best_explore_points = []
        for j in range(len(cone)):  # tqdm(range(len(cone)), desc="\033[31mThe %dth screw:\033[0m" % (i + 1),):
            n_dir = cone[j]
            r_dist = np.sqrt(norm**2 - np.abs(np.dot(cent_var, n_dir.T))**2)
            indices = np.argwhere(r_dist < eps).flatten()
            if indices.shape[0] == 0:
                continue
            tmp_points = restPoints[indices]
            y_pred = DBSCAN(eps=dist_eps / 2).fit_predict(tmp_points)
            y_uniq = np.unique(np.array(y_pred))
            # center_list = []
            fp_list = []
            cp_list = []
            ps = None
            cone_pcd = []
            explore_points = []
            for y in y_uniq:
                idx = np.argwhere(y_pred == y).flatten()
                ps = np.array(tmp_points[idx])
                # if ps.shape[0] < 200:
                #     continue
                dist = np.dot(ps, n_dir.T)
                # center_list.append(np.mean(ps, axis=0))
                cp_list.append(ps[np.argmin(dist).flatten()[0]])
                fp_list.append(ps[np.argmax(dist).flatten()[0]])
                tmp_pcd = o3d.geometry.PointCloud()
                tmp_pcd.points = o3d.utility.Vector3dVector(ps)
                cone_pcd.append(tmp_pcd)
            # center_list = np.array(center_list)
            fp_list = np.array(fp_list)
            cp_list = np.array(cp_list)
            ori_cent = np.expand_dims(cent, 0).repeat(fp_list.shape[0], axis=0)
            # dif_cent = center_list - ori_cent
            dir_fp = fp_list - ori_cent
            dir_cp = cp_list - ori_cent
            # com_cent = np.linalg.norm(dif_cent, axis=1)
            com_fp = np.linalg.norm(dir_fp, axis=1)
            com_cp = np.linalg.norm(dir_cp, axis=1)
            # index = np.argmin(com_cent).flatten()[0]
            length1 = 0
            length2 = 0
            tmp_length1 = 0
            tmp_length2 = 0
            idx1 = -1
            idx2 = -1
            if com_fp.shape[0] <= 1:
                continue
            for k in range(com_fp.shape[0]):
                if com_fp[k] < com_cp[k]:
                    tmp_com = com_cp[k].copy()
                    com_cp[k] = com_fp[k]
                    com_fp[k] = tmp_com
                    tmp_diff = dir_cp[k].copy()
                    dir_cp[k] = dir_fp[k]
                    dir_fp[k] = tmp_diff
            tmp_com_cp = np.array([com_cp[0]])
            tmp_com_fp = np.array([com_fp[0]])
            tmp_dir_cp = np.array([dir_cp[0]])
            for m in range(1, com_cp.shape[0]):
                for k in range(tmp_com_cp.shape[0]):
                    if com_cp[m] <= tmp_com_cp[k]:
                        tmp_com_cp = np.insert(tmp_com_cp,
                                               k,
                                               com_cp[m],
                                               axis=0)
                        tmp_com_fp = np.insert(tmp_com_fp,
                                               k,
                                               com_fp[m],
                                               axis=0)
                        tmp_dir_cp = np.insert(tmp_dir_cp,
                                               k,
                                               dir_cp[m],
                                               axis=0)
                        break
                    elif k == tmp_com_cp.shape[0] - 1:
                        tmp_com_cp = np.concatenate([tmp_com_cp, [com_cp[m]]],
                                                    axis=0)
                        tmp_com_fp = np.concatenate([tmp_com_fp, [com_fp[m]]],
                                                    axis=0)
                        tmp_dir_cp = np.concatenate([tmp_dir_cp, [dir_cp[m]]],
                                                    axis=0)
                        break
            com_cp = tmp_com_cp
            com_fp = tmp_com_fp
            dir_cp = tmp_dir_cp
            explore_flag1 = False
            explore_flag2 = False
            for k in range(com_fp.shape[0]):
                pn = np.dot(dir_cp[k], n_dir.T)
                if pn > 0 and not explore_flag1:
                    tmp_length1 = com_cp[k]
                    # visualization.points_visualization_by_vtk(rest_pcds_for_explore, centers=[cent+(tmp_length1 + com_fp[idx1])/2*n_dir])
                    if idx1 == -1:
                        explore_length1 = tmp_length1 / 2
                        if not isExploreV1(rest_pcds_for_explore, [n_dir, cent, id1, id2, explore_length1, 0]):
                            length1 = tmp_length1
                            idx1 = k
                        else:
                            explore_points.append(cent + n_dir * explore_length1)
                            explore_flag1 = True
                    else:
                        explore_length1 = (tmp_length1 + com_fp[idx1]) / 2
                        explore_points.append(cent + n_dir * explore_length1)
                        explore_points.append(cent + n_dir * (com_cp[idx1] + com_fp[idx1]) / 2)
                        if not isExploreV1(rest_pcds_for_explore, [n_dir, cent, id1, id2, explore_length1, 0]) and not isExploreV1(rest_pcds_for_explore, [ n_dir, cent, id1, id2, (com_cp[idx1] + com_fp[idx1]) / 2, 0]):
                            length1 = tmp_length1
                            idx1 = k
                        else:
                            explore_flag1 = True
                elif pn <= 0 and not explore_flag2:
                    tmp_length2 = com_cp[k]
                    # visualization.points_visualization_by_vtk(rest_pcds_for_explore, centers=[cent+(tmp_length1 + com_fp[idx1])/2*n_dir])
                    if idx2 == -1:
                        explore_length2 = tmp_length2 / 2
                        explore_points.append(cent - n_dir * explore_length2)
                        if not isExploreV2(rest_pcds_for_explore, [n_dir, cent, id1, id2, 0, explore_length2]):
                            length2 = tmp_length2
                            idx2 = k
                        else:
                            explore_flag2 = True
                    else:
                        explore_length2 = (tmp_length2 + com_fp[idx2]) / 2
                        explore_points.append(cent - n_dir * explore_length2)
                        explore_points.append(cent - n_dir * (com_cp[idx2] + com_fp[idx2]) / 2)
                        if not isExploreV2(rest_pcds_for_explore, [n_dir, cent, id1, id2, 0, explore_length2]) and not isExploreV2(rest_pcds_for_explore, [n_dir, cent, id1, id2, 0, (com_cp[idx2] + com_fp[idx2]) / 2]):
                            length2 = tmp_length2
                            idx2 = k
                        else:
                            explore_flag2 = True
            if idx1 == -1 or idx2 == -1 or length1 <= 3 * dist_eps or length2 <= 3 * dist_eps:
                # visualization.points_visualization_by_vtk(rest_pcds_for_explore, centers=[cent, cent+length1*n_dir, cent-length2*n_dir])
                continue
            if np.linalg.norm(cent + n_dir * length1 - allCenter) > np.linalg.norm(cent - n_dir * length2 - allCenter):
                n_dir = -n_dir
                tmp_idx = idx1
                idx1 = idx2
                idx2 = tmp_idx
                length1 = com_cp[idx1]
                length2 = max(com_fp[idx2], com_cp[idx2] + 4)
            
            interference_info = rf_path_info.copy()
            for k in range(len(rf_path_info) + 1, len(path_info)):
                if k != i:
                    interference_info.append([
                        path_info[k][0], path_info[k][1], path_info[k][2],
                        path_info[k][3], dist_eps, dist_eps
                    ])
            ret, _ = isInterference([n_dir, cent, id1, id2, length1, length2],
                                    interference_info)
            new_length1 = length1
            new_length2 = length2
            flag_interference1 = True
            ret1 = ret
            ret2 = ret
            if ret and isInterference(
                [n_dir, cent, id1, id2, 3 * dist_eps, 3 * dist_eps],
                    interference_info)[0]:
                continue
            while ret1 or ret2:
                if flag_interference1:
                    new_length1 = new_length1 * 0.9
                    if new_length1 < max(best_length1, 3 * dist_eps):
                        new_length1 = 0
                        break
                    com_cp[idx1] = new_length1
                    com_fp[idx1] = new_length1
                    ret1, _ = isInterference(
                        [n_dir, cent, id1, id2, new_length1, 3 * dist_eps],
                        interference_info)
                    if not ret1:
                        flag_interference1 = False
                else:
                    new_length2 = new_length2 * 0.9
                    if new_length2 < max(best_length2, 3 * dist_eps):
                        new_length2 = 0
                        break
                    com_cp[idx2] = new_length2
                    com_fp[idx2] = new_length2
                    ret2, _ = isInterference(
                        [n_dir, cent, id1, id2, new_length1, new_length2],
                        interference_info)
            length1 = new_length1
            length2 = new_length2
            # if length1 >= 60:
            #     length1 = 60
            #     com_cp[idx1] = length1
            #     com_fp[idx1] = length1
            if np.abs(length1) + np.abs(length2) < 8 * dist_eps or min(length1, length2) < 3 * dist_eps or (min(length1, length2) / max(length1, length2) < 0.33 and min(length1, length2) < 20):  # or isExplore(rest_pcds_for_explore, [n_dir, cent, id1, id2, length1, length2]):
                continue
            # if length1 < 30 and length1/length2 < 0.33:
            #     continue
            continue_ornot = True
            if min(length1, length2) > min(best_length1, best_length2):
                continue_ornot = False
            elif (max(length1, length2) - max(best_length1, best_length2)) / (min(best_length1, best_length2) - min(length1, length2) + 0.01) > 2:
                continue_ornot = False
            if length1 >= best_length1 and com_cp[idx1] + com_cp[idx2] > best_length1 + close_length2:
                continue_ornot = False
            elif com_cp[idx2] > close_length2 and (length2 - best_length2) / (best_length1 - length1 + 0.01) > 0.8:
                continue_ornot = False
            # t_length1 = length1
            # t_length2 = length2
            # if 0.8*length2 + 0.2*length1 > 0.8*best_length2 + 0.2*best_length1: # and length1/length2 > 0.2 and length2/length1 > 0.2:
            #     continue_ornot = False
            if continue_ornot:
                continue
            best_length1 = length1
            best_length2 = length2
            # close_length1 = com_cp[idx1]
            close_length2 = com_cp[idx2]
            best_dir = n_dir
            best_cone_pcd = cone_pcd
            # best_explore_points = explore_points
        # if np.abs(length1) + np.abs(length2) >= 10*dist_eps:
        end = time.time()
        if best_length1 == 0 or best_length2 == 0:
            continue
        if i != 0 and len(rf_path_info) != 0 and rf_path_info[-1][2] == id1 and rf_path_info[-1][3] == id2 and np.linalg.norm(path_info[i-1][0] - dire) < 1e-2:
            if np.linalg.norm(cent - rf_path_info[-1][1]) < 16 * get_screw_radius() and np.abs(rf_path_info[-1][4] + rf_path_info[-1][5] - (best_length1 + best_length2)) > 40:
                if best_length1 + best_length2 > rf_path_info[-1][4] + rf_path_info[-1][5]:
                    rf_path_info[-1] = [best_dir, cent, id1, id2, best_length1, best_length2]
                    print("螺钉%d方向规划时间:%.2f秒, length1:%.2f, length2:%.2f" % (len(rf_path_info) + 1, end - start, best_length1, best_length2))
                    matched_pcds.extend(best_cone_pcd)
                    continue
                else:
                    continue
        print("螺钉%d方向规划时间:%.2f秒, length1:%.2f, length2:%.2f" % (len(rf_path_info) + 1, end - start, best_length1, best_length2))
        rf_path_info.append([best_dir, cent, id1, id2, best_length1, best_length2])
        matched_pcds.extend(best_cone_pcd)
        # isExplore_vis(rest_pcds, [best_dir, cent, id1, id2, (tmp_length1 + com_cp[k])/2, (tmp_length2 + com_cp[k])/2])

        # visualization.points_visualization_by_vtk(rest_pcds, best_explore_points)
        # visualization.points_visualization_by_vtk(best_cone_pcd, best_explore_points)
        # tmp_length1 = 0
        # tmp_length2 = 0
        # for k in range(com_fp.shape[0]):
        #     pn = np.dot(dir_cp[k], n_dir.T)
        #     if pn > 0:
        #         isExplore_vis(rest_pcds, [best_dir, cent, id1, id2, (tmp_length1 + com_cp[k])/2, (tmp_length2 + com_cp[k])/2])
        #         tmp_length1 = com_cp[k]
        #     elif pn < 0:
        #         isExplore_vis(rest_pcds, [best_dir, cent, id1, id2, (tmp_length1 + com_cp[k])/2, (tmp_length2 + com_cp[k])/2])
        #         tmp_length2 = com_cp[k]
    # visualization.points_visualization_by_vtk(matched_pcds)
    # matched_pcds.extend(rest_pcds)
    # for i in range(len(rf_path_info)):
    #     isExplore_vis(matched_pcds, rf_path_info[i])
    # visualization.points_visualization_by_vtk(matched_pcds)
    # for i in range(len(rf_path_info)):
    #     if i in [0, 4]:
    #         rf_path_info[i][0] = -rf_path_info[i][0]
    #         tmp = rf_path_info[i][4]
    #         rf_path_info[i][4] = rf_path_info[i][5] - 4
    #         rf_path_info[i][5] = tmp + 4
    return rf_path_info


def path_program(frac_pcds, all_pcds, rest_pcds):
    start = time.time()
    refine_cluster = get_effect_points(frac_pcds)
    path_info = []
    all_points = []
    frac_points = []
    sizes = [0]
    id_record = []
    frac_id = []
    for pcd in frac_pcds:
        frac_points.append(np.asarray(pcd.points))
    for i in range(len(all_pcds)):
        pcd = all_pcds[i]
        all_points.append(np.asarray(pcd.points))
        sizes.append(sizes[i] + np.asarray(pcd.points).shape[0])
        id_record.append(0)
    for i in range(len(refine_cluster)):  #tqdm(range(len(refine_cluster)), desc="\033[31mInitializing implantation centers\033[0m"):
        points = refine_cluster[i]
        points1 = points[0]
        points2 = points[1]
        path_dir = get_screw_dir_by_SVM(points1, points2)
        # path_dir = np.mean(points1, axis=0) - np.mean(points2, axis=0)
        # path_dir = path_dir/np.linalg.norm(path_dir)
        # path_dir = get_screw_dir_by_norm2(points1, points2) # there exist some problems
        # path_dir = get_screw_dir_by_ransac(points1, points2)
        path_center = get_screw_implant_position_by_Chebyshev_center(
            points1, points2)
        tmp_p1 = points1[1, :]
        tmp_p2 = points2[1, :]
        id1 = None
        id2 = None
        # pcd1 = o3d.geometry.PointCloud()
        # pcd1.points = o3d.utility.Vector3dVector(points1)
        # pcd2 = o3d.geometry.PointCloud()
        # pcd2.points = o3d.utility.Vector3dVector(points2)
        # visualization.points_visualization_by_vtk([pcd1, pcd2], [path_center])
        f_id1 = None
        f_id2 = None
        for j in range(len(frac_points)):
            t1 = np.sum(np.abs(frac_points[j] - np.expand_dims(
                tmp_p1, 0).repeat(frac_points[j].shape[0], axis=0)),
                        axis=1)
            t2 = np.sum(np.abs(frac_points[j] - np.expand_dims(
                tmp_p2, 0).repeat(frac_points[j].shape[0], axis=0)),
                        axis=1)
            if np.where(t1 == 0)[0].shape[0] != 0:
                f_id1 = j
            if np.where(t2 == 0)[0].shape[0] != 0:
                f_id2 = j
        frac_id.append([f_id1, f_id2])

        allPoints1 = np.empty((0, 3))
        for j in range(len(all_points)):
            allPoints1 = np.concatenate([allPoints1, all_points[j]], axis=0)
        # visualization.points_visualization_by_vtk(all_pcds, [np.mean(allPoints1, axis=0)], radius=10)
        t1 = np.sum(np.abs(
            allPoints1 -
            np.expand_dims(tmp_p1, 0).repeat(allPoints1.shape[0], axis=0)),
                    axis=1)
        index1 = np.argmin(t1).flatten()[0]
        for j in range(len(sizes)):
            if index1 == 0:
                id1 = 0
                break
            elif index1 < sizes[j]:
                id1 = j - 1
                break
        allPoints2 = np.empty((0, 3))
        for j in range(len(all_points)):
            if j != id1:
                allPoints2 = np.concatenate([allPoints2, all_points[j]],
                                            axis=0)
            else:
                allPoints2 = np.concatenate(
                    [allPoints2,
                     np.ones([all_points[j].shape[0], 3]) * 10000],
                    axis=0)
        t2 = np.linalg.norm(
            allPoints2 -
            np.expand_dims(tmp_p2, 0).repeat(allPoints2.shape[0], axis=0),
            axis=1)
        index2 = np.argmin(t2).flatten()[0]
        for j in range(len(sizes)):
            if index2 == 0:
                if id1 == 0:
                    id2 = 1
                else:
                    id2 = 0
                break
            elif index2 < sizes[j]:
                id2 = j - 1
                break
        # path_dir = np.mean(all_points[id1], axis=0) - np.mean(all_points[id2], axis=0)
        # path_dir = path_dir/np.linalg.norm(path_dir)
        path_info.append([path_dir, path_center, id1, id2, 0, 0])
    # refine the number of implanted screws
    for info in path_info:
        id_record[info[2]] = id_record[info[2]] + 1
        id_record[info[3]] = id_record[info[3]] + 1
    for i in range(len(id_record)):
        if id_record[i] < 2:
            for j in range(len(path_info)):
                info = path_info[j]
                if info[2] == i or info[3] == i:
                    frac_idx = None
                    for k in range(len(frac_id)):
                        if frac_id[k][0] == i or frac_id[k][1] == i:
                            frac_idx = k
                            break
                    point1 = refine_cluster[frac_idx][0]
                    point2 = refine_cluster[frac_idx][1]
                    points = np.concatenate([point1, point2], axis=0)
                    # center = np.mean(points, axis=0)
                    pca = PCA()
                    pca.fit(points)
                    vec = np.array(pca.components_[0, :])
                    vec = vec / np.linalg.norm(vec)

                    # c_dsbt = np.dot(center, vec.T)
                    dsp_dsbt = np.dot(points, vec.T)
                    min_val = dsp_dsbt[np.argmin(dsp_dsbt)]
                    max_val = dsp_dsbt[np.argmax(dsp_dsbt)]
                    res = max_val - min_val
                    length_rate = 20
                    if res > length_rate * get_screw_radius():
                        # dsp_dsbt1 = np.dot(points1, vec)
                        # # min_val1 = dsp_dsbt1[np.argmin(dsp_dsbt1)]
                        # # max_val1 = dsp_dsbt1[np.argmax(dsp_dsbt1)]
                        # indices11 = np.array(np.where(dsp_dsbt1 < c_dsbt)).flatten()
                        # indices12 = np.array(np.where(dsp_dsbt1 > c_dsbt)).flatten()

                        # dsp_dsbt2 = np.dot(points2, vec)
                        # # min_val2 = dsp_dsbt2[np.argmin(dsp_dsbt2)]
                        # # max_val2 = dsp_dsbt2[np.argmax(dsp_dsbt2)]
                        # indices21 = np.array(np.where(dsp_dsbt2 < c_dsbt)).flatten()
                        # indices22 = np.array(np.where(dsp_dsbt2 > c_dsbt)).flatten()

                        # ps11 = points1[indices11]
                        # ps12 = points1[indices21]

                        # ps21 = points2[indices12]
                        # ps22 = points2[indices22]

                        path_center1, path_center2 = get_2_screw_implant_positions_by_Chebyshev_center(
                            point1, point2, length_rate / 2)
                        info1 = [info[0], path_center1, info[2], info[3], 0, 0]
                        info2 = [info[0], path_center2, info[2], info[3], 0, 0]
                        id_record[info[2]] = id_record[info[2]] + 1
                        id_record[info[3]] = id_record[info[3]] + 1
                        path_info[j] = info2
                        j = j + 1
                        path_info.insert(j, info1)
                        break
    # path_info = add_screw_length(path_info, all_pcds)
    path_info = refine_path_info(path_info, rest_pcds)
    end = time.time()
    print("循环运行时间:%.2f秒" % (end - start))
    return path_info


def refine_path_info(path_info,
                     pcds,
                     radius=screw_setting.path_refine_radius,
                     length_eps=screw_setting.length_eps):
    rf_path_info = []
    all_points = np.empty((0, 3))
    allPoints = []
    for pcd in pcds:
        points = np.asarray(pcd.points)
        all_points = np.concatenate([all_points, points], axis=0)
        allPoints.append(points)
    tree = spatial.KDTree(all_points)
    centers = []
    frac_size = []
    for i in range(len(path_info)):
        info = path_info[i]
        point = info[1]
        centers.append(point)
        direc = info[0]
        id1 = info[2]
        id2 = info[3]
        indices = tree.query_ball_point(point, radius, workers=-1)
        indices = np.array(indices).flatten()
        if i != 0 and path_info[i - 1][2] == id1 and path_info[i - 1][3] == id2 and np.linalg.norm(path_info[i - 1][0] - direc) < 1e-2:
            rf_path_info.append([rf_path_info[-1][0], point, id1, id2, 0, 0])
            frac_size.append(indices.shape[0])
            continue
        points = all_points[indices]
        pca = PCA()
        pca.fit(points)
        vec = np.array(pca.components_[0, :])
        vec = vec / np.linalg.norm(vec)
        if np.dot(vec, direc.T) < 0:
            vec = -vec
        # rf_direc = (direc + vec)/2
        rf_direc = vec  #+ path_info[i][0]
        # rf_direc = rf_direc/np.linalg.norm(rf_direc)
        rf_path_info.append([rf_direc, point, id1, id2, 0, 0])
        frac_size.append(indices.shape[0])
    # rf_path_info = add_screw_length(rf_path_info, pcds)
    for i in range(len(path_info)):
        # tmp = pca.explained_variance_ratio_
        # tmp = 0
        dire = np.mean(allPoints[path_info[i][2]], axis=0) - np.mean(
            allPoints[path_info[i][3]], axis=0)
        dire = dire / np.linalg.norm(dire)
        rf_path_info[i][0] = dire
        # if np.abs(np.dot(dire, vec.T)) > 0.5 and frac_size[i] > 30000:
        #     rf_path_info[i][0] = dire
        #     rf_path_info[i][4] = 0
        #     rf_path_info[i][5] = 0
    # visualization.points_visualization_by_vtk(pcds, centers, radius)
    return rf_path_info


def refine_path_info_v1(path_info,
                        pcds,
                        radius=screw_setting.path_refine_radius):
    rf_path_info = []
    all_points = np.empty((0, 3))
    for pcd in pcds:
        points = np.asarray(pcd.points)
        all_points = np.concatenate([all_points, points], axis=0)
    tree = spatial.KDTree(all_points)
    for info in path_info:
        point = info[1]
        direc = info[0]
        id1 = info[2]
        id2 = info[3]
        indices = tree.query_ball_point(point, radius, workers=-1)
        indices = np.array(indices).flatten()
        points = all_points[indices]
        pca = PCA()
        pca.fit(points)
        vec = np.array(pca.components_[0, :])
        vec = vec / np.linalg.norm(vec)
        if np.dot(vec, direc.T) < 0:
            vec = -vec
        # rf_direc = (direc + vec)/2
        rf_direc = vec
        rf_path_info.append([rf_direc, point, id1, id2])
    rf_path_info = add_screw_length(rf_path_info, pcds)
    return rf_path_info


def refine_path_info_v2(path_info,
                        pcds,
                        radius=screw_setting.path_refine_radius):
    rf_path_info = []
    all_points = np.empty((0, 3))
    for pcd in pcds:
        points = np.asarray(pcd.points)
        all_points = np.concatenate([all_points, points], axis=0)
    tree = spatial.KDTree(all_points)
    for i in range(len(path_info)):
        info = path_info[i]
        point = info[1]
        direc = info[0]
        id1 = info[2]
        id2 = info[3]
        indices = tree.query_ball_point(point, radius, workers=-1)
        indices = np.array(indices).flatten()
        points = all_points[indices]
        pca = PCA()
        pca.fit(points)
        vec1 = np.array(pca.components_[0, :])
        vec2 = np.array(pca.components_[1, :])
        vec3 = np.array(pca.components_[2, :])
        dsp_dsbt1 = np.dot(points, vec1)
        dsp_dsbt2 = np.dot(points, vec2)
        dsp_dsbt3 = np.dot(points, vec3)
        ctbt1 = dsp_dsbt1[np.argmax(dsp_dsbt1)] - dsp_dsbt1[np.argmin(
            dsp_dsbt1)]
        ctbt2 = dsp_dsbt2[np.argmax(dsp_dsbt2)] - dsp_dsbt2[np.argmin(
            dsp_dsbt2)]
        ctbt3 = dsp_dsbt3[np.argmax(dsp_dsbt3)] - dsp_dsbt3[np.argmin(
            dsp_dsbt3)]
        # ctbt1 = pca.explained_variance_ratio_[0]
        # ctbt2 = pca.explained_variance_ratio_[1]
        # ctbt3 = pca.explained_variance_ratio_[2]
        if np.dot(vec1, direc.T) < 0:
            vec1 = -vec1
        rf_direc = relu_refine_dir_v2(ctbt1, ctbt2, ctbt3, direc, vec1)
        # rf_direc = vec
        if rf_direc is not None:
            rf_path_info.append([rf_direc, point, id1, id2])
    rf_path_info = add_screw_length(rf_path_info, pcds)
    return rf_path_info


def relu_refine_dir_v2(ctbt1,
                       ctbt2,
                       ctbt3,
                       direc1,
                       direc2,
                       radius=screw_setting.screw_radius,
                       eps1=screw_setting.rrd_eps_max,
                       eps2=screw_setting.rrd_eps_min):
    ctbt_rate1 = ctbt2 / ctbt1
    ctbt_rate2 = ctbt3 / ctbt1
    if ctbt3 < 2.5 * radius:
        return None
    elif ctbt3 < 4 * radius or ctbt_rate1 < eps1:
        return direc2
    elif ctbt3 > 8 * radius or ctbt_rate2 > eps2:
        return direc1
    else:
        return (direc1 * ctbt_rate2 + direc2 * (1 - ctbt_rate2))


def refine_path_info_v3(path_info,
                        pcds,
                        radius1=screw_setting.path_refine_radius,
                        radius2=screw_setting.screw_radius):
    rf_path_info = []
    all_points = []
    allPoints = np.empty((0, 3))
    trees = []
    for pcd in pcds:
        points = np.asarray(pcd.points)
        all_points.append(points)
        trees.append(spatial.KDTree(points))
        allPoints = np.concatenate([allPoints, points], axis=0)
    tree = spatial.KDTree(allPoints)
    for info in path_info:
        point = info[1]
        direc = info[0]
        id1 = info[2]
        id2 = info[3]
        # indices = tree.query_ball_point(point, 6*radius2, workers=-1)
        # points = allPoints[indices]
        # pca = PCA()
        # pca.fit(points)
        # vec2 = np.array(pca.components_[1, :])
        # dsp_dsbt = np.dot(points, vec2)
        # ctbt = dsp_dsbt[np.argmax(dsp_dsbt)] - dsp_dsbt[np.argmin(dsp_dsbt)]
        # if ctbt < 5*radius2:
        #     continue
        indices = tree.query_ball_point(point, radius1, workers=-1)
        points = allPoints[indices]
        pca = PCA()
        pca.fit(points)
        vec = np.array(pca.components_[0, :])

        indices1 = trees[id1].query_ball_point(point, radius1, workers=-1)
        points1 = all_points[id1][indices1]
        pca1 = PCA()
        pca1.fit(points1)
        vec11 = np.array(pca1.components_[0, :])
        vec21 = np.array(pca1.components_[1, :])
        vec31 = np.array(pca1.components_[2, :])
        dsp_dsbt11 = np.dot(points1, vec11)
        dsp_dsbt21 = np.dot(points1, vec21)
        dsp_dsbt31 = np.dot(points1, vec31)
        ctbt11 = dsp_dsbt11[np.argmax(dsp_dsbt11)] - dsp_dsbt11[np.argmin(
            dsp_dsbt11)]
        ctbt21 = dsp_dsbt21[np.argmax(dsp_dsbt21)] - dsp_dsbt21[np.argmin(
            dsp_dsbt21)]
        ctbt31 = dsp_dsbt31[np.argmax(dsp_dsbt31)] - dsp_dsbt31[np.argmin(
            dsp_dsbt31)]
        # ctbt11 = pca1.explained_variance_ratio_[0]
        # ctbt21 = pca1.explained_variance_ratio_[1]
        # ctbt31 = pca1.explained_variance_ratio_[2]
        ########################################################
        indices2 = trees[id2].query_ball_point(point, radius1, workers=-1)
        points2 = all_points[id2][indices2]
        pca2 = PCA()
        pca2.fit(points2)
        vec12 = np.array(pca2.components_[0, :])
        vec22 = np.array(pca2.components_[1, :])
        vec32 = np.array(pca2.components_[2, :])
        dsp_dsbt12 = np.dot(points2, vec12)
        dsp_dsbt22 = np.dot(points2, vec22)
        dsp_dsbt32 = np.dot(points2, vec32)
        ctbt12 = dsp_dsbt12[np.argmax(dsp_dsbt12)] - dsp_dsbt12[np.argmin(
            dsp_dsbt12)]
        ctbt22 = dsp_dsbt22[np.argmax(dsp_dsbt22)] - dsp_dsbt22[np.argmin(
            dsp_dsbt22)]
        ctbt32 = dsp_dsbt32[np.argmax(dsp_dsbt32)] - dsp_dsbt32[np.argmin(
            dsp_dsbt32)]
        # ctbt12 = pca2.explained_variance_ratio_[0]
        # ctbt22 = pca2.explained_variance_ratio_[1]
        # ctbt32 = pca2.explained_variance_ratio_[2]
        if np.dot(vec11, direc.T) < 0:
            vec11 = -vec11
        if np.dot(vec12, direc.T) < 0:
            vec12 = -vec12
        rf_direc = relu_refine_dir_v3(ctbt11, ctbt21, ctbt31, ctbt12, ctbt22,
                                      ctbt32, vec, vec11, vec12, direc)
        # rf_direc = vec
        if rf_direc is not None:
            rf_path_info.append([rf_direc, point, id1, id2])
    rf_path_info = add_screw_length(rf_path_info, pcds)
    return rf_path_info


def relu_refine_dir_v3(ctbt11,
                       ctbt21,
                       ctbt31,
                       ctbt12,
                       ctbt22,
                       ctbt32,
                       vec,
                       vec11,
                       vec12,
                       direc,
                       radius=screw_setting.screw_radius,
                       eps1=screw_setting.rrd_eps_max,
                       eps2=screw_setting.rrd_eps_min):
    ctbt_rate11 = ctbt21 / ctbt11
    ctbt_rate21 = ctbt31 / ctbt11
    ctbt_rate12 = ctbt22 / ctbt12
    ctbt_rate22 = ctbt32 / ctbt12
    if ctbt31 < 3 * radius or ctbt32 < 3 * radius:
        return None
    elif ctbt31 < 4 * radius or ctbt_rate11 < eps1 or ctbt32 < 4 * radius or ctbt_rate12 < eps1:
        if ctbt21 < 0.8 * ctbt22:
            return vec11
        elif ctbt21 < 0.8 * ctbt22:
            return vec12
        else:
            return vec
    elif ctbt31 > 8 * radius and ctbt_rate21 > eps2 and ctbt32 > 8 * radius and ctbt_rate22 > eps2:
        return direc
    else:
        return (vec12 / ctbt_rate22 + vec11 / ctbt_rate21 +
                direc) / np.linalg.norm(vec12 / ctbt_rate22 +
                                        vec11 / ctbt_rate21 + direc)


def refine_path_info_v4(path_info, rest_pcds, rest_pcds_for_explore):
    return get_optimal_info(path_info, rest_pcds, rest_pcds_for_explore)

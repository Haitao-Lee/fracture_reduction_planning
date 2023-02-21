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


def get_screw_dir_by_SVM(points1, points2, svm_threshold=screw_setting.svm_threshold):
    plane_normal, _ = geometry.fit_plane_by_svm(points1, points2, svm_threshold)
    return plane_normal


def get_screw_dir_by_Center(points1, points2):
    center1 = np.mean(points1, axis=0)
    center2 = np.mean(points2, axis=0)
    normal = center1 - center2
    normal = normal/np.linalg.norm(normal)
    return normal


def get_screw_dir_by_norm2(points1, points2):
    points = np.concatenate([points1, points2], axis=0)
    plane_info = geometry.fit_plane_by_norm2(np.array(points))
    return plane_info[1]


def get_screw_dir_by_ransac(points1, points2):
    points = np.concatenate([points1, points2], axis=0)
    plane_info, _, _ = geometry.ransac_planefit(points, ransac_n=3, max_dst=screw_setting.ransac_eps)
    return plane_info[3:6]


# def get_screw_implant_position(points1, points2):
#     center1 = np.mean(points1, axis=0)
#     center2 = np.mean(points2, axis=0)
#     # center = (center1 + center2)/2
#     center = (points2.shape[0] * center1 + points1.shape[0] * center2) / (points1.shape[0] + points2.shape[0])
#     return center


# 近似求取切比雪夫中心
def get_screw_implant_position_Chebyshev_center(points1, points2):
    points1 = np.array(points1)
    points2 = np.array(points2)
    all_points = np.concatenate([points1, points2], axis=0)
    pca = PCA()
    pca.fit(all_points)
    vec1 = np.array(pca.components_[0, :])
    vec1 = vec1/np.linalg.norm(vec1)
    vec2 = np.array(pca.components_[1, :])
    vec2 = vec2/np.linalg.norm(vec2)
    dsp_dsbt = np.dot(all_points, vec1)
    min_val = dsp_dsbt[np.argmin(dsp_dsbt)]
    max_val = dsp_dsbt[np.argmax(dsp_dsbt)]
    res = max_val - min_val
    s_r = get_screw_radius()
    initial_center = np.mean(all_points, axis=0)
    if res < 3*s_r:
        return initial_center
    else:
        tmp_position = min_val + 5*s_r
        max_res = 0
        best_center = initial_center
        last_num = 0
        while tmp_position <= max_val - 5*s_r:
            indices = np.array(np.where((dsp_dsbt > tmp_position - 5*s_r) & (dsp_dsbt < tmp_position + 5*s_r))).flatten()
            if indices.shape[0] <= 30:
                tmp_position = tmp_position + s_r
                continue
            tmp_points = all_points[indices]
            # pcd = o3d.geometry.PointCloud()
            # pcd.points = o3d.utility.Vector3dVector(all_points)
            # tmp_pcd = o3d.geometry.PointCloud()
            # tmp_pcd.points = o3d.utility.Vector3dVector(tmp_points)
            # visualization.points_visualization_by_vtk([tmp_pcd, pcd], screw_setting.color)
            tmp_dsp_dsbt = np.dot(tmp_points, vec2)
            tmp_min_val = tmp_dsp_dsbt[np.argmin(tmp_dsp_dsbt)]
            tmp_max_val = tmp_dsp_dsbt[np.argmax(tmp_dsp_dsbt)]
            tmp_res = tmp_max_val - tmp_min_val
            tmp_position = tmp_position + s_r
            if tmp_res > max_res or (tmp_res == max_res and indices.shape[0] > last_num):
                max_res = tmp_res
                last_num = indices.shape[0]
                _, tmp_points, _ = geometry.ransac_planefit(tmp_points, ransac_n=3, max_dst=screw_setting.ransac_eps/5)
                best_center = np.mean(tmp_points, axis=0)
                # min_point = tmp_points[np.argmin(tmp_dsp_dsbt)]
                # max_point = tmp_points[np.argmax(tmp_dsp_dsbt)]
                # best_center = (min_point + max_point)/2
        return best_center


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


def separate_point(points, radius=screw_setting.sp_radius, eps=screw_setting.sp_threshold):
    points = np.array(points)
    all_points = []
    y_pred = DBSCAN(eps=eps).fit_predict(points)
    y_uniq = np.unique(np.array(y_pred))
    for y in y_uniq:
        indices = np.argwhere(y_pred == y).flatten()
        ps = np.array(points[indices])
        if ps.shape[0] / points.shape[0] < 0.2 and ps.shape[0] < 50:
            continue
        pca = PCA()
        pca.fit(points)
        vec = np.array(pca.components_[0, :])
        vec = vec/np.linalg.norm(vec)
        dsp_dsbt = np.dot(ps, vec)
        min_val = dsp_dsbt[np.argmin(dsp_dsbt)]
        max_val = dsp_dsbt[np.argmax(dsp_dsbt)]
        res = max_val - min_val
        if res > 2 * radius:
            indices1 = np.array(np.where(dsp_dsbt < (min_val + max_val) / 2)).flatten()
            indices2 = np.array(np.where(dsp_dsbt > (min_val + max_val) / 2)).flatten()
            ps1 = ps[indices1]
            ps2 = ps[indices2]
            all_points.append(ps1)
            all_points.append(ps2)
            continue
        all_points.append(ps)
    return all_points


def get_effect_points(pcds, threshold=screw_setting.gep_threshold):
    trees = []
    all_points = []
    for pcd in pcds:
        points = np.array(pcd.points)
        tree = spatial.KDTree(points)
        trees.append(tree)
        all_points.append(points)
    finish_indices = []
    match_clusters = []
    for i in range(0, len(all_points)):
        finish_indices.append(i)
        points1 = np.empty((1, 3))
        points2 = np.empty((1, 3))
        for j in range(0, len(trees)):
            if j not in finish_indices:
                _, indices = trees[j].query(all_points[i], 1, workers=-1)
                indices = np.array(indices)
                dist = np.linalg.norm(all_points[i] - all_points[j][indices], axis=1)
                dist_idx = np.argwhere(dist < threshold).flatten()
                points1 = np.concatenate([points1, all_points[i][dist_idx]], axis=0)
                points2 = np.concatenate([points2, all_points[j][indices[dist_idx]]], axis=0)
        if points1.shape[0] > 50:
            match_clusters.append([points1, points2])
    refine_cluster = []
    matched_pcds = []
    for match_cluster in match_clusters:
        points1 = match_cluster[0]
        points2 = match_cluster[1]
        all_points = separate_point(points1)
        tree = spatial.KDTree(points2)
        tmp_cluster = []
        for points in all_points:
            _, indices = tree.query(points, 1, workers=-1)
            tmp_cluster.append([points, points2[indices]])
            pcd_ps = np.concatenate([points, points2[indices]],axis=0)
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(pcd_ps)
            matched_pcds.append(pcd)
        refine_cluster.append(tmp_cluster)
    # visualization.points_visualization_by_vtk(matched_pcds, screw_setting.color)
    return refine_cluster


def get_cone(dire, angle=screw_setting.cone_angle, r_resolution=screw_setting.r_res, c_resolution=screw_setting.c_res):
    orth_dir = np.array([dire[2], 0, -dire[0]])
    orth_dir = orth_dir/np.linalg.norm(orth_dir)
    radius = np.tan(angle)
    rot_mtx = scipy.linalg.expm(np.cross(np.eye(3), dire/scipy.linalg.norm(dire)*2*np.pi/c_resolution))
    cone = [dire]
    for i in range(c_resolution):
        orth_dir = np.dot(rot_mtx, orth_dir)
        for j in range(r_resolution):
            n_dir = dire + orth_dir*radius*j/r_resolution
            n_dir = n_dir/np.linalg.norm(n_dir)
            cone.append(n_dir)
    return cone


def add_screw_length(path_info, pcds, eps=screw_setting.screw_radius, dist_eps=screw_setting.dist_eps):
    rf_path_info = []
    allPoints = np.empty((1, 3))
    for pcd in pcds:
        points = np.asarray(pcd.points)
        allPoints = np.concatenate([allPoints, points], axis=0)
    # tree = spatial.KDTree(allPoints)
    # matched_pcds = []
    for info in path_info:
        dire = info[0]
        cent = info[1]
        id1 = info[2]
        id2 = info[3]
        tmp = allPoints - np.expand_dims(cent, 0).repeat(allPoints.shape[0], axis=0)
        norm = np.linalg.norm(tmp, axis=1)
        # tmp = tmp/np.expand_dims(norm , 1).repeat(3, axis=1)
        # angle = np.arccos(np.abs(np.dot(tmp, dire.T)))*180/np.pi
        r_dist = np.sqrt(norm*norm - np.abs(np.dot(tmp, dire.T))*np.abs(np.dot(tmp, dire.T)))
        indices = np.argwhere(r_dist < eps).flatten()
        tmp_points = allPoints[indices]
        y_pred = DBSCAN(eps=dist_eps).fit_predict(tmp_points)
        y_uniq = np.unique(np.array(y_pred))
        # tmp_clst = []
        center_list = []
        for y in y_uniq:
            idx = np.argwhere(y_pred == y).flatten()
            ps = np.array(tmp_points[idx])
            center_list.append(np.mean(ps, axis=0))
            # tmp_clst.append(ps)
            # pcd = o3d.geometry.PointCloud()
            # pcd.points = o3d.utility.Vector3dVector(ps)
            # matched_pcds.append(pcd)
        center_list = np.array(center_list)
        ori_cent = np.expand_dims(cent, 0).repeat(center_list.shape[0], axis=0)
        dif_cent = center_list - ori_cent
        com_cent = np.linalg.norm(dif_cent, axis=1)
        index = np.argmin(com_cent).flatten()[0]
        length1 = 1000 
        length2 = 1000
        tmp_length1 = 0
        tmp_length2 = 0
        if dif_cent.shape[0] <= 1:
            length1 = 0 
            length2 = 0
        for i in range(dif_cent.shape[0]):
            if i != index:
                pn = np.dot(dif_cent[i], dire)
                if pn > 0:
                    tmp_length1 = com_cent[i]
                elif pn < 0:
                    tmp_length2 = com_cent[i]
                if length1 > tmp_length1 or length1 == 0:
                    length1 = tmp_length1
                if length2 > tmp_length2 or length2 == 0:
                    length2 = tmp_length2
        rf_path_info.append([dire, cent, id1, id2, length1, length2])
    # visualization.points_visualization_by_vtk(matched_pcds, screw_setting.color)
    return rf_path_info


def estimate_dist(info1, info2):
    dire1 = info1[0]
    cent1 = info1[1]
    f_length1 = info1[4]
    b_length1 = info1[5]
    f_p1 = cent1 + dire1*f_length1
    b_p1 = cent1 - dire1*b_length1
   
    dire2 = info2[0]
    cent2 = info2[1]
    f_length2 = info2[4]
    b_length2 = info2[5]
    f_p2 = cent2 + dire2*f_length2
    b_p2 = cent2 - dire2*b_length2
    return geometry.segment_3d_dist(f_p1, b_p1, f_p2, b_p2)


def interference(new_info, path_info, eps=3*screw_setting.screw_radius):
    for info in path_info:
        dist = estimate_dist(new_info, info)
        if dist <= eps:
            return True
    return False


def get_optimal_info(path_info, pcds, eps=screw_setting.screw_radius, dist_eps=screw_setting.dist_eps):
    rf_path_info = []
    allPoints = np.empty((1, 3))
    all_points = []
    for pcd in pcds:
        points = np.asarray(pcd.points)
        allPoints = np.concatenate([allPoints, points], axis=0)
        all_points.append(points)
    # tree = spatial.KDTree(allPoints)
    # matched_pcds = []
    allCenter = np.mean(allPoints, axis=0)
    print("\n\n\033[31mPath program: there are %d screws needed to be processed.\033[0m" % len(path_info))
    for i in range(len(path_info)):
        info = path_info[i]
        dire = info[0]
        cent = info[1]
        id1 = info[2]
        id2 = info[3]
        path_points = np.concatenate([all_points[id1], all_points[id2]], axis=0)
        cone = get_cone(dire)
        best_dir = dire
        best_length1 = info[4]
        best_length2 = info[5]
        # tmp = allPoints - np.expand_dims(cent, 0).repeat(allPoints.shape[0], axis=0)
        tmp = path_points - np.expand_dims(cent, 0).repeat(path_points.shape[0], axis=0)
        norm = np.linalg.norm(tmp, axis=1)
        for j in tqdm(range(len(cone)), desc="\033[31mThe %dth screw:\033[0m" % (i + 1),):
            n_dir = cone[j]
            r_dist = np.sqrt(norm**2 - np.abs(np.dot(tmp, n_dir.T))**2)
            indices = np.argwhere(r_dist < eps).flatten()
            tmp_points = path_points[indices]
            y_pred = DBSCAN(eps=dist_eps).fit_predict(tmp_points)
            y_uniq = np.unique(np.array(y_pred))
            center_list = []
            fp_list = []
            cp_list = []
            for y in y_uniq:
                idx = np.argwhere(y_pred == y).flatten()
                ps = np.array(tmp_points[idx])
                # if ps.shape[0] < 200:
                #     continue
                dist = np.dot(ps, n_dir.T)
                center_list.append(np.mean(ps, axis=0))
                cp_list.append(ps[np.argmin(dist).flatten()[0]])
                fp_list.append(ps[np.argmax(dist).flatten()[0]])
            center_list = np.array(center_list)
            fp_list = np.array(fp_list)
            cp_list = np.array(cp_list)
            ori_cent = np.expand_dims(cent, 0).repeat(center_list.shape[0], axis=0)
            dif_cent = center_list - ori_cent
            dir_fp = fp_list - ori_cent
            dir_cp = cp_list - ori_cent
            com_cent = np.linalg.norm(dif_cent, axis=1)
            com_fp = np.linalg.norm(dir_fp, axis=1)
            com_cp = np.linalg.norm(dir_cp, axis=1)
            # index = np.argmin(com_cent).flatten()[0]
            length1 = 1000
            length2 = 1000
            tmp_length1 = 0
            tmp_length2 = 0
            idx1 = None
            idx2 = None
            if dif_cent.shape[0] <= 1:
                length1 = 0
                length2 = 0
            for j in range(dif_cent.shape[0]):
                if com_fp[j] < com_cp[j]:
                    tmp_com = com_cp[j]
                    com_cp[j] = com_fp[j]
                    com_fp[j] = tmp_com
                if com_cent[j] > dist_eps:
                    pn = np.dot(dif_cent[j], n_dir)
                    if pn > 0:
                        tmp_length1 = com_cp[j]
                    elif pn < 0:
                        tmp_length2 = com_fp[j]
                    if length1 > tmp_length1 or length1 == 0:
                        length1 = tmp_length1
                        idx1 = j
                    if length2 > tmp_length2 or length2 == 0:
                        length2 = tmp_length2
                        idx2 = j
            min_bl = min(best_length1, best_length2)
            max_bl = max(best_length1, best_length2)
            min_cl = min(com_cp[idx1], com_cp[idx2])
            # min_fl = min(com_fp[idx1], com_fp[idx2])
            # max_cl = max(com_cp[idx1], com_cp[idx2])
            max_fl = max(com_fp[idx1], com_fp[idx2])
            if min_cl > min_bl:
                # if np.linalg.norm(cent + n_dir*com_cp[idx1] - allCenter) > np.linalg.norm(cent - n_dir*com_fp[idx2] - allCenter):
                if com_cp[idx1] < com_cp[idx2]:
                    n_dir = - n_dir
                    length1 = com_cp[idx2]
                    length2 = com_fp[idx1]
                path_info_tmp = [path_info[k] for k in range(len(path_info))]
                del path_info_tmp[i]
                if np.abs(length1) + np.abs(length2) <= 30 or interference([n_dir, cent, id1, id2, length1, length2], path_info_tmp):
                    continue
                best_length1 = length1
                best_length2 = length2
                best_dir = n_dir
            elif max_fl > 2*max_bl and min_cl > 0.6*min_bl and min_cl > dist_eps:
                # if np.linalg.norm(cent + n_dir*com_cp[idx1] - allCenter) > np.linalg.norm(cent - n_dir*com_fp[idx2] - allCenter):
                if com_fp[idx1] > com_fp[idx2]:
                    n_dir = - n_dir
                    length1 = com_cp[idx2]
                    length2 = com_fp[idx1]
                path_info_tmp = [path_info[k] for k in range(len(path_info))]
                del path_info_tmp[i]
                if np.abs(length1) + np.abs(length2) <= 30 or interference([n_dir, cent, id1, id2, length1, length2], path_info_tmp):
                    continue
                best_length1 = length1 - 3
                best_length2 = length2
                best_dir = n_dir
        rf_path_info.append([best_dir, cent, id1, id2, best_length1, best_length2])
    # visualization.points_visualization_by_vtk(matched_pcds, screw_setting.color)
    return rf_path_info


def path_program(frac_pcds, all_pcds):
    refine_cluster = get_effect_points(frac_pcds)
    path_info = []
    all_points = []
    for pcd in frac_pcds:
        all_points.append(np.asarray(pcd.points))
    for i in range(len(refine_cluster)):
        for points in refine_cluster[i]:
            points1 = points[0]
            points2 = points[1]
            # path_dir = get_screw_dir_by_SVM(points1, points2)
            # path_dir = get_screw_dir_by_norm2(points1, points2) # there exist some problems
            path_center = get_screw_implant_position_Chebyshev_center(points1, points2)
            path_dir = get_screw_dir_by_ransac(points1, points2)
            tmp_p1 = points1[1, :]
            tmp_p2 = points2[1, :]
            id1 = None
            id2 = None
            for j in range(len(all_points)):
                tmp_points = all_points[j]
                t1 = np.sum(np.abs(tmp_points - np.expand_dims(tmp_p1, 0).repeat(tmp_points.shape[0], axis=0)), axis=1)
                t2 = np.sum(np.abs(tmp_points - np.expand_dims(tmp_p2, 0).repeat(tmp_points.shape[0], axis=0)), axis=1)
                if np.where(t1 == 0)[0].shape[0] != 0:
                    id1 = j
                elif np.where(t2 == 0)[0].shape[0] != 0:
                    id2 = j             
            path_info.append([path_dir, path_center, id1, id2])
    path_info = add_screw_length(path_info, all_pcds)
    path_info = refine_path_info(path_info, all_pcds)
    return path_info


def refine_path_info(path_info, pcds, radius=screw_setting.path_refine_radius, length_eps=screw_setting.length_eps):
    rf_path_info = []
    all_points = np.empty((1, 3))
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
        vec = np.array(pca.components_[0, :])
        vec = vec/np.linalg.norm(vec)
        if np.dot(vec, direc.T) < 0:
            vec = -vec
        # rf_direc = (direc + vec)/2
        rf_direc = vec # + path_info[i][0]
        # rf_direc = rf_direc/np.linalg.norm(rf_direc)
        rf_path_info.append([rf_direc, point, id1, id2])
    rf_path_info = add_screw_length(rf_path_info, pcds)
    for i in range(len(path_info)):
        length = path_info[i][4] + path_info[i][5]
        rf_length = rf_path_info[i][4] + rf_path_info[i][5]
        if length > length_eps or rf_length < 1.6*length:
            rf_path_info[i][0] = path_info[i][0]
            rf_path_info[i][4] = path_info[i][4]
            rf_path_info[i][5] = path_info[i][5]
    return rf_path_info


def refine_path_info_v1(path_info, pcds, radius=screw_setting.path_refine_radius):
    rf_path_info = []
    all_points = np.empty((1, 3))
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
        vec = vec/np.linalg.norm(vec)
        if np.dot(vec, direc.T) < 0:
            vec = -vec
        # rf_direc = (direc + vec)/2
        rf_direc = vec
        rf_path_info.append([rf_direc, point, id1, id2])
    rf_path_info = add_screw_length(rf_path_info, pcds)
    return rf_path_info


def refine_path_info_v2(path_info, pcds, radius=screw_setting.path_refine_radius):
    rf_path_info = []
    all_points = np.empty((1, 3))
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
        vec1 = np.array(pca.components_[0, :])
        vec2 = np.array(pca.components_[1, :])
        vec3 = np.array(pca.components_[2, :])
        dsp_dsbt1 = np.dot(points, vec1)
        dsp_dsbt2 = np.dot(points, vec2)
        dsp_dsbt3 = np.dot(points, vec3)
        ctbt1 = dsp_dsbt1[np.argmax(dsp_dsbt1)] - dsp_dsbt1[np.argmin(dsp_dsbt1)]
        ctbt2 = dsp_dsbt2[np.argmax(dsp_dsbt2)] - dsp_dsbt2[np.argmin(dsp_dsbt2)]
        ctbt3 = dsp_dsbt3[np.argmax(dsp_dsbt3)] - dsp_dsbt3[np.argmin(dsp_dsbt3)]
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


def relu_refine_dir_v2(ctbt1, ctbt2, ctbt3, direc1, direc2, radius=screw_setting.screw_radius, eps1=screw_setting.rrd_eps_max, eps2=screw_setting.rrd_eps_min):
    ctbt_rate1 = ctbt2/ctbt1
    ctbt_rate2 = ctbt3/ctbt1
    if ctbt3 < 2.5*radius:
        return None
    elif ctbt3 < 4*radius or ctbt_rate1 < eps1:
        return direc2
    elif ctbt3 > 8*radius or ctbt_rate2 > eps2:
        return direc1
    else:
        return (direc1*ctbt_rate2 + direc2*(1 - ctbt_rate2))


def refine_path_info_v3(path_info, pcds, radius1=screw_setting.path_refine_radius, radius2=screw_setting.screw_radius):
    rf_path_info = []
    all_points = []
    allPoints = np.empty((1, 3))
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
        ctbt11 = dsp_dsbt11[np.argmax(dsp_dsbt11)] - dsp_dsbt11[np.argmin(dsp_dsbt11)]
        ctbt21 = dsp_dsbt21[np.argmax(dsp_dsbt21)] - dsp_dsbt21[np.argmin(dsp_dsbt21)]
        ctbt31 = dsp_dsbt31[np.argmax(dsp_dsbt31)] - dsp_dsbt31[np.argmin(dsp_dsbt31)]
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
        ctbt12 = dsp_dsbt12[np.argmax(dsp_dsbt12)] - dsp_dsbt12[np.argmin(dsp_dsbt12)]
        ctbt22 = dsp_dsbt22[np.argmax(dsp_dsbt22)] - dsp_dsbt22[np.argmin(dsp_dsbt22)]
        ctbt32 = dsp_dsbt32[np.argmax(dsp_dsbt32)] - dsp_dsbt32[np.argmin(dsp_dsbt32)]
        # ctbt12 = pca2.explained_variance_ratio_[0]
        # ctbt22 = pca2.explained_variance_ratio_[1]
        # ctbt32 = pca2.explained_variance_ratio_[2]
        if np.dot(vec11, direc.T) < 0:
            vec11 = -vec11
        if np.dot(vec12, direc.T) < 0:
            vec12 = -vec12
        rf_direc = relu_refine_dir_v3(ctbt11, ctbt21, ctbt31, ctbt12, ctbt22, ctbt32, vec, vec11, vec12, direc)
        # rf_direc = vec
        if rf_direc is not None:
            rf_path_info.append([rf_direc, point, id1, id2])
    rf_path_info = add_screw_length(rf_path_info, pcds)
    return rf_path_info


def relu_refine_dir_v3(ctbt11, ctbt21, ctbt31, ctbt12, ctbt22, ctbt32, vec, vec11, vec12, direc, radius=screw_setting.screw_radius, eps1=screw_setting.rrd_eps_max, eps2=screw_setting.rrd_eps_min):
    ctbt_rate11 = ctbt21/ctbt11
    ctbt_rate21 = ctbt31/ctbt11
    ctbt_rate12 = ctbt22/ctbt12
    ctbt_rate22 = ctbt32/ctbt12
    if ctbt31 < 3*radius or ctbt32 < 3*radius:
        return None
    elif ctbt31 < 4*radius or ctbt_rate11 < eps1 or ctbt32 < 4*radius or ctbt_rate12 < eps1:
        if ctbt21 < 0.8*ctbt22:
            return vec11
        elif ctbt21 < 0.8*ctbt22:
            return vec12
        else:
            return vec
    elif ctbt31 > 8*radius and ctbt_rate21 > eps2 and ctbt32 > 8*radius and ctbt_rate22 > eps2:
        return direc
    else:
        return (vec12/ctbt_rate22 + vec11/ctbt_rate21 + direc)/np.linalg.norm(vec12/ctbt_rate22 + vec11/ctbt_rate21 + direc)
    
    
def refine_path_info_v4(path_info, pcds):
    return get_optimal_info(path_info, pcds)
    
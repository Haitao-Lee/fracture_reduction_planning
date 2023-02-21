# coding = utf-8
# import open3d as o3d
# import vtkmodules.all as vtk
import numpy as np
import math
import scipy.spatial as spatial
import open3d as o3d
import matplotlib.pyplot as plt
from tqdm import tqdm
from collections import Counter
import fit_plane as fp
import vtkmodules.all as vtk


def compute_SD_value(neighbourhood, points, effect_points, effect_normals,
                     p_idx):
    p_i = effect_points[p_idx]
    n_p_i = effect_normals[p_idx]
    p_i_bar = np.mean(points[neighbourhood], axis=0)
    v = p_i - p_i_bar
    # print('\n')
    # print(points[neighbourhood])
    # print('\n')
    # print(p_i)
    # print(p_i_bar)
    # print(np.linalg.norm(v))
    SD = np.dot(v, n_p_i)
    return SD


# get feature points
def get_feature_pcd(ref_pcd, tar_pcd, effect_rate, sd_rate, r):
    ref_points = np.asarray(ref_pcd.points)
    ref_normals = np.asarray(ref_pcd.normals)
    tar_points = np.asarray(tar_pcd.points)
    tar_normals = np.asarray(tar_pcd.normals)
    ref_center = np.mean(ref_points, axis=0)
    tar_center = np.mean(tar_points, axis=0)
    ref_kd_tree = spatial.KDTree(ref_points)
    tar_kd_tree = spatial.KDTree(tar_points)
    k = min(ref_points.shape[0] * effect_rate,
            tar_points.shape[0] * effect_rate)
    _, ref_effect_points_index = ref_kd_tree.query(tar_center, k, workers=-1)
    ref_effect_points_index.astype(int)
    # print(ref_effect_points_index)
    _, tar_effect_points_index = tar_kd_tree.query(ref_center, k, workers=-1)
    tar_effect_points_index.astype(int)
    # print(tar_effect_points_index)
    ref_effect_points = ref_points[ref_effect_points_index]
    # print(ref_effect_points)
    ref_effect_normals = ref_normals[ref_effect_points_index]
    ref_effect_pcd = o3d.geometry.PointCloud()
    ref_effect_pcd.points = o3d.utility.Vector3dVector(ref_effect_points)
    ref_effect_pcd.normals = o3d.utility.Vector3dVector(ref_effect_normals)
    tar_effect_points = tar_points[tar_effect_points_index]
    tar_effect_normals = tar_normals[tar_effect_points_index]
    tar_effect_pcd = o3d.geometry.PointCloud()
    tar_effect_pcd.points = o3d.utility.Vector3dVector(tar_effect_points)
    tar_effect_pcd.normals = o3d.utility.Vector3dVector(tar_effect_normals)
    ###################################################################################
    ref_SD = []
    tar_SD = []
    print("--Extracting SD values:")
    for i in tqdm(range(0, ref_effect_points.shape[0])):
        ref_nbhd = ref_kd_tree.query_ball_point(ref_effect_points[i],
                                                r,
                                                workers=-1)
        ref_SD.append(
            compute_SD_value(ref_nbhd, ref_points, ref_effect_points,
                             ref_effect_normals, i))
        tar_nbhd = tar_kd_tree.query_ball_point(tar_effect_points[i],
                                                r,
                                                workers=-1)
        tar_SD.append(
            compute_SD_value(tar_nbhd, tar_points, tar_effect_points,
                             tar_effect_normals, i))
    ref_SD = np.abs(np.asarray(ref_SD))
    tar_SD = np.abs(np.asarray(tar_SD))
    ref_SD[ref_SD <= 1e-8] = 1e-8
    tar_SD[tar_SD <= 1e-8] = 1e-8
    ref_SD_threshold = sd_rate * np.mean(ref_SD)
    tar_SD_threshold = sd_rate * np.mean(tar_SD)
    #########################################################################
    # plt.figure()
    # plt.subplot(1, 2, 1)
    # ref_fig = np.sort(ref_SD)
    # ref_index = range(ref_fig.shape[0])
    # plt.bar(ref_index, ref_fig, label="ref_sd", color="#87CEFA")
    # plt.hlines(ref_SD_threshold, 0, ref_fig.shape[0], color="red")
    # plt.xlabel("index")
    # plt.ylabel("value")
    # plt.title("ref_sd value distribution")

    # plt.subplot(1, 2, 2)
    # tar_fig = np.sort(tar_SD)
    # tar_index = range(tar_fig.shape[0])
    # plt.bar(tar_index, tar_fig, label="tar_sd", color="#87CEFA")
    # plt.hlines(tar_SD_threshold, 0, tar_fig.shape[0], color="red")
    # plt.xlabel("index")
    # plt.ylabel("value")
    # plt.title("tar_sd value distribution")

    # plt.show()
    # print(ref_fig)
    # print(tar_fig)
    #########################################################################
    ref_feature_points = ref_effect_points[np.argwhere(
        ref_SD > ref_SD_threshold)[:, 0]]
    ref_feature_normals = ref_effect_normals[np.argwhere(
        ref_SD > ref_SD_threshold)[:, 0]]
    tar_feature_points = tar_effect_points[np.argwhere(
        tar_SD > tar_SD_threshold)[:, 0]]
    tar_feature_normals = tar_effect_normals[np.argwhere(
        tar_SD > tar_SD_threshold)[:, 0]]
    # print(ref_feature_points.shape)
    # print(tar_feature_points.shape)
    ref_feature_pcd = o3d.geometry.PointCloud()
    ref_feature_pcd.points = o3d.utility.Vector3dVector(ref_feature_points)
    ref_feature_pcd.normals = o3d.utility.Vector3dVector(ref_feature_normals)
    tar_feature_pcd = o3d.geometry.PointCloud()
    tar_feature_pcd.points = o3d.utility.Vector3dVector(tar_feature_points)
    tar_feature_pcd.normals = o3d.utility.Vector3dVector(tar_feature_normals)
    return ref_feature_pcd, tar_feature_pcd, ref_effect_pcd, tar_effect_pcd, ref_center, tar_center


# refine matching points via minimum spanning tree
def refine_feature_points(feature_pcd, pcd, dist_threhold, num_threshold):
    points = np.asarray(feature_pcd.points)
    normals = np.asarray(feature_pcd.normals)
    all_points = np.asarray(pcd.points)
    all_normals = np.asarray(pcd.normals)
    # repair the surface inside feature points
    #############################################################################
    tree = spatial.KDTree(all_points)
    indices = tree.query_ball_point(points, dist_threhold, workers=-1)
    indices = [j for i in indices for j in i]
    indices = np.asarray(indices).flatten()
    effect_indices = getRepeatedElements(indices, num_threshold)
    # points = np.concatenate((points, all_points[effect_indices]), axis=0)
    # normals = np.concatenate((normals, all_normals[effect_indices]), axis=0)
    #############################################################################
    # points_tmp = np.asarray(clear_repeated_element(points.tolist()))
    # normals_tmp = np.asarray(clear_repeated_element(normals.tolist()))
    #############################################################################
    # tree = spatial.KDTree(points)
    # indices = tree.query_ball_point(all_points, dist_threhold, workers=-1)
    # for i in tqdm(range(0, all_points.shape[0])):
    #     point_center = np.mean(points[indices[i]], axis=0)
    #     print(indices[i])
    #     print(point_center)
    #     if np.linalg.norm(all_points[i]-point_center) < 1/3*dist_threhold:
    #         points_tmp = np.concatenate((points_tmp, all_points[i]), axis=0)
    #         normals_tmp = np.concatenate((normals_tmp, all_normals[i]), axis=0)
    #############################################################################
    tmp_pcd = o3d.geometry.PointCloud()
    tmp_pcd.points = o3d.utility.Vector3dVector(all_points[effect_indices])
    tmp_pcd.normals = o3d.utility.Vector3dVector(all_normals[effect_indices])
    # tmp_pcd, _ = feature_pcd.remove_radius_outlier(nb_points=num_threshold, radius=dist_threhold)
    # ###################################################################################
    # point_clusters = []
    # normal_clusters = []
    # while points_tmp.shape[0]:
    #     point_cluster, normal_cluster = cluster(points_tmp[0], tmp_pcd, dist_threhold)
    #     print("--Forming clusters:")
    #     for point in tqdm(point_cluster):
    #         index = np.asarray(np.where((points_tmp == point).all(axis=1)))
    #         if index.shape[0] >= 1:
    #             points_tmp = np.delete(points_tmp, index, axis=0)
    #             normals_tmp = np.delete(normals_tmp, index, axis=0)
    #     tmp_pcd.points = o3d.utility.Vector3dVector(points_tmp)
    #     tmp_pcd.normals = o3d.utility.Vector3dVector(normals_tmp)
    #     point_clusters.append(point_cluster)
    #     normal_clusters.append(normal_cluster)
    # ###################################################################################
    # feature_pcds = []
    # print("--Clustering:")
    # for i in tqdm(range(0, len(point_clusters))):
    #     if len(point_clusters[i]) <= num_threshold:
    #         continue
    #     cluster_points = np.asarray(point_clusters[i])
    #     cluster_normals = np.asarray(normal_clusters[i])
    #     feature_pcd_tmp = o3d.geometry.PointCloud()
    #     feature_pcd_tmp.points = o3d.utility.Vector3dVector(cluster_points)
    #     feature_pcd_tmp.normals = o3d.utility.Vector3dVector(cluster_normals)
    #     feature_pcds.append(feature_pcd_tmp)
    return tmp_pcd


# cluster
def cluster(seed_point, pcd, threshold):
    points = np.asarray(pcd.points)
    normals = np.asarray(pcd.normals)
    tree = spatial.KDTree(points)
    point_cluster = []
    normal_cluster = []
    point_cluster.append(seed_point)
    normal_cluster.append(seed_point)
    index = tree.query_ball_point(seed_point, threshold, workers=-1)
    index = np.asarray(index)
    if index.shape[0] >= 1:
        point_cluster.extend(points[index])
        normal_cluster.extend(normals[index])
        points = np.delete(points, index, axis=0)
        normals = np.delete(normals, index, axis=0)
        tmp_pcd = o3d.geometry.PointCloud()
        tmp_pcd.points = o3d.utility.Vector3dVector(points)
        tmp_pcd.normals = o3d.utility.Vector3dVector(normals)
        num_point = len(point_cluster)
        for i in range(1, num_point):
            new_point_cluster, new_normal_cluster = cluster(
                point_cluster[i], tmp_pcd, threshold)
            point_cluster.extend(new_point_cluster)
            normal_cluster.extend(new_normal_cluster)
            for new_point in new_point_cluster:
                # print(np.where((points == points[0]).all(axis=1)))
                new_index = np.where((points == new_point).all(axis=1))[0]
                # print(new_point)
                # print(new_index)
                # print(len(new_index))
                if len(new_index) >= 1:
                    points = np.delete(points, new_index, axis=0)
                    # print(points)
                    normals = np.delete(normals, new_index, axis=0)
            tmp_pcd.points = o3d.utility.Vector3dVector(points)
            tmp_pcd.normals = o3d.utility.Vector3dVector(normals)
        # point_cluster = list(set(point_cluster))
        # normal_cluster = list(set(normal_cluster))
    return point_cluster, normal_cluster,


# get repeated elements
def getRepeatedElements(array, num):
    array = array.flatten()
    array_dic = Counter(array)
    result = []
    for key, value in array_dic.items():
        if value >= num:
            result.append(key)
    result = np.asarray(result)
    result = np.unique(result)
    return result


def clear_repeated_element(a):
    b = []
    for c in a:
        if c not in b:
            b.append(c)
    return b


# compute distance between 2 polyData
def compute_distance(ref_ply, tar_ply):
    ref_center = ref_ply.GetCenter()
    tar_center = tar_ply.GetCenter()
    distance = 0
    for i in range(0, len(ref_center)):
        distance += pow(ref_center[i] - tar_center[i], 2)
    distance = math.sqrt(distance)
    return distance


def self_cluster_by_gravity(pcd, dir_vect, radius):
    points = np.asarray(pcd.points)
    normals = np.asarray(pcd.normals)
    plane = fp.fit_plane_by_norm2(points, dir_vect)
    plane_center = plane[0]
    plane_normal = plane[1]
    # print(plane_normal)
    plane_p1 = plane[3]
    plane_p2 = plane[4]
    ###################################################################################
    renderer = vtk.vtkRenderer()
    vtk_points = vtk.vtkPoints()
    vtk_cells = vtk.vtkCellArray()
    xyz = np.asarray(points)
    for j in range(0, xyz.shape[0]):
        vtk_cells.InsertNextCell(1)
        vtk_cells.InsertCellPoint(
            vtk_points.InsertNextPoint(xyz[j][0], xyz[j][1], xyz[j][2]))
    ply = vtk.vtkPolyData()
    ply.SetPoints(vtk_points)
    ply.SetVerts(vtk_cells)

    ply_mapper = vtk.vtkPolyDataMapper()
    ply_mapper.SetInputData(ply)
    ply_mapper.Update()

    ply_actor = vtk.vtkActor()
    ply_actor.SetMapper(ply_mapper)
    ply_actor.GetProperty().SetColor(1, 0, 0)
    renderer.AddActor(ply_actor)

    planeSource = vtk.vtkPlaneSource()
    planeSource.SetCenter(plane_center)
    # planeSource.SetNormal(plane_normal)
    planeSource.SetPoint1(plane_p1)
    planeSource.SetPoint2(plane_p2)
    planeSource.Update()

    polydataPlane = vtk.vtkPolyData()
    polydataPlane = planeSource.GetOutput()
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputData(polydataPlane)

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(0, 1, 0)
    renderer.AddActor(actor)

    # sphere = vtk.vtkSphereSource()
    # sphere.SetCenter(plane_center)
    # sphere.SetRadius(radius)

    # sp_mapper = vtk.vtkPolyDataMapper()
    # sp_mapper.SetInputConnection(sphere.GetOutputPort())
    # sp_mapper.Update()

    # sp_actor = vtk.vtkActor()
    # sp_actor.SetMapper(sp_mapper)
    # sp_actor.GetProperty().SetColor(1, 1, 1)
    # renderer.AddActor(sp_actor)

    render_window = vtk.vtkRenderWindow()
    render_window.AddRenderer(renderer)
    rw_style = vtk.vtkInteractorStyleTrackballCamera()
    rw_interactor = vtk.vtkRenderWindowInteractor()
    rw_interactor.SetRenderWindow(render_window)
    rw_interactor.SetInteractorStyle(rw_style)
    render_window.Render()
    rw_interactor.Initialize()
    rw_interactor.Start()
    ###################################################################################
    tree = spatial.KDTree(points)
    points_indices = np.asarray(
        tree.query_ball_point(points, radius, workers=-1))
    # _, points_indices = tree.query(points, 6, workers=-1)
    # points_indices = np.asarray(points_indices)
    # print(points_index)
    # print("\n")
    # points_index.astype(int)
    # point_clusters = []
    # normal_clusters = []
    indices_clusters = []
    valley_cluster = []
    print("--cluster by gravity:")
    for i in tqdm(range(0, points.shape[0])):
        point = np.asarray(points[i])
        # normal = np.asarray(normals[i])
        # print((point*np.ones([numOfPoints, 3])).shape)
        # print(points[points_index[i]].shape)
        points_indices[i] = np.asarray(points_indices[i])
        # print(points_indices[i])
        # print(point * np.ones([points_indices[i].shape[0], 3]))
        # print("\n")
        # print(points[points_indices[i]])
        gravity_array = np.asarray(
            point * np.ones([points_indices[i].shape[0], 3])) - np.asarray(
                points[points_indices[i]])
        # gravity_weight = np.linalg.norm(gravity_array, axis = 1)
        # gravity_array = gravity_array.T/gravity_weight
        gravity_field = np.asarray(np.dot(plane_normal, gravity_array.T))
        j = np.argmax(gravity_field)
        if gravity_field[j] <= 1e-5:
            valley_cluster.append(point)
            cluster_flag = True
            for k in range(0, len(indices_clusters)):
                # tmp = get_list_element_index(point_clusters[k], new_point)
                if i in indices_clusters[k]:
                    cluster_flag = False
                    break
                # point_clusters[k] = clear_repeated_element(point_clusters[k])
            if cluster_flag:
                # point_cluster_tmp = []
                # normal_cluster_tmp = []
                # point_cluster_tmp.append(point)
                # normal_cluster_tmp.append(normal)
                indices_tmp = []
                indices_tmp.append(i)
                # point_clusters.append(point_cluster_tmp)
                # normal_clusters.append(normal_cluster_tmp)
                indices_clusters.append(indices_tmp)
        else:
            # new_point = points[points_indices[i][j]]
            cluster_index1 = -1
            cluster_index2 = -1
            for k in range(0, len(indices_clusters)):
                # tmp = get_list_element_index(point_clusters[k], new_point)
                if cluster_index1 == -1:
                    if i in indices_clusters[k]:
                        cluster_index1 = k
                        if cluster_index2 != -1:
                            break
                if cluster_index2 == -1:
                    if points_indices[i][j] in indices_clusters[k]:
                        cluster_index2 = k
                        if cluster_index1 != -1:
                            break
                # point_clusters[k] = clear_repeated_element(point_clusters[k])
            if cluster_index1 == cluster_index2 and cluster_index1 == -1:
                # point_cluster_tmp = []
                # point_cluster_tmp.append(point)
                # point_cluster_tmp.append(new_point)
                indices_tmp = []
                indices_tmp.append(i)
                indices_tmp.append(points_indices[i][j])
                # point_clusters.append(point_cluster_tmp)
                indices_clusters.append(indices_tmp)
            elif cluster_index1 == -1:
                # point_clusters[cluster_index2].append(point)
                indices_clusters[cluster_index2].append(i)
            elif cluster_index2 == -1:
                # point_clusters[cluster_index1].append(new_point)
                indices_clusters[cluster_index1].append(points_indices[i][j])
            elif cluster_index1 != cluster_index2:
                # point_cluster_tmp = point_clusters[cluster_index2]
                indices_tmp = indices_clusters[cluster_index2]
                # point_clusters[cluster_index1].extend(point_cluster_tmp)
                indices_clusters[cluster_index1].extend(indices_tmp)
                # del point_clusters[cluster_index2]
                del indices_clusters[cluster_index2]
                # print(gravity_field)

    pcd_clusters = []
    for index_cluster in indices_clusters:
        point_cluster = np.asarray(points[index_cluster])
        normal_cluster = np.asarray(normals[index_cluster])
        tmp_pcd = o3d.geometry.PointCloud()
        tmp_pcd.points = o3d.utility.Vector3dVector(point_cluster)
        tmp_pcd.normals = o3d.utility.Vector3dVector(normal_cluster)
        pcd_clusters.append(tmp_pcd)
    return pcd_clusters, valley_cluster, indices_clusters


def cluster_by_gravity(pcd, plane_normal, radius):
    points = np.asarray(pcd.points)
    normals = np.asarray(pcd.normals)
    tree = spatial.KDTree(points)
    points_indices = np.asarray(tree.query_ball_point(points, radius, workers=-1))
    # _, points_indices = tree.query(points, 6, workers=-1)
    # points_indices = np.asarray(points_indices)
    # print(points_index)
    # print("\n")
    # points_index.astype(int)
    # point_clusters = []
    # normal_clusters = []
    indices_clusters = []
    valley_cluster = []
    print("--cluster by gravity:")
    for i in tqdm(range(0, points.shape[0])):
        point = np.asarray(points[i])
        # normal = np.asarray(normals[i])
        # print((point*np.ones([numOfPoints, 3])).shape)
        # print(points[points_index[i]].shape)
        points_indices[i] = np.asarray(points_indices[i])
        # print(points_indices[i])
        # print(point * np.ones([points_indices[i].shape[0], 3]))
        # print("\n")
        # print(points[points_indices[i]])
        gravity_array = np.asarray(
            point * np.ones([points_indices[i].shape[0], 3])) - np.asarray(
                points[points_indices[i]])
        # gravity_weight = np.linalg.norm(gravity_array, axis = 1)
        # gravity_array = gravity_array.T/gravity_weight
        gravity_field = np.asarray(np.dot(plane_normal, gravity_array.T))
        j = np.argmax(gravity_field)
        if gravity_field[j] <= 1e-5:
            valley_cluster.append(point)
            cluster_flag = True
            for k in range(0, len(indices_clusters)):
                # tmp = get_list_element_index(point_clusters[k], new_point)
                if i in indices_clusters[k]:
                    cluster_flag = False
                    break
                # point_clusters[k] = clear_repeated_element(point_clusters[k])
            if cluster_flag:
                # point_cluster_tmp = []
                # normal_cluster_tmp = []
                # point_cluster_tmp.append(point)
                # normal_cluster_tmp.append(normal)
                indices_tmp = []
                indices_tmp.append(i)
                # point_clusters.append(point_cluster_tmp)
                # normal_clusters.append(normal_cluster_tmp)
                indices_clusters.append(indices_tmp)
        else:
            # new_point = points[points_indices[i][j]]
            cluster_index1 = -1
            cluster_index2 = -1
            for k in range(0, len(indices_clusters)):
                # tmp = get_list_element_index(point_clusters[k], new_point)
                if cluster_index1 == -1:
                    if i in indices_clusters[k]:
                        cluster_index1 = k
                        if cluster_index2 != -1:
                            break
                if cluster_index2 == -1:
                    if points_indices[i][j] in indices_clusters[k]:
                        cluster_index2 = k
                        if cluster_index1 != -1:
                            break
                # point_clusters[k] = clear_repeated_element(point_clusters[k])
            if cluster_index1 == cluster_index2 and cluster_index1 == -1:
                # point_cluster_tmp = []
                # point_cluster_tmp.append(point)
                # point_cluster_tmp.append(new_point)
                indices_tmp = []
                indices_tmp.append(i)
                indices_tmp.append(points_indices[i][j])
                # point_clusters.append(point_cluster_tmp)
                indices_clusters.append(indices_tmp)
            elif cluster_index1 == -1:
                # point_clusters[cluster_index2].append(point)
                indices_clusters[cluster_index2].append(i)
            elif cluster_index2 == -1:
                # point_clusters[cluster_index1].append(new_point)
                indices_clusters[cluster_index1].append(points_indices[i][j])
            elif cluster_index1 != cluster_index2:
                # point_cluster_tmp = point_clusters[cluster_index2]
                indices_tmp = indices_clusters[cluster_index2]
                # point_clusters[cluster_index1].extend(point_cluster_tmp)
                indices_clusters[cluster_index1].extend(indices_tmp)
                # del point_clusters[cluster_index2]
                del indices_clusters[cluster_index2]
                # print(gravity_field)

    pcd_clusters = []
    for index_cluster in indices_clusters:
        point_cluster = np.asarray(points[index_cluster])
        normal_cluster = np.asarray(normals[index_cluster])
        tmp_pcd = o3d.geometry.PointCloud()
        tmp_pcd.points = o3d.utility.Vector3dVector(point_cluster)
        tmp_pcd.normals = o3d.utility.Vector3dVector(normal_cluster)
        pcd_clusters.append(tmp_pcd)
    return pcd_clusters, valley_cluster, indices_clusters


def ud_cluster_by_gravity(pcd, plane_normal, radius):
    points = np.asarray(pcd.points)
    normals = np.asarray(pcd.normals)
    _, valley_ponts, indices_cluster1 = cluster_by_gravity(pcd, plane_normal, radius)
    _, _, indices_cluster2 = cluster_by_gravity(pcd, -plane_normal, radius)
    indices_cluster = []
    dealed_indices = []
    undealed_indices = []
    for i in tqdm(range(0, len(indices_cluster1))):
        indices1 = indices_cluster1[i]
        for j in range(0, len(indices_cluster2)):
            tmp_idx_clst = []
            indices2 = indices_cluster2[j]
            for index in indices1:
                if index in indices2:
                    indices1.remove(index)
                    indices_cluster2[j].remove(index)
                    tmp_idx_clst.append(index)
                    dealed_indices.append(index)
            if len(tmp_idx_clst):
                indices_cluster.append(tmp_idx_clst)
        undealed_indices.extend(indices1)
    ##########################################################################################
    far_dist = 3000
    dealed_indices = np.asarray(dealed_indices)
    undealed_indices = np.asarray(undealed_indices)
    tmp_points = points.copy()
    points_center = np.mean(points, axis=0)
    tmp_points[undealed_indices] = np.expand_dims(points_center + far_dist, 0).repeat(len(undealed_indices), axis=0)
    tree = spatial.KDTree(tmp_points)
    for index in undealed_indices:
        point = points[index]
        _, idx = tree.query(point, 1, workers=-1)
        for i in range(0, len(indices_cluster)):
            if idx in indices_cluster[i]:
                indices_cluster[i].append(index)
    ##########################################################################################
    pcd_clusters = []
    for indices in indices_cluster:
        indices = np.asarray(indices)
        tmp_pcd = o3d.geometry.PointCloud()
        tmp_pcd.points = o3d.utility.Vector3dVector(points[indices])
        tmp_pcd.normals = o3d.utility.Vector3dVector(normals[indices])
        pcd_clusters.append(tmp_pcd)
    return pcd_clusters, valley_ponts, indices_cluster
            

def get_list_element_index(ob_list, word):
    _ob_arr = np.asarray(ob_list)
    _diff = np.abs(_ob_arr - word[None, :])
    eps = 1e-6
    ret = (_diff < eps).all(axis=1)
    if np.any(ret):
        return [
            np.argmax(ret),
        ]
    else:
        return []
    # return [i for (i, v) in enumerate(ob_list) if v.all() == word.all()]


def get_valley_points_by_gravity(points, dir_vec, radius):
    points = np.asarray(points)
    tree = spatial.KDTree(points)
    points_indices = np.asarray(
        tree.query_ball_point(points, radius, workers=-1))
    # _, points_indices = tree.query(points, 6, workers=-1)
    # points_indices = np.asarray(points_indices)
    # print(points_index)
    # print("\n")
    # points_index.astype(int)
    # point_clusters = []
    # indices_clusters = []
    valley_cluster = []
    # print("--cluster by gravity:")
    for i in range(0, points.shape[0]):
        point = np.asarray(points[i])
        # print((point*np.ones([numOfPoints, 3])).shape)
        # print(points[points_index[i]].shape)
        points_indices[i] = np.asarray(points_indices[i])
        # print(points_indices[i])
        # print(point * np.ones([points_indices[i].shape[0], 3]))
        # print("\n")
        # print(points[points_indices[i]])
        gravity_array = np.asarray(
            point * np.ones([points_indices[i].shape[0], 3])) - np.asarray(
                points[points_indices[i]])
        # gravity_weight = np.linalg.norm(gravity_array, axis = 1)
        # gravity_array = gravity_array.T/gravity_weight
        gravity_field = np.asarray(np.dot(dir_vec, gravity_array.T))
        j = np.argmax(gravity_field)
        if gravity_field[j] <= 1e-5:
            valley_cluster.append(point)
    return valley_cluster


def get_cvx_ccv_points(pcd, radius):
    points = np.asarray(pcd.points)
    normals = np.asarray(pcd.normals)
    tree = spatial.KDTree(points)
    nbhd = tree.query_ball_point(points, radius, workers=-1)
    cvx_ccv_points = []
    print("compute convex and concave points:",)
    for i in tqdm(range(0, points.shape[0])):
        point = points[i]
        normal = normals[i]
        nbhds = points[nbhd[i]]
        vect = np.expand_dims(point, 0).repeat(nbhds.shape[0], axis=0) - nbhds
        vect_norm = np.linalg.norm(vect, axis=1)
        vect = np.delete(vect, np.where(vect_norm == 0), axis=0)
        vect_norm = np.delete(vect_norm, np.where(vect_norm == 0))
        vect = vect/np.c_[vect_norm, vect_norm, vect_norm]
        ret = np.dot(vect, normal)
        abs_ret = np.abs(ret)
        if ret[np.argmin(ret)]*ret[np.argmax(ret)] >= 0 and abs_ret[np.argmax(abs_ret)] > math.sin(math.pi/9):
            cvx_ccv_points.append(point)
    return cvx_ccv_points

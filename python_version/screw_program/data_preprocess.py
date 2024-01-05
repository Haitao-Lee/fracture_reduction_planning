# coding = utf-8
import screw_setting
import numpy as np
import open3d as o3d
from tqdm import tqdm
import scipy.spatial as spatial
import geometry
import visualization


def remove_outliers(pcds, nd=screw_setting.nd, std_rt=screw_setting.std_rt):
    new_pcds = []
    for pcd in pcds:
        n_pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=nd, std_ratio=std_rt)
        new_pcds.append(n_pcd)
    return new_pcds


def downSample(pcds, voxel_size=screw_setting.voxel_size):
    dsp_pcds = []
    for i in range(len(pcds)):
        pcd = pcds[i]
        pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
        dsp_pcds.append(pcd)
    return dsp_pcds


def pcds_normals_outside(pcds):
    all_points = np.empty((0, 3))
    all_normals = np.empty((0, 3))
    allPoints = []
    allNormals = []
    sizes = [0]
    for i in range(len(pcds)):
        pcd = pcds[i]
        pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1, max_nn=30))
        all_points = np.concatenate([all_points, np.array(pcd.points)], axis=0)
        all_normals = np.concatenate([all_normals, np.array(pcd.normals)], axis=0)
        allPoints.append(np.array(pcd.points))
        allNormals.append(np.array(pcd.normals))
        sizes.append(sizes[i] + np.array(pcd.points).shape[0])
    z_axis = np.array([0, 0, 1])
    dsbs = np.dot(all_points, z_axis.T)
    index = np.argmax(dsbs).flatten()[0]
    tmp_n = None
    for i in range(len(sizes)):
        if index == 0:
            if np.dot(all_normals[index], z_axis.T) <= 0:
                allNormals[0][0] = -allNormals[0][0]
            tmp_n = allNormals[0][0].copy()
            break
        elif index < sizes[i]:
            if np.dot(all_normals[index], z_axis.T) <= 0:
                allNormals[i-1][index-sizes[i-1]] = -allNormals[i-1][index-sizes[i-1]] 
            tmp_n = allNormals[i-1][index-sizes[i-1]].copy()
            break
    tmp_points = all_points.copy()
    tmp_p = tmp_points[index].copy()
    tmp_points[index] = np.array([10000, 10000, 10000])
    for i in tqdm(range(all_points.shape[0]), desc="\033[31mNormalizing direction:\033[0m",):
        tmp_norm = np.linalg.norm(tmp_points - np.expand_dims(tmp_p, 0).repeat(tmp_points.shape[0], axis=0), axis=1)
        tmp_idx = np.argmin(tmp_norm).flatten()[0]
        tmp_p = tmp_points[tmp_idx].copy()
        tmp_points[tmp_idx] = np.array([10000, 10000, 10000])
        for i in range(len(sizes)):
            if tmp_idx == 0:
                if np.dot(allNormals[0][0], tmp_n.T) <= 0:
                    allNormals[0][0] = -allNormals[0][0]
                tmp_n = allNormals[0][0].copy()
                break
            elif index < sizes[i]:
                if np.dot(allNormals[i-1][index-sizes[i-1]], tmp_n.T) <= 0:
                    allNormals[i-1][index-sizes[i-1]] = -allNormals[i-1][index-sizes[i-1]] 
                tmp_n = allNormals[i-1][index-sizes[i-1]].copy()
                break
    new_pcds = []
    for i in range(len(allPoints)):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(allPoints[i])
        pcd.normals = o3d.utility.Vector3dVector(allNormals[i])
        new_pcds.append(pcd)
    # visualization.points_visualization_by_vtk(new_pcds)
    return new_pcds

def fromAllPCDs2FracPCDs(all_pcds, dist_threshold=4):
    all_points = []
    frac_pcds = []
    trees = []
    for all_pcd in all_pcds:
        all_points.append(np.array(all_pcd.points))
        trees.append(spatial.KDTree(np.array(all_pcd.points)))
    # finish_indices=[]
    for i in range(len(all_points)):
        # finish_indices.append(i)
        # points1 = np.empty((0, 3))
        # points2 = np.empty((0, 3))
        for j in range(len(all_points)):
            if j != i:
                indices = trees[j].query_ball_point(all_points[i], dist_threshold, workers=-1)
                indices = np.array(indices).flatten()
                eff_idx = []
                for index in indices:
                    if len(index) > 0:
                        eff_idx = eff_idx + index
                eff_idx = np.unique(np.array(eff_idx)).astype(int)
                frac_point1 = all_points[j][eff_idx]
                # _, indices = trees[j].query(all_points[i], 1, workers=-1)
                # indices = np.array(indices)
                # dist = np.linalg.norm(all_points[i] - all_points[j][indices], axis=1)
                # dist_idx = np.argwhere(dist < dist_threshold).flatten()
                # frac_point1 = np.concatenate([points1, all_points[i][dist_idx]], axis=0)
                # frac_point2 = np.concatenate([points2, all_points[j][indices[dist_idx]]], axis=0)    
                if frac_point1.shape[0] > 50:# + frac_point2.shape[0]> 100:
                    pcd = o3d.geometry.PointCloud()
                    pcd.points = o3d.utility.Vector3dVector(frac_point1)
                    frac_pcds.append(pcd)    
                    # pcd = o3d.geometry.PointCloud()
                    # pcd.points = o3d.utility.Vector3dVector(frac_point2)
                    # frac_pcds.append(pcd)     
    return frac_pcds


def get_rest_pcds(all_pcds, frac_pcds, radius=0.5*screw_setting.screw_radius):
    frac_points = np.empty((0,3))
    trees = []
    rest_points = []
    for frac_pcd in frac_pcds:
        frac_points = np.concatenate([frac_points, frac_pcd.points], axis=0)
    for all_pcd in all_pcds:
        trees.append(0)
        rest_points.append(np.array(all_pcd.points))
    indices = np.empty((0, 1))
    for j in range(len(rest_points)):
        trees[j] = spatial.KDTree(np.array(rest_points[j]))
        all_ps = rest_points[j]
        _, tmp_indices = trees[j].query(frac_points, 1, workers=-1)
        tmp_indices = np.array(tmp_indices).flatten()
        dist = np.linalg.norm(frac_points - all_ps[tmp_indices], axis=1)
        indices = np.argwhere(dist < radius).flatten()
        mask = np.ones(rest_points[j].shape, dtype=bool)
        mask[tmp_indices[indices], :] = False
        rest_points[j] = rest_points[j][mask].reshape(-1, 3)
    rest_pcds = []
    for i in range(len(rest_points)):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(rest_points[i])
        rest_pcds.append(pcd)
        # visualization.points_visualization_by_vtk(rest_pcds)
    return rest_pcds

    
        

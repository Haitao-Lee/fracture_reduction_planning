# coding = utf-8
import screw_setting
import numpy as np
import open3d as o3d
from tqdm import tqdm
import scipy.spatial as spatial
import geometry


def remove_outliers(pcds, nd=screw_setting.nd, std_rt=screw_setting.std_rt):
    new_pcds = []
    for pcd in pcds:
        n_pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=nd, std_ratio=std_rt)
        new_pcds.append(n_pcd)
    return new_pcds


def get_rest_pcds(all_pcds, frac_pcds):
    all_points = []
    frac_points = []
    trees = []
    for pcd in frac_pcds:
        frac_points.append(np.asarray(pcd.points))
    for pcd in all_pcds:
        all_points.append(np.array(pcd.points))
        trees.append(spatial.KDTree(np.array(pcd.points)))
    for i in tqdm(range(len(frac_points)), desc="\033[31mGenerating effect point clouds:\033[0m",):
        frac_ps = frac_points[i]
        _, frac_ps, _ = geometry.ransac_planefit(frac_ps, ransac_n=3, max_dst=screw_setting.ransac_eps/10)
        index = None
        min_dist = 1000
        indices = None
        for j in range(len(all_points)):
            all_ps = all_points[j]
            _, tmp_indices = trees[j].query(frac_ps, 1, workers=-1)
            tmp_indices = np.array(tmp_indices).flatten()
            dist = np.mean(np.linalg.norm(frac_ps - all_ps[tmp_indices], axis=1))
            if dist < min_dist:
                index = j
                min_dist = dist
                indices = tmp_indices
        all_points[index] = np.delete(all_points[index], indices, axis=0)
    rest_pcds = []
    for i in range(len(all_points)):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(all_points[i])
        pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1, max_nn=30))
        rest_pcds.append(pcd)
    return rest_pcds
        

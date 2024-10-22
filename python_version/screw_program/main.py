# coding = utf-8
import argparse
import numpy as np
# import vtkmodules.all as vtk
import data_input
import visualization
import screw_setting
import core
import data_preprocess
from sklearn.decomposition import PCA
import core_software


def initialize(args):
    screw_setting.pcd_dir = args.pcd_dir
    screw_setting.stl_dir = args.stl_dir
    # screw_setting.mtx_dir = args.mtx_dir
    # screw_setting.label_dir = args.label_dir
    # screw_setting.img_dir = args.img_dir
    screw_setting.color = args.color
    screw_setting.svm_threshold = args.svm_threshold
    screw_setting.gep_threshold = args.gep_threshold
    screw_setting.sp_threshold = args.sp_threshold
    screw_setting.sp_radius = args.sp_radius
    screw_setting.screw_radius = args.screw_radius
    screw_setting.screw_length = args.screw_length
    screw_setting.line_length_rate = args.line_length_rate
    screw_setting.path_refine_radius = args.path_refine_radius
    screw_setting.rrd_eps_max = args.rrd_eps_max
    screw_setting.rrd_eps_min = args.rrd_eps_min
    screw_setting.ransac_eps = args.ransac_eps
    screw_setting.angle_eps = args.angle_eps
    screw_setting.dist_eps = args.dist_eps
    screw_setting.cone_angle = args.cone_angle
    screw_setting.r_res = args.r_res
    screw_setting.c_res = args.c_res
    screw_setting.length_eps = args.length_eps


def screw_program(args):
    stl_filenames = data_input.get_filenames(args.stl_dir, ".stl")
    pcd_filenames = data_input.get_filenames(args.pcd_dir, ".pcd")
    # mtx_filenames = data_input.get_filenames(args.mtx_dir, ".npy")
    stls = data_input.getSTLs(stl_filenames)
    all_pcds = data_input.getPCDfromSTL(stl_filenames)
    all_pcds = data_preprocess.downSample(all_pcds, voxel_size=1.5*screw_setting.voxel_size)
    # all_pcds = data_preprocess.downSample(all_pcds, voxel_size=screw_setting.voxel_size)
    # visualization.points_visualization_by_vtk(all_pcds)
    frac_pcds = data_preprocess.fromAllPCDs2FracPCDs(all_pcds)
    # visualization.stl_pcd_visualization_with_path_by_vtk1(stls, frac_pcds)
    # all_pcds = data_prepprocess.remove_outliers(all_pcds)
    # all_pcds = data_prepprocess.pcds_normals_outside(all_pcds)
    # frac_pcds = data_input.getPCDs(pcd_filenames)
    rest_pcds = data_preprocess.get_rest_pcds(all_pcds, frac_pcds)
    rest_pcds = data_preprocess.downSample(rest_pcds)
    rest_pcds_for_explore = data_preprocess.downSample(rest_pcds, voxel_size=2*screw_setting.voxel_size)
    visualization.stl_pcd_visualization_with_path_by_vtk1(stls, frac_pcds)
    # visualization.points_visualization_by_vtk(rest_pcds)
    # visualization.stl_pcd_visualization_by_vtk(stls, all_pcds, args.color)
    
    path_info = core_software.initial_program(frac_pcds, all_pcds, rest_pcds)
    
    
    # restPoints = np.empty((0, 3))
    # for pcd in rest_pcds:
    #     points = np.asarray(pcd.points)
    #     restPoints = np.concatenate([restPoints, points], axis=0)
    # allCenter = np.mean(restPoints, axis=0)
    # pca = PCA()
    # pca.fit(restPoints)
    # vec0 = np.array(pca.components_[0, :])
    # vec0 = vec0/np.linalg.norm(vec0)
    # vec1 = np.array(pca.components_[1, :])
    # vec1 = vec1/np.linalg.norm(vec1)
    # vec2 = np.array(pca.components_[2, :])
    # vec2 = vec2/np.linalg.norm(vec2)
    # dsp_dsbt = np.dot(restPoints, vec2.T)
    # tip_point = restPoints[np.argmax(dsp_dsbt)]
    # #  visualization.points_visualization_by_vtk(rest_pcds, [tip_point], radius=10)
    # if np.dot(tip_point - allCenter, vec1) > 0:
    #     vec1 = -vec1
    # allCenter = allCenter  - vec0*40 + vec1*80  - vec2*20 
    # vecs = [vec0, vec1, vec2]
    # test_info = []
    # for i in range(3):
    #     test_info.append([vecs[i], allCenter, 0, 0, 20*(3-i), 20*(3-i)])
    # visualization.best_result_visualization(stls, test_info, args.color)
    
    
    # rf_path_info_v1 = path_program.refine_path_info_v1(path_info, all_pcds)
    
    # rf_path_info_v2 = path_program.refine_path_info_v2(path_info, all_pcds)
    
    # rf_path_info_v3 = path_program.refine_path_info_v3(path_info, all_pcds)
    # visualization.best_result_visualization(stls, path_info, args.color)
    rf_path_info_v4 = core.refine_path_info_v4(stls, path_info, rest_pcds, rest_pcds_for_explore)
    tmp_path_info = core.add_screw_length(path_info, all_pcds)
    for i in range(min(len(tmp_path_info), len(rf_path_info_v4))):
        print("normal length1:%.2fmm, normal length2:%.2fmm, \nour method length1:%.2fmm, our method length2:%.2fmm"
              % (tmp_path_info[i][4], tmp_path_info[i][5], rf_path_info_v4[i][4], rf_path_info_v4[i][5]))
    visualization.compare_screw_program2(stls, tmp_path_info, rf_path_info_v4, args.color)
    visualization.stl_pcd_visualization_with_path_by_vtk(stls, frac_pcds, rf_path_info_v4, args.color)
    visualization.best_result_visualization(stls, rf_path_info_v4, args.color)
    # print(np.array(rf_path_info_v4))
    
    
if __name__ == '__main__':
    print("start")
    parser = argparse.ArgumentParser()
    parser.add_argument("--pcd_dir",
                        default=screw_setting.pcd_dir,
                        type=str)
    parser.add_argument("--stl_dir",
                        default=screw_setting.stl_dir,
                        type=str)
    # parser.add_argument("--mtx_dir",
    #                     default=screw_setting.mtx_dir,
    #                     type=str)
    # parser.add_argument("--label_dir",
    #                     default=screw_setting.label_dir,
    #                     type=str)
    # parser.add_argument("--img_dir",
    #                     default=screw_setting.img_dir,
    #                     type=str)
    parser.add_argument("--color",
                        default=screw_setting.color,
                        type=list)
    parser.add_argument("--svm_threshold",
                        default=screw_setting.svm_threshold,
                        type=float)
    parser.add_argument("--gep_threshold",
                        default=screw_setting.gep_threshold,
                        type=float)
    parser.add_argument("--sp_threshold",
                        default=screw_setting.sp_threshold,
                        type=float)
    parser.add_argument("--sp_radius",
                        default=screw_setting.sp_radius,
                        type=float)
    parser.add_argument("--screw_radius",
                        default=screw_setting.screw_radius,
                        type=float)
    parser.add_argument("--screw_length",
                        default=screw_setting.screw_length,
                        type=float)
    parser.add_argument("--line_length_rate",
                        default=screw_setting.line_length_rate,
                        type=float)
    parser.add_argument("--path_refine_radius",
                        default=screw_setting.path_refine_radius,
                        type=float)
    parser.add_argument("--rrd_eps_max",
                        default=screw_setting.rrd_eps_max,
                        type=float)
    parser.add_argument("--rrd_eps_min",
                        default=screw_setting.rrd_eps_min,
                        type=float)
    parser.add_argument("--ransac_eps",
                        default=screw_setting.ransac_eps,
                        type=float)
    parser.add_argument("--angle_eps",
                        default=screw_setting.angle_eps,
                        type=float)
    parser.add_argument("--dist_eps",
                        default=screw_setting.dist_eps,
                        type=float)
    parser.add_argument("--cone_angle",
                        default=screw_setting.cone_angle,
                        type=float)
    parser.add_argument("--r_res",
                        default=screw_setting.r_res,
                        type=float)
    parser.add_argument("--c_res",
                        default=screw_setting.c_res,
                        type=float)
    parser.add_argument("--length_eps",
                        default=screw_setting.length_eps,
                        type=float)
    args = parser.parse_args()
    print("initialize")
    initialize(args)
    print("screw program")
    screw_program(args)

# coding = utf-8
import argparse
import numpy as np
# import vtkmodules.all as vtk
import data_input
import visualization
import screw_setting
import path_program


def initialize(args):
    screw_setting.pcd_dir = args.pcd_dir
    screw_setting.stl_dir = args.stl_dir
    screw_setting.mtx_dir = args.mtx_dir
    screw_setting.label_dir = args.label_dir
    screw_setting.img_dir = args.img_dir
    screw_setting.color = args.color
    screw_setting.svm_threshold = args.svm_threshold
    screw_setting.gep_threshold = args.gep_threshold
    screw_setting.sp_threshold = args.sp_threshold
    screw_setting.sp_radius = args.sp_radius
    screw_setting.screw_radius = args.screw_radius
    screw_setting.screw_length = args.screw_length


def screw_program(args):
    stl_filenames = data_input.get_filenames(args.stl_dir, ".stl")
    pcd_filenames = data_input.get_filenames(args.pcd_dir, ".pcd")
    mtx_filenames = data_input.get_filenames(args.mtx_dir, ".npy")
    
    stls = data_input.getSTLs(stl_filenames)
    all_pcds = data_input.getPCDfromSTL(stl_filenames)
    frac_pcds = data_input.getPCDs(pcd_filenames)
    mtxs = data_input.getNPYs(mtx_filenames)
    # img = data_input.getNIIasNPY(args.img_dir)
    # label = data_input.getNIIasNPY(args.label_dir)
    
    # img = img*label
    
    # print(img.shape)
    # print(label.shape)
    
    # visualization.stl_pcd_visualization_by_vtk(stls, pcds, args.color)
    
    path_info = path_program.path_program(frac_pcds, all_pcds)
    
    # rf_path_info_v1 = path_program.refine_path_info_v1(path_info, all_pcds)
    
    # rf_path_info_v2 = path_program.refine_path_info_v2(path_info, all_pcds)
    
    # rf_path_info_v3 = path_program.refine_path_info_v3(path_info, all_pcds)
    
    rf_path_info_v4 = path_program.refine_path_info_v4(path_info, all_pcds)
    visualization.compare_screw_program2(stls, path_info, rf_path_info_v4, args.color)
    # visualization.compare_screw_program4(stls, path_info, rf_path_info_v1, rf_path_info_v2, rf_path_info_v4, args.color)
    visualization.stl_pcd_visualization_with_path_by_vtk(stls, frac_pcds, rf_path_info_v4, args.color)
    
    print(np.array(mtxs))
    print(np.array(path_info))
    
    
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--pcd_dir",
                        default=screw_setting.pcd_dir,
                        type=str)
    parser.add_argument("--stl_dir",
                        default=screw_setting.stl_dir,
                        type=str)
    parser.add_argument("--mtx_dir",
                        default=screw_setting.mtx_dir,
                        type=str)
    parser.add_argument("--label_dir",
                        default=screw_setting.label_dir,
                        type=str)
    parser.add_argument("--img_dir",
                        default=screw_setting.img_dir,
                        type=str)
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
    args = parser.parse_args()
    initialize(args)
    screw_program(args)

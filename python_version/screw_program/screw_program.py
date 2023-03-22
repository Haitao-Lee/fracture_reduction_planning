# coding = utf-8
import numpy as np
import data_input_software
import visualization_software
import screw_setting
import core_software
import data_preprocess_software


def screw_program(stl_folder, pcd_folder):
    stl_filenames = data_input_software.get_filenames(stl_folder, ".stl")
    pcd_filenames = data_input_software.get_filenames(pcd_folder, ".pcd")
    stls = data_input_software.getSTLs(stl_filenames)
    all_pcds = data_input_software.getPCDfromSTL(stl_filenames)
    frac_pcds = data_input_software.getPCDs(pcd_filenames)
    rest_pcds = data_preprocess_software.get_rest_pcds(all_pcds, frac_pcds)
    rest_pcds = data_preprocess_software.downSample(rest_pcds)
    rest_pcds_for_explore = data_preprocess_software.downSample(rest_pcds, voxel_size=2*screw_setting.voxel_size)
    path_info = core_software.initial_program(frac_pcds, all_pcds, rest_pcds)
    optimal_info = core_software.get_optimal_info(path_info, rest_pcds, rest_pcds_for_explore)
    visualization_software.best_result_visualization(stls, optimal_info)
    screw_actors = []
    screw_info = []
    for info in optimal_info:
        dire = info[0]
        cent = info[1]
        length1 = info[4]
        length2 = info[5]
        screw_info.append(np.array([cent - length2*dire, cent + length1*dire]))
        screw_actors.append(visualization_software.get_screw_actor(cent, dire, length1, length2)[0])
    return screw_info, screw_actors

info, actors = screw_program(screw_setting.stl_dir, screw_setting.pcd_dir)
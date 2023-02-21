# coding = utf-8
import argparse
# from multiprocessing import Pool
import data_input
import data_preprocess
import feature_extract
import reduction_setting
import visualization
import register
from threading import Thread as thread
# import numpy as np


class MyThreadForRefine(thread):

    def __init__(self, feature_pcd, pcd, r, threshold):
        thread.__init__(self)
        self.args = args
        self.feature_pcd = feature_pcd
        self.pcd = pcd
        self.r = r
        self.threshold = threshold

    def run(self):
        self.result = feature_extract.refine_feature_points(
            self.feature_pcd, self.pcd, self.r, self.threshold)

    def get_result(self):
        return self.result


def initialize(args):
    reduction_setting.folder_path = args.dataset_dir
    reduction_setting.file_type = args.file_type
    reduction_setting.volume_threshold = args.vth
    reduction_setting.voxel_size = args.vxs
    reduction_setting.effect_rate = args.effect_rate
    reduction_setting.sd_rate = args.sd_rate
    reduction_setting.PCD_color = args.color_space
    reduction_setting.r = args.r
    reduction_setting.num_threshold = args.num_threshold
    reduction_setting.svm_c = args.svm_c
    reduction_setting.phy_iter = args.phy_iter


# mode0 only performs the reduction of 2 fractures
def mode0(args):
    ref_STL = data_input.getSTLs([args.fragment1])[0]
    ref_PCD = data_input.getPCDs([args.fragment1])[0]
    tar_STL = data_input.getSTLs([args.fragment2])[0]
    tar_PCD = data_input.getPCDs([args.fragment2])[0]
    ds_PCDs = data_preprocess.downSample([ref_PCD, tar_PCD], args.vxs)
    print("\n--downsampled point cloud info:")
    for ds_PCD in ds_PCDs:
        print(ds_PCD)
    ref_feature_pcd, tar_feature_pcd, ref_effect_pcd, tar_effect_pcd, ref_center, tar_center = feature_extract.get_feature_pcd(
        ds_PCDs[0], ds_PCDs[1], args.effect_rate, args.sd_rate, args.r)
    visualization.points_visualization_by_vtk(
        [ds_PCDs[0], ds_PCDs[1], ref_feature_pcd, tar_feature_pcd],
        args.color_space)
    # refine_args = [(ref_feature_pcd, ds_PCDs[0], args.r, args.num_threshold), (tar_feature_pcd, ds_PCDs[1], args.r, args.num_threshold)]
    # with Pool(processes=2) as pool:
    #     ref_pcds = pool.map(feature_extract.refine_feature_points, refine_args)
    #     print(ref_pcds)
    # ref_thread = MyThreadForRefine(ref_feature_pcd, ds_PCDs[0], args.r, args.num_threshold)
    # tar_thread = MyThreadForRefine(tar_feature_pcd, ds_PCDs[1], args.r, args.num_threshold)
    # ref_thread.start()
    # tar_thread.start()
    # ref_thread.join()
    # tar_thread.join()
    # ref_feature_pcds = ref_thread.get_result()
    # tar_feature_pcds = tar_thread.get_result()
    ref_feature_pcd = feature_extract.refine_feature_points(
        ref_feature_pcd, ref_effect_pcd, 2 * args.r, args.num_threshold)
    # refined_ref_points = np.asarray(ref_feature_pcd.points)
    # np.save("./target.npy", refined_ref_points)
    tar_feature_pcd = feature_extract.refine_feature_points(
        tar_feature_pcd, tar_effect_pcd, 2 * args.r, args.num_threshold)
    # refined_tar_points = np.asarray(tar_feature_pcd.points)
    # np.save("./source.npy", refined_tar_points)
    # feature_pcds = data_preprocess.downSample([ref_feature_pcd, tar_feature_pcd], 2*args.vxs)
    # ref_feature_pcd = feature_pcds[0]
    # tar_feature_pcd = feature_pcds[1]
    visualization.points_visualization_by_vtk([ref_feature_pcd, tar_feature_pcd], args.color_space)
    # trans_matrix = register.get_matching_matrix_by_o3d_ICP(tar_feature_pcd, ref_feature_pcd, 0)
    register.physical_register(ref_feature_pcd, tar_feature_pcd, 3*args.vxs, args.svm_c, args.phy_iter)
    # ref_feature_pcds, ref_valleys = feature_extract.cluster_by_gravity(ref_feature_pcd, ref_center-tar_center, 3*args.vxs)
    # tar_feature_pcds, tar_valleys = feature_extract.cluster_by_gravity(tar_feature_pcd, tar_center-ref_center, 3*args.vxs)
    # visualization.point_ball_visualize(ref_feature_pcds+tar_feature_pcds, ref_valleys+tar_valleys, args.color_space, 3*args.vxs)
    # visualization.points_visualization_by_vtk(ref_feature_pcds+tar_feature_pcds, args.color_space)
    trans_matrix = register.get_matching_matrix_by_vtk_ICP(
        tar_feature_pcd, ref_feature_pcd, 2000)
    # # tar_down, ref_down, tar_fpfh, ref_fpfh = register.prepare_dataset(tar_feature_pcd, ref_feature_pcd, args.vxs)
    # # trans_matrix = register.execute_global_registration(tar_down, ref_down, tar_fpfh, ref_fpfh, args.vxs).transformation
    print("--transformation matrix:")
    print(trans_matrix)
    tar_feature_pcd.transform(trans_matrix)

    visualization.points_visualization_by_vtk(
        [ref_feature_pcd, tar_feature_pcd],
        args.color_space)
    visualization.stl_visualization_by_vtk(
        [register.transform_stl(tar_STL, trans_matrix), ref_STL],
        [tar_STL, ref_STL], args.color_space)
    return 0


# mode1 performs global reduction planning
def mode1(args):
    fileNames = data_input.get_filenames(args.dataset_dir, args.file_type)
    STLs = data_input.getSTLs(fileNames)
    PCDs = data_input.getPCDs(fileNames)
    filtered_STLs, filtered_PCDs = data_preprocess.hp_filter(
        STLs, PCDs, args.vth)
    print("\n--high-pass filtered point cloud info:")
    for filtered_PCD in filtered_PCDs:
        print(filtered_PCD)
    ds_PCDs = data_preprocess.downSample(filtered_PCDs, args.vxs)
    print("\n--downsampled point cloud info:")
    for ds_PCD in ds_PCDs:
        print(ds_PCD)
    return 0


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", default=0, type=int)
    parser.add_argument("--dataset_dir",
                        default=reduction_setting.folder_path,
                        type=str)
    parser.add_argument("--file_type",
                        default=reduction_setting.file_type,
                        type=str)
    parser.add_argument("--vth",
                        type=float,
                        default=reduction_setting.volume_threshold)
    parser.add_argument("--vxs", type=float, default=reduction_setting.voxel_size)
    parser.add_argument("--effect_rate",
                        type=float,
                        default=reduction_setting.effect_rate)
    parser.add_argument("--sd_rate",
                        type=float,
                        default=reduction_setting.sd_rate)
    parser.add_argument("--color_space",
                        type=list,
                        default=reduction_setting.PCD_color)
    parser.add_argument("--r", type=float, default=reduction_setting.r)
    parser.add_argument("--num_threshold",
                        type=int,
                        default=reduction_setting.num_threshold)
    parser.add_argument("--svm_c",
                        type=int,
                        default=reduction_setting.svm_c)
    parser.add_argument("--phy_iter",
                        type=int,
                        default=reduction_setting.phy_iter)
    parser.add_argument("--fragment1",
                        type=str,
                        default=str(reduction_setting.folder_path) +
                        "/00900005.stl")
    parser.add_argument("--fragment2",
                        type=str,
                        default=str(reduction_setting.folder_path) +
                        "/00900003.stl")
    args = parser.parse_args()
    initialize(args)
    if args.mode == 0:
        mode0(args)
    elif args.mode == 1:
        mode1(args)

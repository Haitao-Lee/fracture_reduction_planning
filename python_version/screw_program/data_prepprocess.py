# coding = utf-8
import screw_setting


def remove_outliers(pcds, nd=screw_setting.nd, std_rt=screw_setting.std_rt):
    new_pcds = []
    for pcd in pcds:
        n_pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=nd, std_ratio=std_rt)
        new_pcds.append(n_pcd)
    return new_pcds

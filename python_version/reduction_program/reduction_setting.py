# coding = utf-8

# the folder path that stores STL files
folder_path = r"E:\code\3d-fracture-reassembly\src\dataset_generation\stl"

# type of the input files, here it is '.stl'
file_type = ".stl"

# apply high-pass filter to the volume by the threshold
volume_threshold = 1

# damplesample the pointcloud by this voxel size
voxel_size = 1

# the rate effect points to all points
effect_rate = 0.3

# the radius of ball neighbourhood
r = 4*voxel_size

# minimum number of points in one cluster
num_threshold = (r/voxel_size-1)**2

# the rate feature points to effect points
sd_rate = 1.6

# threshold in SVM
svm_c = 1.5

# number of physical register iterations
phy_iter = 100 

# pointcloud color
PCD_color = [0,   1,   0,
             1,   0,   0,
             0,   0,   1,
             1,   1,   0,
             1,   0,   1,
             0,   1,   1,
             1,   1,   1]

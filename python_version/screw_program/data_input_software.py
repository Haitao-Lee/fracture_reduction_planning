# coding = utf-8
import os
import vtkmodules.all as vtk
import open3d as o3d
import numpy as np
import nibabel as nib


def get_filenames(path, filetype):  # 输入路径、文件类型例如'.csv'
    names = []
    for _, _, files in os.walk(path):
        for i in files:
            if os.path.splitext(i)[1] == filetype:
                names.append(path + '/' + i)
    return names  # 输出由有后缀的文件名组成的列表


# obtain the lists of STL files in the same folder
def getSTLs(fileNames):
    STLs = []
    for fileName in fileNames:
        stl_reader = vtk.vtkSTLReader()
        stl_reader.SetFileName(fileName)
        stl_reader.Update()
        STLs.append(stl_reader.GetOutput())
    return STLs


# obtain the lists of PCD files in the same folder
def getPCDs(fileNames):
    PCDs = []
    for fileName in fileNames:
        PCD = o3d.io.read_point_cloud(fileName)
        PCDs.append(PCD)
    return PCDs


# obtain the lists of npy files in the same folder
def getNPYs(fileNames):
    NPYs = []
    for fileName in fileNames:
        NPY = np.load(fileName)
        NPYs.append(NPY)
    return NPYs


def getNIIasNPY(fileName):
    img = nib.load(fileName).get_data() #载入
    img = np.array(img)
    return img


def getPCDfromSTL(fileNames):
    PCDs = []
    for fileName in fileNames:
        mesh_ply = o3d.io.read_triangle_mesh(fileName)
        PCD = o3d.geometry.PointCloud()
        PCD.points = mesh_ply.vertices
        PCD.normals = mesh_ply.vertex_normals
        PCD.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1, max_nn=30))
        PCDs.append(PCD)
    return PCDs

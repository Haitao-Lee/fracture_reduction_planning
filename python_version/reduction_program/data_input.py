# coding = utf-8
import os
import vtkmodules.all as vtk
import open3d as o3d


def get_filenames(path, filetype):  # 输入路径、文件类型例如'.csv'
    names = []
    for root, dirs, files in os.walk(path):
        for i in files:
            if os.path.splitext(i)[1] == filetype:
                names.append(path + '/' + i)
    return names  # 输出由有后缀的文件名组成的列表


# obtain the lists of STL files in the same folder
def getSTLs(fileNames):
    STLs = []
    print("--reading files:")
    for fileName in fileNames:
        print(fileName)
        stl_reader = vtk.vtkSTLReader()
        stl_reader.SetFileName(fileName)
        stl_reader.Update()
        STLs.append(stl_reader.GetOutput())
    return STLs


# obtain the lists of PCD files in the same folder
def getPCDs(fileNames):
    PCDs = []
    for fileName in fileNames:
        mesh_ply = o3d.io.read_triangle_mesh(fileName)
        # mesh_ply.compute_vertex_normals()
        PCD = o3d.geometry.PointCloud()
        PCD.points = mesh_ply.vertices
        PCD.normals = mesh_ply.vertex_normals
        PCDs.append(PCD)
    return PCDs

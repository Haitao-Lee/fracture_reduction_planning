# coding = utf-8
import vtkmodules.all as vtk


# compute volum of a polyData
def compute_volum(polyData):
    tri_filter = vtk.vtkTriangleFilter()
    tri_filter.SetInputData(polyData)
    tri_filter.Update()

    mas_prop = vtk.vtkMassProperties()
    mas_prop.SetInputData(tri_filter.GetOutput())
    mas_prop.Update()
    return mas_prop.GetVolume()


# compute the curvature of each point in the pointcloud
def compute_curvature(PCDs):
    return PCDs


# high-pass filter on the volume of STLs
def hp_filter(STLs, PCDs, threshold):
    for i in range(0, len(STLs)):
        if compute_volum(STLs[i]) < threshold:
            STLs.delete(i)
            PCDs.delete(i)
    return STLs, PCDs


# set reference
def get_reference(rank, STLs):
    if rank < 0 or rank > len(STLs):
        return STLs[0], -1
    else:
        return STLs[rank], 0


# downsample pointcloud
def downSample(PCDs, voxel_size):
    downSamplePCDs = []
    for PCD in PCDs:
        downSamplePCDs.append(PCD.voxel_down_sample(voxel_size))
    return downSamplePCDs


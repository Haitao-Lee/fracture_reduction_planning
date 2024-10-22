# coding = utf-8
import vtkmodules.all as vtk
import numpy as np
import screw_setting
import math
import os
import matplotlib.pyplot as plt


# def points_visualization_by_vtk(PCDs, centers=None, radius=screw_setting.screw_radius, pcd_radius=screw_setting.pcd_radius, color=screw_setting.color):
#     renderer = vtk.vtkRenderer()
#     for i in range(0, len(PCDs)):
#         xyz = np.asarray(PCDs[i].points)
#         pcd_polydata = vtk.vtkAppendPolyData()
#         for j in range(0, xyz.shape[0]):
#             sphere = vtk.vtkSphereSource()
#             sphere.SetCenter(xyz[j])
#             sphere.SetRadius(pcd_radius)
#             sphere.Update()
#             pcd_polydata.AddInputConnection(sphere.GetOutputPort())

#         ply_mapper = vtk.vtkPolyDataMapper()
#         ply_mapper.SetInputData(pcd_polydata.GetOutput())
#         ply_mapper.Update()

#         ply_actor = vtk.vtkActor()
#         ply_actor.SetMapper(ply_mapper)
#         ply_actor.GetProperty().SetColor(color[(3 * i) % len(color)],
#                                          color[(3 * i + 1) % len(color)],
#                                          color[(3 * i + 2) % len(color)])
#         renderer.AddActor(ply_actor)
#     if centers is not None:
#         for center in centers:
#             sph_actor = get_sphere_actor(center, radius, (1, 0, 0))
#             sph_actor.GetProperty().SetOpacity(0.8)
#             renderer.AddActor(sph_actor)
#     render_window = vtk.vtkRenderWindow()
#     render_window.AddRenderer(renderer)
#     rw_style = vtk.vtkInteractorStyleTrackballCamera()
#     rw_interactor = vtk.vtkRenderWindowInteractor()
#     rw_interactor.SetRenderWindow(render_window)
#     rw_interactor.SetInteractorStyle(rw_style)
#     render_window.Render()
#     rw_interactor.Initialize()
#     rw_interactor.Start()

 
def viz_matplot(points):
    x = points[:, 0]  # x position of point
    y = points[:, 1]  # y position of point
    z = points[:, 2]  # z position of point
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(x,   # x
               y,   # y
               z,   # z
               c=z, # height data for color
               cmap='rainbow',
               marker="x")
    ax.axis()
    plt.show()


def points_visualization_by_vtk(PCDs, centers=None, radius=screw_setting.screw_radius, color=screw_setting.color):
    renderer = vtk.vtkRenderer()
    renderer.SetBackground(1, 1, 1)
    for i in range(0, len(PCDs)):
        vtk_points = vtk.vtkPoints()
        xyz = np.asarray(PCDs[i].points)
        for j in range(0, xyz.shape[0]):
            vtk_points.InsertNextPoint(xyz[j][0], xyz[j][1], xyz[j][2])

        ply = vtk.vtkPolyData()
        ply.SetPoints(vtk_points)
        # ply.GetPointData().SetScalars([color[(3 * i) % len(color)],
        #                               color[(3 * i + 1) % len(color)],
        #                               color[(3 * i + 2) % len(color)]])
        
        sphere_source = vtk.vtkSphereSource()
        
        glyph = vtk.vtkGlyph3D()
        glyph.SetSourceConnection(sphere_source.GetOutputPort())
        glyph.SetInputData(ply)
        glyph.ScalingOff()
        glyph.Update()

        ply_mapper = vtk.vtkPolyDataMapper()
        ply_mapper.SetInputData(glyph.GetOutput())
        ply_mapper.Update()

        ply_actor = vtk.vtkActor()
        ply_actor.SetMapper(ply_mapper)
        ply_actor.GetProperty().SetColor(color[(3 * i) % len(color)],
                                         color[(3 * i + 1) % len(color)],
                                         color[(3 * i + 2) % len(color)])
        ply_actor.GetProperty().SetOpacity(0.8)
        renderer.AddActor(ply_actor)
    if centers is not None:
        for center in centers:
            sph_actor = get_sphere_actor(center, radius, (1, 0, 0))
            sph_actor.GetProperty().SetOpacity(0.4)
            renderer.AddActor(sph_actor)
    render_window = vtk.vtkRenderWindow()
    render_window.AddRenderer(renderer)
    rw_style = vtk.vtkInteractorStyleTrackballCamera()
    rw_interactor = vtk.vtkRenderWindowInteractor()
    rw_interactor.SetRenderWindow(render_window)
    rw_interactor.SetInteractorStyle(rw_style)
    render_window.Render()
    rw_interactor.Initialize()
    rw_interactor.Start()


def stl_visualization_by_vtk(result_stls, origin_stls, color=screw_setting.color):
    result_renderer = vtk.vtkRenderer()
    result_renderer.SetViewport(0, 0, 0.5, 1)
    origin_renderer = vtk.vtkRenderer()
    origin_renderer.SetViewport(0.5, 0, 1, 1)
    for i in range(0, len(result_stls)):
        result_stl = result_stls[i]
        result_ply_mapper = vtk.vtkPolyDataMapper()
        result_ply_mapper.SetInputData(result_stl)
        result_ply_mapper.Update()

        result_ply_actor = vtk.vtkActor()
        result_ply_actor.SetMapper(result_ply_mapper)
        result_ply_actor.GetProperty().SetColor(color[(3 * i) % len(color)],
                                                color[(3 * i + 1) % len(color)],
                                                color[(3 * i + 2) % len(color)])
        result_renderer.AddActor(result_ply_actor)

        origin_stl = origin_stls[i]
        origin_ply_mapper = vtk.vtkPolyDataMapper()
        origin_ply_mapper.SetInputData(origin_stl)
        origin_ply_mapper.Update()

        origin_ply_actor = vtk.vtkActor()
        origin_ply_actor.SetMapper(origin_ply_mapper)
        origin_ply_actor.GetProperty().SetColor(color[(3 * i) % len(color)],
                                                color[(3 * i + 1) % len(color)],
                                                color[(3 * i + 2) % len(color)])
        origin_renderer.AddActor(origin_ply_actor)
    render_window = vtk.vtkRenderWindow()
    render_window.AddRenderer(result_renderer)
    render_window.AddRenderer(origin_renderer)
    rw_style = vtk.vtkInteractorStyleTrackballCamera()
    rw_interactor = vtk.vtkRenderWindowInteractor()
    rw_interactor.SetRenderWindow(render_window)
    rw_interactor.SetInteractorStyle(rw_style)
    render_window.Render()
    rw_interactor.Initialize()
    rw_interactor.Start()
 
    
def stl_pcd_visualization_by_vtk(stls, pcds, color=screw_setting.color):
    stl_renderer = vtk.vtkRenderer()
    stl_renderer.SetViewport(0, 0, 0.5, 1)
    for i in range(0, len(stls)):
        stl = stls[i]
        stl_ply_mapper = vtk.vtkPolyDataMapper()
        stl_ply_mapper.SetInputData(stl)
        stl_ply_mapper.Update()

        stl_ply_actor = vtk.vtkActor()
        stl_ply_actor.SetMapper(stl_ply_mapper)
        stl_ply_actor.GetProperty().SetColor(color[(3 * i) % len(color)],
                                             color[(3 * i + 1) % len(color)],
                                             color[(3 * i + 2) % len(color)])
        stl_renderer.AddActor(stl_ply_actor)

    pcd_renderer = vtk.vtkRenderer()
    pcd_renderer.SetViewport(0.5, 0, 1, 1)
    for i in range(0, len(pcds)):
        vtk_points = vtk.vtkPoints()
        xyz = np.asarray(pcds[i].points)
        for j in range(0, xyz.shape[0]):
            vtk_points.InsertNextPoint(xyz[j][0], xyz[j][1], xyz[j][2])

        ply = vtk.vtkPolyData()
        ply.SetPoints(vtk_points)
        # ply.GetPointData().SetScalars([color[(3 * i) % len(color)],
        #                               color[(3 * i + 1) % len(color)],
        #                               color[(3 * i + 2) % len(color)]])
        
        sphere_source = vtk.vtkSphereSource()
        
        glyph = vtk.vtkGlyph3D()
        glyph.SetSourceConnection(sphere_source.GetOutputPort())
        glyph.SetInputData(ply)
        glyph.ScalingOff()
        glyph.Update()

        ply_mapper = vtk.vtkPolyDataMapper()
        ply_mapper.SetInputData(glyph.GetOutput())
        ply_mapper.Update()

        ply_actor = vtk.vtkActor()
        ply_actor.SetMapper(ply_mapper)
        ply_actor.GetProperty().SetColor(color[(3 * i) % len(color)],
                                         color[(3 * i + 1) % len(color)],
                                         color[(3 * i + 2) % len(color)])
        pcd_renderer.AddActor(ply_actor)
    
    render_window = vtk.vtkRenderWindow()
    render_window.AddRenderer(stl_renderer)
    render_window.AddRenderer(pcd_renderer)
    rw_style = vtk.vtkInteractorStyleTrackballCamera()
    rw_interactor = vtk.vtkRenderWindowInteractor()
    rw_interactor.SetRenderWindow(render_window)
    rw_interactor.SetInteractorStyle(rw_style)
    render_window.Render()
    rw_interactor.Initialize()
    rw_interactor.Start()

    
def get_screw_line_actor(center, direct, length_rate=screw_setting.line_length_rate, screw_length=screw_setting.screw_length):
    dashed = vtk.vtkAppendPolyData()
    start = center - 0.5*length_rate*screw_length*direct
    for i in range(2*length_rate):
        p1 = start
        p2 = start + 3/8*screw_length*direct
        line = vtk.vtkLineSource()
        line.SetPoint1(p1)
        line.SetPoint2(p2)
        line.Update()
        dashed.AddInputConnection(line.GetOutputPort())
        start = start + 0.5*screw_length*direct
    dashed.Update()
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(dashed.GetOutputPort())
    
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(1, 0, 0)
    actor.GetProperty().SetLineWidth(0.5)
    return actor
    
    
def get_screw_cylinder_actor(center, direct, radius=screw_setting.screw_radius, length=screw_setting.screw_length):
    cylinder = vtk.vtkCylinderSource()
    cylinder.SetHeight(length)
    cylinder.SetRadius(radius)
    cylinder.SetResolution(6)
    cylinder.Update()
    
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(cylinder.GetOutputPort())
    
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(1, 1, 1)
    direct = direct/np.linalg.norm(direct)
    rotate_axis = np.cross([0, 1, 0], direct)
    angle = math.acos(direct[1])*180/math.pi
    actor.RotateWXYZ(angle, rotate_axis[0], rotate_axis[1], rotate_axis[2])
    actor.AddPosition(center)
    return actor
    
    # point1 = center - length/2*direct
    # point2 = center + length/2*direct
    # line = vtk.vtkLineSource()
    # line.SetPoint1(point1)
    # line.SetPoint2(point2)
    
    # tube = vtk.vtkTubeFilter()
    # tube.SetInputConnection(line.GetOutputPort())
    # tube.SetRadius(screw_setting.screw_radius)
    # tube.SetNumberOfSides(10);
    # tube.SetCapping(0)
    # tube.Update()
 
    # tubeMapper = vtk.vtkPolyDataMapper()
    # tubeMapper.SetInputConnection(tube.GetOutputPort())
    
    # tubeActor = vtk.vtkActor()
    # tubeActor.SetMapper(tubeMapper)
    # tubeActor.GetProperty().SetColor(1, 1, 1)
    # return tubeActor


def get_screw_actor(center, direct, length1, length2, radius=screw_setting.screw_radius):
    cylinder = vtk.vtkCylinderSource()
    length = length1 + length2
    cylinder.SetHeight(length)
    cylinder.SetRadius(0.7*radius)
    cylinder.SetResolution(6)
    cylinder.Update()
    
    head_cylinder = vtk.vtkCylinderSource()
    head_cylinder.SetHeight(4)
    head_cylinder.SetRadius(1.5*radius)
    head_cylinder.SetResolution(6)
    head_cylinder.Update()
    
    tf = vtk.vtkTransform()
    tf.Translate(0, length/2-2, 0)
    tf.Update()
    
    tf_pf = vtk.vtkTransformPolyDataFilter()
    tf_pf.SetInputData(head_cylinder.GetOutput())
    tf_pf.SetTransform(tf)
    tf_pf.Update()
    
    screw = vtk.vtkAppendPolyData()
    screw.AddInputData(cylinder.GetOutput())
    screw.AddInputData(tf_pf.GetOutput())
    
    for i in range(round(length)):
        thread_cylinder = vtk.vtkCylinderSource()
        thread_cylinder.SetHeight(0.5)
        thread_cylinder.SetRadius(radius)
        thread_cylinder.SetResolution(6)
        thread_cylinder.Update()
        thread_tf = vtk.vtkTransform()
        thread_tf.Translate(0, -length/2 + i, 0)
        thread_tf.Update()
        
        thread_tf_pf = vtk.vtkTransformPolyDataFilter()
        thread_tf_pf.SetInputData(thread_cylinder.GetOutput())
        thread_tf_pf.SetTransform(thread_tf)
        thread_tf_pf.Update()
        
        screw.AddInputData(thread_tf_pf.GetOutput())
        
    screw.Update()
    direct = direct/np.linalg.norm(direct)
    rotate_axis = np.cross([0, -1, 0], direct)
    angle = math.acos(-direct[1])*180/math.pi
    
    tf = vtk.vtkTransform()
    tf.PostMultiply()
    tf.RotateWXYZ(angle, rotate_axis[0], rotate_axis[1], rotate_axis[2])
    tf.Translate((length1 - length/2)*direct + center)
    tf.Update()
    
    tf_polydata = vtk.vtkTransformPolyDataFilter()
    tf_polydata.SetTransform(tf)
    tf_polydata.SetInputConnection(screw.GetOutputPort())
    
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(tf_polydata.GetOutputPort())
    
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(0.4, 0.4, 0.4)
    # actor.RotateWXYZ(angle, rotate_axis[0], rotate_axis[1], rotate_axis[2])
    # actor.AddPosition((length1 - length/2)*direct + center)
    return actor, tf_polydata


def get_sphere_actor(center, radius, color=(1, 1, 1)):
    sphere = vtk.vtkSphereSource()
    sphere.SetCenter(center)
    sphere.SetRadius(radius)
    sphere.Update()
    
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(sphere.GetOutputPort())
    
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(color)
    return actor

       
def stl_pcd_visualization_with_path_by_vtk(stls, pcds, path_info, color=screw_setting.color, save=screw_setting.save_stl):
    stl_renderer = vtk.vtkRenderer()
    stl_renderer.SetViewport(0, 0.5, 0.5, 1)
    stl_renderer.SetBackground(1, 1, 1)
    pcd_screw_renderer = vtk.vtkRenderer()
    pcd_screw_renderer.SetViewport(0.5, 0, 1, 0.5)
    for i in range(0, len(stls)):
        stl = stls[i]
        stl_ply_mapper = vtk.vtkPolyDataMapper()
        stl_ply_mapper.SetInputData(stl)
        stl_ply_mapper.Update()

        stl_ply_actor = vtk.vtkActor()
        stl_ply_actor.SetMapper(stl_ply_mapper)
        stl_ply_actor.GetProperty().SetColor(color[(3 * i) % len(color)],
                                             color[(3 * i + 1) % len(color)],
                                             color[(3 * i + 2) % len(color)])
        stl_renderer.AddActor(stl_ply_actor)
    
    stl_screw_renderer = vtk.vtkRenderer()
    stl_screw_renderer.SetViewport(0, 0, 0.5, 0.5)
    stl_screw_renderer.SetBackground(1, 1, 1)
    for i in range(0, len(stls)):
        stl = stls[i]
        stl_ply_mapper = vtk.vtkPolyDataMapper()
        stl_ply_mapper.SetInputData(stl)
        stl_ply_mapper.Update()

        stl_ply_actor = vtk.vtkActor()
        stl_ply_actor.SetMapper(stl_ply_mapper)
        stl_ply_actor.GetProperty().SetColor(color[(3 * i) % len(color)],
                                             color[(3 * i + 1) % len(color)],
                                             color[(3 * i + 2) % len(color)])
        stl_ply_actor.GetProperty().SetOpacity(0.1)
        pcd_screw_renderer.AddActor(stl_ply_actor)
        stl_screw_renderer.AddActor(stl_ply_actor)
        stl_writer = vtk.vtkSTLWriter()
        stl_writer.SetFileName(save + '/frac%d.stl' % i)
        if not os.path.exists(save):
            os.makedirs(save)
        stl_writer.SetInputData(stl)
        stl_writer.Write()
    for i in range(len(path_info)):
        path = path_info[i]
        path_dir = path[0]
        path_center = path[1]
        length1 = path[4]
        length2 = path[5]
        # screw_cylinder_actor = get_screw_cylinder_actor(path_center, path_dir)
        screw_actor, screw_polydata = get_screw_actor(path_center, path_dir, length1, length2)
        screw_line_actor = get_screw_line_actor(path_center, path_dir)
        # sphere_actor = get_sphere_actor(path_center, 10)
        stl_screw_renderer.AddActor(screw_actor)
        stl_screw_renderer.AddActor(screw_line_actor)
        # stl_screw_renderer.AddActor(sphere_actor)
        stl_writer = vtk.vtkSTLWriter()
        stl_writer.SetFileName(save + '/screw_%d.stl' % i)
        if not os.path.exists(save):
            os.makedirs(save)
        stl_writer.SetInputConnection(screw_polydata.GetOutputPort())
        stl_writer.Write()
    pcd_renderer = vtk.vtkRenderer()
    pcd_renderer.SetViewport(0.5, 0.5, 1, 1)
    pcd_renderer.SetBackground(1, 1, 1)
    for i in range(0, len(pcds)):
        vtk_points = vtk.vtkPoints()
        xyz = np.asarray(pcds[i].points)
        for j in range(0, xyz.shape[0]):
            vtk_points.InsertNextPoint(xyz[j][0], xyz[j][1], xyz[j][2])

        ply = vtk.vtkPolyData()
        ply.SetPoints(vtk_points)
        # ply.GetPointData().SetScalars([color[(3 * i) % len(color)],
        #                               color[(3 * i + 1) % len(color)],
        #                               color[(3 * i + 2) % len(color)]])
        
        sphere_source = vtk.vtkSphereSource()
        
        glyph = vtk.vtkGlyph3D()
        glyph.SetSourceConnection(sphere_source.GetOutputPort())
        glyph.SetInputData(ply)
        glyph.ScalingOff()
        glyph.Update()

        ply_mapper = vtk.vtkPolyDataMapper()
        ply_mapper.SetInputData(glyph.GetOutput())
        ply_mapper.Update()

        ply_actor = vtk.vtkActor()
        ply_actor.SetMapper(ply_mapper)
        ply_actor.GetProperty().SetColor(color[(3 * i) % len(color)],
                                         color[(3 * i + 1) % len(color)],
                                         color[(3 * i + 2) % len(color)])
        pcd_renderer.AddActor(ply_actor)
        pcd_screw_renderer.AddActor(ply_actor)
        
    # pcd_screw_renderer = vtk.vtkRenderer()
    # pcd_screw_renderer.SetViewport(0.5, 0, 1, 0.5)
    pcd_screw_renderer.SetBackground(1, 1, 1)
    for i in range(0, len(pcds)):
        vtk_points = vtk.vtkPoints()
        xyz = np.asarray(pcds[i].points)
        for j in range(0, xyz.shape[0]):
            vtk_points.InsertNextPoint(xyz[j][0], xyz[j][1], xyz[j][2])

        ply = vtk.vtkPolyData()
        ply.SetPoints(vtk_points)
        # ply.GetPointData().SetScalars([color[(3 * i) % len(color)],
        #                               color[(3 * i + 1) % len(color)],
        #                               color[(3 * i + 2) % len(color)]])
        
        sphere_source = vtk.vtkSphereSource()
        
        glyph = vtk.vtkGlyph3D()
        glyph.SetSourceConnection(sphere_source.GetOutputPort())
        glyph.SetInputData(ply)
        glyph.ScalingOff()
        glyph.Update()

        ply_mapper = vtk.vtkPolyDataMapper()
        ply_mapper.SetInputData(glyph.GetOutput())
        ply_mapper.Update()

        ply_actor = vtk.vtkActor()
        ply_actor.SetMapper(ply_mapper)
        ply_actor.GetProperty().SetColor(color[(3 * i) % len(color)],
                                         color[(3 * i + 1) % len(color)],
                                         color[(3 * i + 2) % len(color)])
        pcd_screw_renderer.AddActor(ply_actor)
    for path in path_info:
        path_dir = path[0]
        path_center = path[1]
        length1 = path[4]
        length2 = path[5]
        # screw_cylinder_actor = get_screw_cylinder_actor(path_center, path_dir)
        screw_actor, _ = get_screw_actor(path_center, path_dir, length1, length2)
        screw_line_actor = get_screw_line_actor(path_center, path_dir)
        pcd_screw_renderer.AddActor(screw_actor)
        pcd_screw_renderer.AddActor(screw_line_actor)
    
    render_window = vtk.vtkRenderWindow()
    render_window.AddRenderer(stl_renderer)
    render_window.AddRenderer(pcd_renderer)
    render_window.AddRenderer(stl_screw_renderer)
    render_window.AddRenderer(pcd_screw_renderer)
    rw_style = vtk.vtkInteractorStyleTrackballCamera()
    rw_interactor = vtk.vtkRenderWindowInteractor()
    rw_interactor.SetRenderWindow(render_window)
    rw_interactor.SetInteractorStyle(rw_style)
    render_window.Render()
    rw_interactor.Initialize()
    rw_interactor.Start()
    
    
def compare_screw_program4(stls, path_info1, path_info2, path_info3, path_info4, color=screw_setting.color):
    renderer1 = vtk.vtkRenderer()
    renderer2 = vtk.vtkRenderer()
    renderer3 = vtk.vtkRenderer()
    renderer4 = vtk.vtkRenderer()
    renderer1.SetViewport(0, 0.5, 0.5, 1)
    renderer2.SetViewport(0.5, 0.5, 1, 1)
    renderer3.SetViewport(0, 0, 0.5, 0.5)
    renderer4.SetViewport(0.5, 0, 1, 0.5)
    # for i in range(0, len(stls)):
    #     stl = stls[i]
    #     stl_ply_mapper = vtk.vtkPolyDataMapper()
    #     stl_ply_mapper.SetInputData(stl)
    #     stl_ply_mapper.Update()

    #     stl_ply_actor = vtk.vtkActor()
    #     stl_ply_actor.SetMapper(stl_ply_mapper)
    #     stl_ply_actor.GetProperty().SetColor(color[(3 * i) % len(color)], 
    #                                          color[(3 * i + 1) % len(color)], 
    #                                          color[(3 * i + 2) % len(color)])
    #     renderer1.AddActor(stl_ply_actor)
   
    for i in range(0, len(stls)):
        stl = stls[i]
        stl_ply_mapper = vtk.vtkPolyDataMapper()
        stl_ply_mapper.SetInputData(stl)
        stl_ply_mapper.Update()

        stl_ply_actor = vtk.vtkActor()
        stl_ply_actor.SetMapper(stl_ply_mapper)
        stl_ply_actor.GetProperty().SetColor(color[(3 * i) % len(color)],
                                             color[(3 * i + 1) % len(color)],
                                             color[(3 * i + 2) % len(color)])
        stl_ply_actor.GetProperty().SetOpacity(0.7)
        renderer1.AddActor(stl_ply_actor)
        renderer2.AddActor(stl_ply_actor)
        renderer3.AddActor(stl_ply_actor)
        renderer4.AddActor(stl_ply_actor)
        
    for i in range(len(path_info1)):
        path_dir = path_info1[i][0]
        path_center = path_info1[i][1]
        length1 = path_info1[i][4]
        length2 = path_info1[i][5]
        # screw_cylinder_actor = get_screw_cylinder_actor(path_center, path_dir)
        screw_actor, _ = get_screw_actor(path_center, path_dir, length1, length2)
        screw_line_actor = get_screw_line_actor(path_center, path_dir)
        renderer1.AddActor(screw_actor)
        renderer1.AddActor(screw_line_actor)
       
    for i in range(len(path_info2)):
        path_dir = path_info2[i][0]
        path_center = path_info2[i][1]
        length1 = path_info2[i][4]
        length2 = path_info2[i][5]
        # screw_cylinder_actor = get_screw_cylinder_actor(path_center, path_dir)
        screw_actor, _ = get_screw_actor(path_center, path_dir, length1, length2)
        screw_line_actor = get_screw_line_actor(path_center, path_dir)
        renderer2.AddActor(screw_actor)
        renderer2.AddActor(screw_line_actor)
        
    for i in range(len(path_info3)):
        path_dir = path_info3[i][0]
        path_center = path_info3[i][1]
        length1 = path_info3[i][4]
        length2 = path_info3[i][5]
        # screw_cylinder_actor = get_screw_cylinder_actor(path_center, path_dir)
        screw_actor, _ = get_screw_actor(path_center, path_dir, length1, length2)
        screw_line_actor = get_screw_line_actor(path_center, path_dir)
        renderer3.AddActor(screw_actor)
        renderer3.AddActor(screw_line_actor)
        
    for i in range(len(path_info4)):
        path_dir = path_info4[i][0]
        path_center = path_info4[i][1]
        length1 = path_info4[i][4]
        length2 = path_info4[i][5]
        # screw_cylinder_actor = get_screw_cylinder_actor(path_center, path_dir)
        screw_actor, _ = get_screw_actor(path_center, path_dir, length1, length2)
        screw_line_actor = get_screw_line_actor(path_center, path_dir)
        # p1 = path_center + path_dir*length1
        # p2 = path_center - path_dir*length2
        # s_a1 = get_sphere_actor(p1, 4)
        # s_a1.GetProperty().SetColor(1, 0, 0)
        # s_a2 = get_sphere_actor(p2, 4)
        # s_a2.GetProperty().SetColor(0, 1, 0)
        # renderer4.AddActor(s_a1)
        # renderer4.AddActor(s_a2)
        renderer4.AddActor(screw_actor)
        renderer4.AddActor(screw_line_actor)
 
    render_window = vtk.vtkRenderWindow()
    render_window.AddRenderer(renderer1)
    render_window.AddRenderer(renderer2)
    render_window.AddRenderer(renderer3)
    render_window.AddRenderer(renderer4)
    rw_style = vtk.vtkInteractorStyleTrackballCamera()
    rw_interactor = vtk.vtkRenderWindowInteractor()
    rw_interactor.SetRenderWindow(render_window)
    rw_interactor.SetInteractorStyle(rw_style)
    render_window.Render()
    rw_interactor.Initialize()
    rw_interactor.Start()
    
     
def compare_screw_program2(stls, path_info1, path_info2, color=screw_setting.color):
    renderer1 = vtk.vtkRenderer()
    renderer2 = vtk.vtkRenderer()
    renderer1.SetViewport(0, 0, 0.5, 1)
    renderer2.SetViewport(0.5, 0, 1, 1)
    # for i in range(0, len(stls)):
    #     stl = stls[i]
    #     stl_ply_mapper = vtk.vtkPolyDataMapper()
    #     stl_ply_mapper.SetInputData(stl)
    #     stl_ply_mapper.Update()

    #     stl_ply_actor = vtk.vtkActor()
    #     stl_ply_actor.SetMapper(stl_ply_mapper)
    #     stl_ply_actor.GetProperty().SetColor(color[(3 * i) % len(color)], 
    #                                          color[(3 * i + 1) % len(color)], 
    #                                          color[(3 * i + 2) % len(color)])
    #     renderer1.AddActor(stl_ply_actor)
   
    for i in range(0, len(stls)):
        stl = stls[i]
        stl_ply_mapper = vtk.vtkPolyDataMapper()
        stl_ply_mapper.SetInputData(stl)
        stl_ply_mapper.Update()

        stl_ply_actor = vtk.vtkActor()
        stl_ply_actor.SetMapper(stl_ply_mapper)
        stl_ply_actor.GetProperty().SetColor(color[(3 * i) % len(color)],
                                             color[(3 * i + 1) % len(color)],
                                             color[(3 * i + 2) % len(color)])
        stl_ply_actor.GetProperty().SetOpacity(0.7)
        renderer1.AddActor(stl_ply_actor)
        renderer2.AddActor(stl_ply_actor)
        
    for i in range(len(path_info1)):
        path_dir = path_info1[i][0]
        path_center = path_info1[i][1]
        length1 = path_info1[i][4]
        length2 = path_info1[i][5]
        # screw_cylinder_actor = get_screw_cylinder_actor(path_center, path_dir)
        screw_actor, _ = get_screw_actor(path_center, path_dir, length1, length2)
        screw_line_actor = get_screw_line_actor(path_center, path_dir)
        renderer1.AddActor(screw_actor)
        renderer1.AddActor(screw_line_actor)
       
    for i in range(len(path_info2)):
        path_dir = path_info2[i][0]
        path_center = path_info2[i][1]
        length1 = path_info2[i][4]
        length2 = path_info2[i][5]
        # screw_cylinder_actor = get_screw_cylinder_actor(path_center, path_dir)
        screw_actor, _ = get_screw_actor(path_center, path_dir, length1, length2)
        screw_line_actor = get_screw_line_actor(path_center, path_dir)
        # p1 = path_center + path_dir*length1
        # p2 = path_center - path_dir*length2
        # s_a1 = get_sphere_actor(p1, 4)
        # s_a1.GetProperty().SetColor(1, 0, 0)
        # s_a2 = get_sphere_actor(p2, 4)
        # s_a2.GetProperty().SetColor(0, 1, 0)
        # renderer2.AddActor(s_a1)
        # renderer2.AddActor(s_a2)
        renderer2.AddActor(screw_actor)
        renderer2.AddActor(screw_line_actor)
 
    render_window = vtk.vtkRenderWindow()
    render_window.AddRenderer(renderer1)
    render_window.AddRenderer(renderer2)
    rw_style = vtk.vtkInteractorStyleTrackballCamera()
    rw_interactor = vtk.vtkRenderWindowInteractor()
    rw_interactor.SetRenderWindow(render_window)
    rw_interactor.SetInteractorStyle(rw_style)
    render_window.Render()
    rw_interactor.Initialize()
    rw_interactor.Start()
    
    
def best_result_visualization(stls, path_info, color=screw_setting.color):
    renderer = vtk.vtkRenderer()
    renderer.SetBackground(1, 1, 1)
    for i in range(0, len(stls)):
        stl = stls[i]
        stl_ply_mapper = vtk.vtkPolyDataMapper()
        stl_ply_mapper.SetInputData(stl)
        stl_ply_mapper.Update()

        stl_ply_actor = vtk.vtkActor()
        stl_ply_actor.SetMapper(stl_ply_mapper)
        stl_ply_actor.GetProperty().SetColor(color[(3 * i) % len(color)],
                                             color[(3 * i + 1) % len(color)],
                                             color[(3 * i + 2) % len(color)])
        stl_ply_actor.GetProperty().SetOpacity(0.5)
        renderer.AddActor(stl_ply_actor)
        
    for i in range(len(path_info)):
        path_dir = path_info[i][0]
        path_center = path_info[i][1]
        length1 = path_info[i][4]
        length2 = path_info[i][5]
        # screw_cylinder_actor = get_screw_cylinder_actor(path_center, path_dir)
        screw_actor, _ = get_screw_actor(path_center, path_dir, length1, length2)
        # screw_line_actor = get_screw_line_actor(path_center, path_dir)
        renderer.AddActor(screw_actor)
        # renderer.AddActor(screw_line_actor)
 
    render_window = vtk.vtkRenderWindow()
    render_window.AddRenderer(renderer)
    rw_style = vtk.vtkInteractorStyleTrackballCamera()
    rw_interactor = vtk.vtkRenderWindowInteractor()
    rw_interactor.SetRenderWindow(render_window)
    rw_interactor.SetInteractorStyle(rw_style)
    render_window.Render()
    rw_interactor.Initialize()
    rw_interactor.Start()
    
    
def screws_vis(infos):
    renderer = vtk.vtkRenderer()
    renderer.SetBackground(1, 1, 1)
    for info in infos:
        renderer.AddActor(get_screw_actor(info[1], info[0], info[4], info[5])[0])
        # renderer.AddActor(get_screw_line_actor(info[1], info[0]))
    render_window = vtk.vtkRenderWindow()
    render_window.AddRenderer(renderer)
    rw_style = vtk.vtkInteractorStyleTrackballCamera()
    rw_interactor = vtk.vtkRenderWindowInteractor()
    rw_interactor.SetRenderWindow(render_window)
    rw_interactor.SetInteractorStyle(rw_style)
    render_window.Render()
    rw_interactor.Initialize()
    rw_interactor.Start()
    

def stl_pcd_visualization_with_path_by_vtk1(stls, pcds, color=screw_setting.color):
    stl_renderer = vtk.vtkRenderer()
    stl_renderer.SetViewport(0, 0.5, 0.5, 1)
    stl_renderer.SetBackground(1, 1, 1)
    pcd_screw_renderer = vtk.vtkRenderer()
    pcd_screw_renderer.SetViewport(0.5, 0, 1, 0.5)
    for i in range(0, len(stls)):
        stl = stls[i]
        stl_ply_mapper = vtk.vtkPolyDataMapper()
        stl_ply_mapper.SetInputData(stl)
        stl_ply_mapper.Update()

        stl_ply_actor = vtk.vtkActor()
        stl_ply_actor.SetMapper(stl_ply_mapper)
        stl_ply_actor.GetProperty().SetColor(color[(3 * i) % len(color)],
                                             color[(3 * i + 1) % len(color)],
                                             color[(3 * i + 2) % len(color)])
        stl_renderer.AddActor(stl_ply_actor)
    
    stl_screw_renderer = vtk.vtkRenderer()
    stl_screw_renderer.SetViewport(0, 0, 0.5, 0.5)
    stl_screw_renderer.SetBackground(1, 1, 1)
    for i in range(0, len(stls)):
        stl = stls[i]
        stl_ply_mapper = vtk.vtkPolyDataMapper()
        stl_ply_mapper.SetInputData(stl)
        stl_ply_mapper.Update()

        stl_ply_actor = vtk.vtkActor()
        stl_ply_actor.SetMapper(stl_ply_mapper)
        stl_ply_actor.GetProperty().SetColor(color[(3 * i) % len(color)],
                                             color[(3 * i + 1) % len(color)],
                                             color[(3 * i + 2) % len(color)])
        stl_ply_actor.GetProperty().SetOpacity(0.1)
        pcd_screw_renderer.AddActor(stl_ply_actor)
        stl_screw_renderer.AddActor(stl_ply_actor)

    pcd_renderer = vtk.vtkRenderer()
    pcd_renderer.SetViewport(0.5, 0.5, 1, 1)
    pcd_renderer.SetBackground(1, 1, 1)
    for i in range(0, len(pcds)):
        vtk_points = vtk.vtkPoints()
        xyz = np.asarray(pcds[i].points)
        for j in range(0, xyz.shape[0]):
            vtk_points.InsertNextPoint(xyz[j][0], xyz[j][1], xyz[j][2])

        ply = vtk.vtkPolyData()
        ply.SetPoints(vtk_points)
        # ply.GetPointData().SetScalars([color[(3 * i) % len(color)],
        #                               color[(3 * i + 1) % len(color)],
        #                               color[(3 * i + 2) % len(color)]])
        
        sphere_source = vtk.vtkSphereSource()
        
        glyph = vtk.vtkGlyph3D()
        glyph.SetSourceConnection(sphere_source.GetOutputPort())
        glyph.SetInputData(ply)
        glyph.ScalingOff()
        glyph.Update()

        ply_mapper = vtk.vtkPolyDataMapper()
        ply_mapper.SetInputData(glyph.GetOutput())
        ply_mapper.Update()

        ply_actor = vtk.vtkActor()
        ply_actor.SetMapper(ply_mapper)
        ply_actor.GetProperty().SetColor(color[(3 * i) % len(color)],
                                         color[(3 * i + 1) % len(color)],
                                         color[(3 * i + 2) % len(color)])
        pcd_renderer.AddActor(ply_actor)
        pcd_screw_renderer.AddActor(ply_actor)
        
    # pcd_screw_renderer = vtk.vtkRenderer()
    # pcd_screw_renderer.SetViewport(0.5, 0, 1, 0.5)
    pcd_screw_renderer.SetBackground(1, 1, 1)
    for i in range(0, len(pcds)):
        vtk_points = vtk.vtkPoints()
        for j in range(0, xyz.shape[0]):
            vtk_points.InsertNextPoint(xyz[j][0], xyz[j][1], xyz[j][2])

        ply = vtk.vtkPolyData()
        ply.SetPoints(vtk_points)
        # ply.GetPointData().SetScalars([color[(3 * i) % len(color)],
        #                               color[(3 * i + 1) % len(color)],
        #                               color[(3 * i + 2) % len(color)]])
        
        sphere_source = vtk.vtkSphereSource()
        
        glyph = vtk.vtkGlyph3D()
        glyph.SetSourceConnection(sphere_source.GetOutputPort())
        glyph.SetInputData(ply)
        glyph.ScalingOff()
        glyph.Update()

        ply_mapper = vtk.vtkPolyDataMapper()
        ply_mapper.SetInputData(glyph.GetOutput())
        ply_mapper.Update()

        ply_actor = vtk.vtkActor()
        ply_actor.SetMapper(ply_mapper)
        ply_actor.GetProperty().SetColor(color[(3 * i) % len(color)],
                                         color[(3 * i + 1) % len(color)],
                                         color[(3 * i + 2) % len(color)])
        pcd_screw_renderer.AddActor(ply_actor)
    
    render_window = vtk.vtkRenderWindow()
    render_window.AddRenderer(stl_renderer)
    render_window.AddRenderer(pcd_renderer)
    render_window.AddRenderer(stl_screw_renderer)
    render_window.AddRenderer(pcd_screw_renderer)
    rw_style = vtk.vtkInteractorStyleTrackballCamera()
    rw_interactor = vtk.vtkRenderWindowInteractor()
    rw_interactor.SetRenderWindow(render_window)
    rw_interactor.SetInteractorStyle(rw_style)
    render_window.Render()
    rw_interactor.Initialize()
    rw_interactor.Start()
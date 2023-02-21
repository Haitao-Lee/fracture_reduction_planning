# coding = utf-8
import vtkmodules.all as vtk
import numpy as np
import open3d as o3d


def points_visualization_by_vtk(PCDs, color):
    renderer = vtk.vtkRenderer()
    for i in range(0, len(PCDs)):
        vtk_points = vtk.vtkPoints()
        vtk_cells = vtk.vtkCellArray()
        xyz = np.asarray(PCDs[i].points)
        for j in range(0, xyz.shape[0]):
            vtk_cells.InsertNextCell(1)
            vtk_cells.InsertCellPoint(
                vtk_points.InsertNextPoint(xyz[j][0], xyz[j][1], xyz[j][2]))

        ply = vtk.vtkPolyData()
        ply.SetPoints(vtk_points)
        ply.SetVerts(vtk_cells)

        ply_mapper = vtk.vtkPolyDataMapper()
        ply_mapper.SetInputData(ply)
        ply_mapper.Update()

        ply_actor = vtk.vtkActor()
        ply_actor.SetMapper(ply_mapper)
        ply_actor.GetProperty().SetColor(color[(3 * i) % 21],
                                         color[(3 * i + 1) % 21],
                                         color[(3 * i + 2) % 21])
        renderer.AddActor(ply_actor)
    render_window = vtk.vtkRenderWindow()
    render_window.AddRenderer(renderer)
    rw_style = vtk.vtkInteractorStyleTrackballCamera()
    rw_interactor = vtk.vtkRenderWindowInteractor()
    rw_interactor.SetRenderWindow(render_window)
    rw_interactor.SetInteractorStyle(rw_style)
    render_window.Render()
    rw_interactor.Initialize()
    rw_interactor.Start()


def points_visualization_by_o3d(PCDs, color):
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    for i in range(0, len(PCDs)):
        source = PCDs[i]
        source.paint_uniform_color(
            [color[3 * i], color[3 * i - 1], color[3 * i - 2]])
        vis.add_geometry(source)
    vis.get_render_option().point_size = 0.01
    vis.get_render_option().background_color = np.asarray([0, 0, 0])
    vis.update_geometry()
    vis.poll_events()
    vis.update_renderer()
    vis.run()


def stl_visualization_by_vtk(result_stls, origin_stls, color):
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
        result_ply_actor.GetProperty().SetColor(color[(3 * i) % 21],
                                                color[(3 * i + 1) % 21],
                                                color[(3 * i + 2) % 21])
        result_renderer.AddActor(result_ply_actor)

        origin_stl = origin_stls[i]
        origin_ply_mapper = vtk.vtkPolyDataMapper()
        origin_ply_mapper.SetInputData(origin_stl)
        origin_ply_mapper.Update()

        origin_ply_actor = vtk.vtkActor()
        origin_ply_actor.SetMapper(origin_ply_mapper)
        origin_ply_actor.GetProperty().SetColor(color[(3 * i) % 21],
                                                color[(3 * i + 1) % 21],
                                                color[(3 * i + 2) % 21])
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


def point_ball_visualize(PCDs, ball_points, color, r):
    renderer = vtk.vtkRenderer()
    # renderer.SetBackground(1, 1, 1)
    for i in range(0, len(PCDs)):
        vtk_points = vtk.vtkPoints()
        vtk_cells = vtk.vtkCellArray()
        xyz = np.asarray(PCDs[i].points)
        for j in range(0, xyz.shape[0]):
            vtk_cells.InsertNextCell(1)
            vtk_cells.InsertCellPoint(
                vtk_points.InsertNextPoint(xyz[j][0], xyz[j][1], xyz[j][2]))

        ply = vtk.vtkPolyData()
        ply.SetPoints(vtk_points)
        ply.SetVerts(vtk_cells)

        ply_mapper = vtk.vtkPolyDataMapper()
        ply_mapper.SetInputData(ply)
        ply_mapper.Update()

        ply_actor = vtk.vtkActor()
        ply_actor.SetMapper(ply_mapper)
        ply_actor.GetProperty().SetColor(color[(3 * i) % 21],
                                         color[(3 * i + 1) % 21],
                                         color[(3 * i + 2) % 21])
        renderer.AddActor(ply_actor)

    for ball_point in ball_points:
        sphere = vtk.vtkSphereSource()
        sphere.SetCenter(ball_point)
        sphere.SetRadius(r)

        sp_mapper = vtk.vtkPolyDataMapper()
        sp_mapper.SetInputConnection(sphere.GetOutputPort())
        sp_mapper.Update()

        sp_actor = vtk.vtkActor()
        sp_actor.SetMapper(sp_mapper)
        sp_actor.GetProperty().SetColor(1, 1, 1)
        sp_actor.GetProperty().SetOpacity(0.5)
        renderer.AddActor(sp_actor)
    render_window = vtk.vtkRenderWindow()
    render_window.AddRenderer(renderer)
    rw_style = vtk.vtkInteractorStyleTrackballCamera()
    rw_interactor = vtk.vtkRenderWindowInteractor()
    rw_interactor.SetRenderWindow(render_window)
    rw_interactor.SetInteractorStyle(rw_style)
    render_window.Render()
    rw_interactor.Initialize()
    rw_interactor.Start()
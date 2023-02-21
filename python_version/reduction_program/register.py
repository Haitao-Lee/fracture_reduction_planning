# coding = utf-8
import open3d as o3d
import numpy as np
import vtkmodules.all as vtk
from tqdm import tqdm
import feature_extract
import fit_plane
import visualization
import reduction_setting
import scipy.spatial as spatial
import pcd_transform


# get matrix between 2 pointcloud
def get_matching_matrix_by_o3d_ICP(source, target, threshold):
    ref_points = np.asarray(target.points)
    tar_points = np.asarray(source.points)
    ref_center = np.mean(ref_points, axis=0)
    tar_center = np.mean(tar_points, axis=0)
    trans_init = [[1, 0, 0, ref_center[0] - tar_center[0]],
                  [0, 1, 0, ref_center[1] - tar_center[1]],
                  [0, 0, 1, ref_center[2] - tar_center[2]], [0, 0, 0, 1.0]]
    # trans_init = [[1, 0, 0, 0],
    #               [0, 1, 0, 0],
    #               [0, 0, 1, 0],
    #               [0, 0, 0, 1]]
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))
    print(reg_p2p)
    # reg_p2l = o3d.pipelines.registration.registration_icp(source, target, threshold, trans_init,
    #                                                       o3d.pipelines.registration.TransformationEstimationPointToPlane())
    # print(reg_p2l)
    return reg_p2p.transformation


def get_matching_matrix_by_vtk_ICP(source, target, threshold):
    ref_points = np.asarray(target.points)
    tar_points = np.asarray(source.points)
    ref_center = np.mean(ref_points, axis=0)
    tar_center = np.mean(tar_points, axis=0)
    # init_vector = [
    #     ref_center[0] - tar_center[0], ref_center[1] - tar_center[1],
    #     ref_center[2] - tar_center[2]
    # ]
    ref_vtk_points = vtk.vtkPoints()
    print("--converting numpy points to vtkPoints:")
    for i in tqdm(range(0, ref_points.shape[0])):
        ref_vtk_points.InsertNextPoint(ref_points[i, 0], ref_points[i, 1],
                                       ref_points[i, 2])

    tar_vtk_points = vtk.vtkPoints()
    print("--converting numpy points to vtkPoints:")
    for i in tqdm(range(0, tar_points.shape[0])):
        # tar_vtk_points.InsertNextPoint(tar_points[i, 0] + init_vector[0],
        #                                tar_points[i, 1] + init_vector[1],
        #                                tar_points[i, 2] + init_vector[2])
        tar_vtk_points.InsertNextPoint(tar_points[i, 0], tar_points[i, 1],
                                       tar_points[i, 2])

    source = vtk.vtkPolyData()
    source.SetPoints(tar_vtk_points)
    # 目标数据
    target = vtk.vtkPolyData()
    target.SetPoints(ref_vtk_points)

    sourceGlypyFilter = vtk.vtkVertexGlyphFilter()
    sourceGlypyFilter.SetInputData(source)
    sourceGlypyFilter.Update()

    targetGlyphFilter = vtk.vtkVertexGlyphFilter()
    targetGlyphFilter.SetInputData(target)
    targetGlyphFilter.Update()

    icp_tf = vtk.vtkIterativeClosestPointTransform()
    icp_tf.SetSource(sourceGlypyFilter.GetOutput())
    icp_tf.SetTarget(targetGlyphFilter.GetOutput())
    icp_tf.GetLandmarkTransform().SetModeToRigidBody()
    icp_tf.SetMaximumNumberOfIterations(threshold)
    icp_tf.SetMeanDistanceModeToAbsoluteValue()
    icp_tf.CheckMeanDistanceOn()
    icp_tf.Modified()
    icp_tf.Update()
    tf_mtx = vtk.vtkMatrix4x4()
    tf_mtx = icp_tf.GetMatrix()
    result = np.zeros([4, 4])
    for i in range(0, 4):
        for j in range(0, 4):
            result[i, j] = tf_mtx.GetElement(i, j)
    result = np.asarray(result)
    # result[0, 3] = result[0, 3] + init_vector[0]
    # result[1, 3] = result[1, 3] + init_vector[1]
    # result[2, 3] = result[2, 3] + init_vector[2]
    return result


def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature,
                                             max_nn=100))
    return pcd_down, pcd_fpfh


def prepare_dataset(source, target, voxel_size):
    trans_init = np.asarray([[0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0],
                             [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
    source.transform(trans_init)
    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source_down, target_down, source_fpfh, target_fpfh


def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    return result


def get_matching_matrix_by_local_feature(source, target, threshold):
    ref_points = target.points
    tar_points = source.points
    ref_points
    tar_points
    threshold


def transform_stl(stl, mtx):
    tf = vtk.vtkTransform()
    tf_mtx = vtk.vtkMatrix4x4()
    tf_mtx.DeepCopy(mtx.flatten()[:])
    tf.SetMatrix(tf_mtx)

    tf_stl = vtk.vtkTransformFilter()
    tf_stl.SetInputData(stl)
    tf_stl.SetTransform(tf)
    tf_stl.Update()
    return tf_stl.GetOutput()


def physical_register(pcd1, pcd2, radius, threshold, iteration):
    points1 = np.asarray(pcd1.points)
    points2 = np.asarray(pcd2.points)
    normals2 = np.asarray(pcd2.normals)
    center1 = np.mean(points1, axis=0)
    center2 = np.mean(points2, axis=0)
    plane_normal, plane_center = fit_plane.fit_plane_by_svm(
        points1, points2, center1 - center2, threshold)
    # plane_normal1 = fit_plane.fit_plane_by_norm2(points1, center1 - center2)[1]
    # plane_normal2 = fit_plane.fit_plane_by_norm2(points2, center2 - center1)[1]
    ###################################################################################################
    renderer = vtk.vtkRenderer()

    vtk_points = vtk.vtkPoints()
    vtk_cells = vtk.vtkCellArray()

    for i in range(0, points1.shape[0]):
        vtk_cells.InsertNextCell(1)
        vtk_cells.InsertCellPoint(
            vtk_points.InsertNextPoint(points1[i][0], points1[i][1],
                                       points1[i][2]))
    for i in range(0, points2.shape[0]):
        vtk_cells.InsertNextCell(1)
        vtk_cells.InsertCellPoint(
            vtk_points.InsertNextPoint(points2[i][0], points2[i][1],
                                       points2[i][2]))
    ply = vtk.vtkPolyData()
    ply.SetPoints(vtk_points)
    ply.SetVerts(vtk_cells)

    ply_mapper = vtk.vtkPolyDataMapper()
    ply_mapper.SetInputData(ply)
    ply_mapper.Update()

    ply_actor = vtk.vtkActor()
    ply_actor.SetMapper(ply_mapper)
    ply_actor.GetProperty().SetColor(1, 0, 0)
    renderer.AddActor(ply_actor)

    planeSource = vtk.vtkPlaneSource()
    planeSource.SetCenter(plane_center)
    planeSource.SetNormal(plane_normal)
    plane_p1 = [
        plane_center[0] + plane_normal[2] * 50, plane_center[1],
        plane_center[2] - plane_normal[0] * 50
    ]
    plane_p2 = [
        plane_center[0] - plane_normal[1] * 50,
        plane_center[1] + plane_normal[0] * 50, plane_center[2]
    ]
    planeSource.SetPoint1(plane_p1)
    planeSource.SetPoint2(plane_p2)
    planeSource.Update()

    polydataPlane = vtk.vtkPolyData()
    polydataPlane = planeSource.GetOutput()
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputData(polydataPlane)

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(0, 1, 0)
    renderer.AddActor(actor)

    render_window = vtk.vtkRenderWindow()
    render_window.AddRenderer(renderer)
    rw_style = vtk.vtkInteractorStyleTrackballCamera()
    rw_interactor = vtk.vtkRenderWindowInteractor()
    rw_interactor.SetRenderWindow(render_window)
    rw_interactor.SetInteractorStyle(rw_style)
    render_window.Render()
    rw_interactor.Initialize()
    rw_interactor.Start()

    # concave_points1 = feature_extract.get_valley_points_by_gravity(points1, -plane_normal, radius/2)
    # concave_points2 = feature_extract.get_valley_points_by_gravity(points2, plane_normal, radius/2)
    # cc_p1 = feature_extract.get_cvx_ccv_points(pcd1, radius)
    # cc_p2 = feature_extract.get_cvx_ccv_points(pcd2, radius)
    pcds1, _, _ = feature_extract.cluster_by_gravity(pcd1, plane_normal, radius)
    pcds2, _, _ = feature_extract.cluster_by_gravity(pcd2, -plane_normal, radius)
    visualization.point_ball_visualize(pcds1 + pcds2,
                                       [],
                                       reduction_setting.PCD_color, radius / 3)
    pcds1, convex_points1, _ = feature_extract.ud_cluster_by_gravity(pcd1, plane_normal, radius)
    pcds2, convex_points2, _ = feature_extract.ud_cluster_by_gravity(pcd2, -plane_normal, radius)
    visualization.point_ball_visualize(pcds1 + pcds2,
                                       [],
                                       reduction_setting.PCD_color, radius / 3)
    #####################################################################################################
    # tree = spatial.KDTree(np.asarray(cc_p1))
    tree = spatial.KDTree(points1)
    f_pull = 1
    for i in range(0, iteration):
        points_center_vec = center1 - center2
        center_norm = np.linalg.norm(points_center_vec)
        points_center_vec = points_center_vec / center_norm
        convex_points2 = np.asarray(convex_points2)
        pull_center = np.mean(convex_points2, axis=0)
        _, points_indices = tree.query(convex_points2, 1, workers=-1)
        points_indices = np.asarray(points_indices)
        # print(points_indices)
        valley_dual_points = points1[points_indices]
        repul_vec = convex_points2 - valley_dual_points
        distance = np.linalg.norm(repul_vec, axis=1)
        distance = np.c_[distance, distance, distance]
        # similar to universal gravitation
        repulsion = repul_vec / distance / (distance * distance)
        repulsion_norm = np.linalg.norm(repulsion, axis=1)
        repul_center = np.sum(
            convex_points2 *
            np.c_[repulsion_norm, repulsion_norm, repulsion_norm] /
            np.sum(repulsion_norm, axis=0),
            axis=0)
        repulsion = repulsion / np.sum(repulsion_norm, axis=0) * f_pull
        f_repul = np.sum(repulsion, axis=0)
        f = f_repul + f_pull * points_center_vec
        f_norm = np.linalg.norm(f)
        f = f / f_norm * center_norm / 3
        # pointcloud transform
        # specify rotate radian and translate distance
        rotate_vec = np.cross(repul_center - pull_center, f_repul)
        rotate_norm = np.linalg.norm(rotate_vec)
        rotate_vec = rotate_vec / rotate_norm
        R = pcd_transform.rotate_mat(rotate_vec, rotate_norm / 20)
        rotate_center = pull_center
        points2 = points2 - rotate_center
        convex_points2 = convex_points2 - np.expand_dims(
            rotate_center, 0).repeat(convex_points2.shape[0], axis=0)
        points2 = np.dot(R, points2.T).T
        normals2 = np.dot(R, normals2.T).T
        convex_points2 = np.dot(R, convex_points2.T).T
        points2 = points2 + np.expand_dims(rotate_center, 0).repeat(
            points2.shape[0], axis=0)
        convex_points2 = convex_points2 + np.expand_dims(
            rotate_center, 0).repeat(convex_points2.shape[0], axis=0)
        points2 = points2 + np.expand_dims(f, 0).repeat(points2.shape[0],
                                                        axis=0)
        convex_points2 = convex_points2 + np.expand_dims(f, 0).repeat(
            convex_points2.shape[0], axis=0)
        center2 = np.mean(points2, axis=0)
        # plane_normal, plane_center = fit_plane.fit_plane_by_svm(points1, points2, center1 - center2, threshold)
        # valley_points2 = feature_extract.get_valley_points_by_gravity(points2, -plane_normal, radius)
        # pcds2, valley_points2 = feature_extract.cluster_by_gravity(points2, -plane_normal, radius)
        convex_points2 = convex_points2.tolist()
        tmp_pcd = o3d.geometry.PointCloud()
        tmp_pcd.points = o3d.utility.Vector3dVector(points2)
        tmp_pcd.normals = o3d.utility.Vector3dVector(normals2)
        pcds2, _, _ = feature_extract.cluster_by_gravity(
            tmp_pcd, -plane_normal, radius)
        visualization.point_ball_visualize(pcds1 + pcds2, convex_points1 + convex_points2, reduction_setting.PCD_color, radius / 3)
    #####################################################################################################
    return 0

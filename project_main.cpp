#include <data_processing.h>
#include <vtkSmartPointer.h>
#include <vtkMapper.h>
#include <vtkPolyDataMapper.h>
#include <vtkPointPicker.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkActor.h>
#include <iostream>
#include <vtkInteractorStyleTrackballCamera.h>
#include <global_setting.h>
#include <vtkCurvatures.h>
#include <vtkLookupTable.h>
#include <vtkDataArray.h>
#include <vtkPointData.h>
#include <vtkScalarBarActor.h>
#include <vtkPointCloudFilter.h>
#include <bone_descriptor.h>
#include <vtkPoints.h>
#include <vtkPolyDataNormals.h>
#include <vtkFloatArray.h>
#include <vtkImageShrink3D.h>
#include <vtkCleanPolyData.h>
#include <vtkProperty.h>
#include <vtkMaskPoints.h>
#include <vtkArrowSource.h>
#include <vtkGlyph3D.h>
#include <vtkPlane.h>
#include <vtkCutter.h>
#include <vtkDataArray.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkPolyDataConnectivityFilter.h>
#include <vtkStripper.h>
#include <vtkSphereSource.h>
#include <vtkOutputWindow.h>
#include <itkOutputWindow.h>
#include <cutter_descriptor.h>
#include <vtkLandmarkTransform.h>
#include <vtkDataSetMapper.h>
#include <vtkKdTree.h>
#include <vtkIdList.h>
#include <vtkIdTypeArray.h>
#include <time.h>
#include <ctime>
#include <vtkLineSource.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/features/eigen.h>
#include <pcl/features/feature.h>
#include <pcl/features/normal_3d.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/features/boundary.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkAutoInit.h> 
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);


int main()
{
	vtkOutputWindow::SetGlobalWarningDisplay(0);//不弹出vtkOutputWindow窗口
	itk::OutputWindow::SetGlobalWarningDisplay(0);;//不弹出itkOutputWindow窗口
	//储存文件名的容器
	std::vector<std::string> stl_file_names;
	GetAllFormatFiles(global_setting::file_name, stl_file_names, global_setting::format);
	std::vector<vtkSmartPointer<vtkSTLReader>> STL_readers;//(stl_file_names.size(), vtkSmartPointer<vtkSTLReader>::New());
	std::vector<vtkSmartPointer<vtkPolyData>> STLs;//(stl_file_names.size(), vtkSmartPointer<vtkPolyData>::New());
    //用循环的方式pushback防止元素指向同一个指针
	for (int i = 0;i < stl_file_names.size();i++)
	{
		STL_readers.push_back(vtkSmartPointer<vtkSTLReader>::New());
		STLs.push_back(vtkSmartPointer<vtkPolyData>::New());
		//fragments_normals.push_back(vtkSmartPointer<vtkDataArray>::New());
	}
	//转化为polyData
	transformSTL2PolyData(stl_file_names, STLs, STL_readers);
	for (auto iter = STLs.begin();iter < STLs.end();iter++)
	{
		double volum = compute_volum(*iter);
		if (volum < global_setting::volum_threshold)
			STLs.erase(iter);
	}
	std::vector<vtkSmartPointer<vtkPoints>> fragments_points;//(STLs.size());
	std::vector<vtkSmartPointer<vtkPolyData>> downSampleSTLs;
	std::vector<vtkSmartPointer<vtkPolyData>> contour_downSampleSTLs;
	std::vector<vtkSmartPointer<vtkDataArray>> fragments_normals(STLs.size());
	std::vector<std::vector<std::vector<vtkSmartPointer<vtkPolyData>>>>  cutter_pd(STLs.size());
	std::vector<std::vector<std::vector<std::vector<double>>>>  cutter_pd_dist(STLs.size());
	std::vector<double> distance_pd(STLs.size());
	for (int i = 0;i < STLs.size();i++)
	{
		fragments_points.push_back(vtkSmartPointer<vtkPoints>::New());
		downSampleSTLs.push_back(vtkSmartPointer<vtkPolyData>::New());
		contour_downSampleSTLs.push_back(vtkSmartPointer<vtkPolyData>::New());
		//fragments_normals.push_back(vtkSmartPointer<vtkDataArray>::New());
	}

	//downSample(STLs, downSampleSTLs,fragments_points, global_setting::tolerance);
	//downSample(STLs, contour_downSampleSTLs, fragments_points, global_setting::tolerance/5);
	////合并
	//vtkSmartPointer<vtkPolyData> pelvic_polydata =
	//	vtkSmartPointer<vtkPolyData>::New();
	//mergeSTLs(pelvic_polydata, STLs);
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//vtkSmartPointer<vtkRenderWindow> renderWindow =
	//	vtkSmartPointer<vtkRenderWindow>::New();

	//vtkSmartPointer<vtkPointPicker> pointPicker = 
	//	vtkSmartPointer<vtkPointPicker>::New();

	//vtkSmartPointer< vtkInteractorStyleTrackballCamera> render_style =
	//	vtkSmartPointer< vtkInteractorStyleTrackballCamera>::New();

	//vtkSmartPointer<vtkRenderWindowInteractor> render_inter =
	//	vtkSmartPointer<vtkRenderWindowInteractor>::New();
	//render_inter->SetPicker(pointPicker);
	//render_inter->SetRenderWindow(renderWindow);
	//render_inter->SetInteractorStyle(render_style);

	//vtkSmartPointer<vtkPolyDataMapper> origin_mapper =
	//	vtkSmartPointer<vtkPolyDataMapper>::New();
	//origin_mapper->SetInputData(pelvic_polydata);
	//origin_mapper->Update();

	//vtkSmartPointer<vtkActor> origin_actor =
	//	vtkSmartPointer<vtkActor>::New();
	//origin_actor->SetMapper(origin_mapper);
	//origin_actor->GetProperty()->SetColor(1, 0, 0);

	//vtkSmartPointer<vtkRenderer> origin_renderer =
	//	vtkSmartPointer<vtkRenderer>::New();
	//origin_renderer->SetRenderWindow(renderWindow);
	//origin_renderer->AddActor(origin_actor);
	//origin_renderer->SetViewport(0,0.4,0.5,1);
	//origin_renderer->SetBackground(1, 1, 1);

	//vtkSmartPointer<vtkRenderer> result_renderer =
	//	vtkSmartPointer<vtkRenderer>::New();
	//result_renderer->SetRenderWindow(renderWindow);
	//result_renderer->SetViewport(0.5, 0.4, 1, 1);
	//result_renderer->SetBackground(1, 1, 1);
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//3和5做实验
	pcl::PointCloud<pcl::PointXYZ>::Ptr pelvic_cloud2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::vtkPolyDataToPointCloud(STLs.at(2), *pelvic_cloud2);
	std::cout << "points sieze is:" << pelvic_cloud2->size() << std::endl;
	pcl::PointCloud<pcl::Normal>::Ptr pelvic_normals2(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Boundary> pelvic_boundaries2;
	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundary_est2;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr pelvic_tree2(new pcl::search::KdTree<pcl::PointXYZ>());

	pcl::KdTreeFLANN<pcl::PointXYZ> pelvic_kdtree2;  //创建一个快速k近邻查询,查询的时候若该点在点云中，则第一个近邻点是其本身
	pelvic_kdtree2.setInputCloud(pelvic_cloud2);
	int k = 2;
	float pelvic_everagedistance2 = 0;
	for (int i = 0; i < pelvic_cloud2->size() / 2;i++)
	{
		std::vector<int> nnh;
		std::vector<float> squaredistance;
		//  pcl::PointXYZ p;
		//   p = cloud->points[i];
		pelvic_kdtree2.nearestKSearch(pelvic_cloud2->points[i], k, nnh, squaredistance);
		pelvic_everagedistance2 += sqrt(squaredistance[1]);
		//   cout<<everagedistance<<endl;
	}
	pelvic_everagedistance2 = pelvic_everagedistance2 / (pelvic_cloud2->size() / 2);
	cout << "everage distance is : " << pelvic_everagedistance2 << endl;

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst;  //其中pcl::PointXYZ表示输入类型数据，pcl::Normal表示输出类型,且pcl::Normal前三项是法向，最后一项是曲率
	normEst.setInputCloud(pelvic_cloud2);
	normEst.setSearchMethod(pelvic_tree2);
	// normEst.setRadiusSearch(2);  //法向估计的半径
	normEst.setKSearch(5000);  //法向估计的点数
	normEst.compute(*pelvic_normals2);
	cout << "normal size is " << pelvic_normals2->size() << endl;

	//normal_est.setViewPoint(0,0,0); //这个应该会使法向一致
	boundary_est2.setInputCloud(pelvic_cloud2);
	boundary_est2.setInputNormals(pelvic_normals2);
	//  est.setAngleThreshold(90);
	//   est.setSearchMethod (pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));
	boundary_est2.setSearchMethod(pelvic_tree2);
	boundary_est2.setKSearch(70);  //一般这里的数值越高，最终边界识别的精度越好
	//  est.setRadiusSearch(everagedistance);  //搜索半径
	boundary_est2.compute(pelvic_boundaries2);

	//  pcl::PointCloud<pcl::PointXYZ> boundPoints;
	pcl::PointCloud<pcl::PointXYZ>::Ptr boundPoints2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr noBoundPoints2(new pcl::PointCloud<pcl::PointXYZ>);
	int countBoundaries2 = 0;
	//std::cout << pelvic_cloud2->size() << std::endl;
	for (int i = 0; i < pelvic_cloud2->size(); i++) {
		uint8_t x = (pelvic_boundaries2.points[i].boundary_point);
		int a = static_cast<int>(x); //该函数的功能是强制类型转换
		if (a == 1)
		{
			//  boundPoints.push_back(cloud->points[i]);
			boundPoints2->push_back(pelvic_cloud2->points[i]);
			countBoundaries2++;
		}
		else
			noBoundPoints2->push_back(pelvic_cloud2->points[i]);
	}
	std::cout << "boudary size is：" << countBoundaries2 << std::endl;


	pcl::PointCloud<pcl::PointXYZ>::Ptr pelvic_cloud4(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::vtkPolyDataToPointCloud(STLs.at(4), *pelvic_cloud4);
	std::cout << "points sieze is:" << pelvic_cloud4->size() << std::endl;
	pcl::PointCloud<pcl::Normal>::Ptr pelvic_normals4(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Boundary> pelvic_boundaries4;
	pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundary_est4;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr pelvic_tree4(new pcl::search::KdTree<pcl::PointXYZ>());

	pcl::KdTreeFLANN<pcl::PointXYZ> pelvic_kdtree4;  //创建一个快速k近邻查询,查询的时候若该点在点云中，则第一个近邻点是其本身
	pelvic_kdtree4.setInputCloud(pelvic_cloud4);
	float pelvic_everagedistance4 = 0;
	for (int i = 0; i < pelvic_cloud4->size() / 2;i++)
	{
		std::vector<int> nnh;
		std::vector<float> squaredistance;
		//  pcl::PointXYZ p;
		//   p = cloud->points[i];
		pelvic_kdtree2.nearestKSearch(pelvic_cloud4->points[i], k, nnh, squaredistance);
		pelvic_everagedistance4 += sqrt(squaredistance[1]);
		//   cout<<everagedistance<<endl;
	}
	pelvic_everagedistance4 = pelvic_everagedistance4 / (pelvic_cloud4->size() / 2);
	cout << "everage distance is : " << pelvic_everagedistance4 << endl;

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst4;  //其中pcl::PointXYZ表示输入类型数据，pcl::Normal表示输出类型,且pcl::Normal前三项是法向，最后一项是曲率
	normEst4.setInputCloud(pelvic_cloud4);
	normEst4.setSearchMethod(pelvic_tree4);
	// normEst.setRadiusSearch(2);  //法向估计的半径
	normEst4.setKSearch(5000);  //法向估计的点数
	normEst4.compute(*pelvic_normals4);
	cout << "normal size is " << pelvic_normals4->size() << endl;

	//normal_est.setViewPoint(0,0,0); //这个应该会使法向一致
	boundary_est4.setInputCloud(pelvic_cloud4);
	boundary_est4.setInputNormals(pelvic_normals4);
	//  est.setAngleThreshold(90);
	//   est.setSearchMethod (pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));
	boundary_est4.setSearchMethod(pelvic_tree4);
	boundary_est4.setKSearch(70);  //一般这里的数值越高，最终边界识别的精度越好
	//  est.setRadiusSearch(everagedistance);  //搜索半径
	boundary_est4.compute(pelvic_boundaries4);

	//  pcl::PointCloud<pcl::PointXYZ> boundPoints;
	pcl::PointCloud<pcl::PointXYZ>::Ptr boundPoints4(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr noBoundPoints4(new pcl::PointCloud<pcl::PointXYZ>);
	int countBoundaries4 = 0;
	//std::cout << pelvic_cloud2->size() << std::endl;
	for (int i = 0; i < pelvic_cloud4->size(); i++) {
		uint8_t x = (pelvic_boundaries4.points[i].boundary_point);
		int a = static_cast<int>(x); //该函数的功能是强制类型转换
		if (a == 1)
		{
			//  boundPoints.push_back(cloud->points[i]);
			boundPoints4->push_back(pelvic_cloud4->points[i]);
			countBoundaries4++;
		}
		else
			noBoundPoints4->push_back(pelvic_cloud4->points[i]);
	}
	std::cout << "boudary size is：" << countBoundaries4 << std::endl;
	//  pcl::io::savePCDFileASCII("boudary.pcd",boundPoints);



	// render
	vtkSmartPointer<vtkPoints> no_boundary_points2 =
		vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkCellArray> no_boundary_cell2 =
		vtkSmartPointer<vtkCellArray>::New();
	for (int i = 0;i < noBoundPoints2->size();i++)
	{
		vtkIdType pid = no_boundary_points2->InsertNextPoint(noBoundPoints2->at(i).x, noBoundPoints2->at(i).y, noBoundPoints2->at(i).z);
		no_boundary_cell2->InsertNextCell(1, &pid);
	}

	vtkSmartPointer<vtkPolyData> no_boundarypelvic2 =
		vtkSmartPointer<vtkPolyData>::New();
	no_boundarypelvic2->SetPoints(no_boundary_points2);
	no_boundarypelvic2->SetVerts(no_boundary_cell2);

	vtkSmartPointer<vtkPolyDataMapper> no_boundary_mapper2 =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	no_boundary_mapper2->SetInputData(no_boundarypelvic2);
	no_boundary_mapper2->Update();

	vtkSmartPointer<vtkActor> no_boundary_actor2 =
		vtkSmartPointer<vtkActor>::New();
	no_boundary_actor2->SetMapper(no_boundary_mapper2);
	no_boundary_actor2->GetProperty()->SetColor(0, 1, 0);


	vtkSmartPointer<vtkPoints> boundary_points2 =
		vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkCellArray> boundary_cell2 =
		vtkSmartPointer<vtkCellArray>::New();
	for (int i = 0;i < boundPoints2->size();i++)
	{
		vtkIdType pid = boundary_points2->InsertNextPoint(boundPoints2->at(i).x, boundPoints2->at(i).y, boundPoints2->at(i).z);
		boundary_cell2->InsertNextCell(1, &pid);
	}

	vtkSmartPointer<vtkPolyData> pelvic2_boundary =
		vtkSmartPointer<vtkPolyData>::New();
	pelvic2_boundary->SetPoints(boundary_points2);
	pelvic2_boundary->SetVerts(boundary_cell2);

	vtkSmartPointer<vtkPolyDataMapper> boundary_mapper2 =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	boundary_mapper2->SetInputData(pelvic2_boundary);
	boundary_mapper2->Update();

	vtkSmartPointer<vtkActor> boundary_actor2 =
		vtkSmartPointer<vtkActor>::New();
	boundary_actor2->SetMapper(boundary_mapper2);
	boundary_actor2->GetProperty()->SetColor(1, 0, 0);

	vtkSmartPointer<vtkPoints> no_boundary_points4 =
		vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkCellArray> no_boundary_cell4 =
		vtkSmartPointer<vtkCellArray>::New();
	for (int i = 0;i < noBoundPoints4->size();i++)
	{
		vtkIdType pid = no_boundary_points4->InsertNextPoint(noBoundPoints4->at(i).x, noBoundPoints4->at(i).y, noBoundPoints4->at(i).z);
		no_boundary_cell4->InsertNextCell(1, &pid);
	}

	vtkSmartPointer<vtkPolyData> no_boundarypelvic4 =
		vtkSmartPointer<vtkPolyData>::New();
	no_boundarypelvic4->SetPoints(no_boundary_points4);
	no_boundarypelvic4->SetVerts(no_boundary_cell4);

	vtkSmartPointer<vtkPolyDataMapper> no_boundary_mapper4 =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	no_boundary_mapper4->SetInputData(no_boundarypelvic4);
	no_boundary_mapper4->Update();

	vtkSmartPointer<vtkActor> no_boundary_actor4 =
		vtkSmartPointer<vtkActor>::New();
	no_boundary_actor4->SetMapper(no_boundary_mapper4);
	no_boundary_actor4->GetProperty()->SetColor(1, 0, 0);

	vtkSmartPointer<vtkPoints> boundary_points4 =
		vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkCellArray> boundary_cell4 =
		vtkSmartPointer<vtkCellArray>::New();
	for (int i = 0;i < boundPoints4->size();i++)
	{
		vtkIdType pid = boundary_points4->InsertNextPoint(boundPoints4->at(i).x, boundPoints4->at(i).y, boundPoints4->at(i).z);
		boundary_cell4->InsertNextCell(1, &pid);
	}

	vtkSmartPointer<vtkPolyData> pelvic4_boundary =
		vtkSmartPointer<vtkPolyData>::New();
	pelvic4_boundary->SetPoints(boundary_points4);
	pelvic4_boundary->SetVerts(boundary_cell4);

	vtkSmartPointer<vtkPolyDataMapper> boundary_mapper4 =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	boundary_mapper4->SetInputData(pelvic4_boundary);
	boundary_mapper4->Update();

	vtkSmartPointer<vtkActor> boundary_actor4 =
		vtkSmartPointer<vtkActor>::New();
	boundary_actor4->SetMapper(boundary_mapper4);
	boundary_actor4->GetProperty()->SetColor(0, 1, 0);

	//Render and interact
	vtkSmartPointer<vtkRenderer> boundary_renderer =
		vtkSmartPointer<vtkRenderer>::New();
	boundary_renderer->SetViewport(0.5, 0, 1, 1);
	boundary_renderer->SetBackground(0, 0, 0);
	boundary_renderer->AddActor(boundary_actor2);
	boundary_renderer->AddActor(boundary_actor4);

	vtkSmartPointer<vtkRenderer> origin_renderer =
		vtkSmartPointer<vtkRenderer>::New();
	origin_renderer->AddActor(no_boundary_actor2);
	origin_renderer->AddActor(boundary_actor2);
	origin_renderer->AddActor(no_boundary_actor4);
	origin_renderer->AddActor(boundary_actor4);
	origin_renderer->SetViewport(0, 0, 0.5, 1);
	origin_renderer->SetBackground(0, 0, 0);

	vtkSmartPointer<vtkRenderWindow> renderWindow =
		vtkSmartPointer<vtkRenderWindow>::New();

	vtkSmartPointer<vtkPointPicker> pointPicker =
		vtkSmartPointer<vtkPointPicker>::New();

	vtkSmartPointer< vtkInteractorStyleTrackballCamera> render_style =
		vtkSmartPointer< vtkInteractorStyleTrackballCamera>::New();

	vtkSmartPointer<vtkRenderWindowInteractor> render_inter =
		vtkSmartPointer<vtkRenderWindowInteractor>::New();
	render_inter->SetPicker(pointPicker);
	render_inter->SetRenderWindow(renderWindow);
	render_inter->SetInteractorStyle(render_style);
	renderWindow->AddRenderer(origin_renderer);
	renderWindow->AddRenderer(boundary_renderer);
	renderWindow->Render();
	//renderWindow2->Render();
	render_inter->Initialize();
	boundary_renderer->ResetCamera();
	render_inter->Start();
}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//vtkSmartPointer<vtkPolyData> ref_PD = STLs.at(0);
	//vtkSmartPointer<vtkPolyData> ref_contour_PD = contour_downSampleSTLs.at(0);
	//vtkSmartPointer<vtkPolyData> ref_downSample_PD = downSampleSTLs.at(0);
	////std::vector<vtkSmartPointer<vtkPolyData>> tar_downsample_STLs;
	////tar_downsample_STLs.assign((downSampleSTLs.begin()++), downSampleSTLs.end());
	//std::vector<size_t> distance_idx(STLs.size());
	//std::vector<int> tar_index(STLs.size(),0);
	//std::vector<vtkSmartPointer<vtkMatrix4x4>> tf_matrix(STLs.size());
	//clock_t start = clock();
	//for (size_t i = 0;i < 1;++i)//downSampleSTLs.size()-1
	//{
	//	vtkSmartPointer<vtkPolyDataNormals> ref_PD_normals =
	//		vtkSmartPointer<vtkPolyDataNormals>::New();
	//	ref_PD_normals->SetInputData(ref_PD);
	//	ref_PD_normals->ComputePointNormalsOn();
	//	ref_PD_normals->ComputeCellNormalsOff();
	//	ref_PD_normals->SetAutoOrientNormals(1);
	//	ref_PD_normals->SetConsistency(1);
	//	ref_PD_normals->SetSplitting(1);
	//	ref_PD_normals->Update();
	//	vtkSmartPointer<vtkPoints> ref_points = ref_downSample_PD->GetPoints();
	//	vtkSmartPointer<vtkKdTree> ref_kd_tree =
	//		vtkSmartPointer<vtkKdTree>::New();
	//	ref_kd_tree->BuildLocatorFromPoints(ref_PD->GetPoints());
	//	ref_kd_tree->Update();
	//	vtkSmartPointer<vtkDoubleArray> ref_blur_normals = getBlurNormal(ref_kd_tree, ref_PD_normals->GetOutput()->GetPointData()->GetNormals(), ref_points, global_setting::kdTree_num);
	//	std::vector<vtkSmartPointer<vtkPolyData>> ref_cutter_PD(ref_points->GetNumberOfPoints());
	//	std::vector<std::vector<double>> ref_cutter_dist(ref_points->GetNumberOfPoints());
	//	std::vector<cutter_descriptor*> ref_cutter_des(ref_points->GetNumberOfPoints());
	//	//std::vector<cutter_descriptor>(int(2 * global_setting::offset_num + 1));
	//	std::cout << ref_points->GetNumberOfPoints() << std::endl;
	//	clock_t end = clock();
	//	std::cout << "It took " << (end - start) / CLOCKS_PER_SEC << " second(s)." << endl;

	
//
//
//		for (int j = 0;j < ref_points->GetNumberOfPoints();++j)
//		{
//			//std::vector<vtkSmartPointer<vtkPolyData>> tmp_PD(2 * global_setting::offset_num + 1);
//			double plane_normal[3] = { 0 };
//			ref_blur_normals->GetTuple(j, plane_normal);
//			double* plane_center = ref_points->GetPoint(j);
//			/*plane_center[0] = plane_center[0] - global_setting::offset_num * global_setting::cutter_offset * plane_normal[0];
//			plane_center[1] = plane_center[1] - global_setting::offset_num * global_setting::cutter_offset * plane_normal[1];
//			plane_center[2] = plane_center[2] - global_setting::offset_num * global_setting::cutter_offset * plane_normal[2];*/
//			vtkSmartPointer<vtkPlane> cutter_plane =
//				vtkSmartPointer<vtkPlane>::New();
//			/*plane_center[0] = plane_center[0] + global_setting::cutter_offset * plane_normal[0];
//			plane_center[1] = plane_center[1] + global_setting::cutter_offset * plane_normal[1];
//			plane_center[2] = plane_center[2] + global_setting::cutter_offset * plane_normal[2];*/
//			cutter_plane->SetOrigin(plane_center);
//			cutter_plane->SetNormal(plane_normal);
//			vtkSmartPointer<vtkCutter> ref_cutter =
//				vtkSmartPointer<vtkCutter>::New();
//			ref_cutter->RemoveAllInputs();
//			ref_cutter->SetCutFunction(cutter_plane);
//			ref_cutter->SetInputData(ref_contour_PD);//切原STL数据，尽量使数据完整
//			//cutter->SetNumberOfContours(2 * global_setting::offset_num + 1);
//			ref_cutter->GenerateValues(2 * global_setting::offset_num + 1, -global_setting::offset_num * global_setting::cutter_offset, global_setting::offset_num * global_setting::cutter_offset);
//			ref_cutter->GenerateTrianglesOn();
//			ref_cutter->Update();
//			//for (int k = 0;k < 2 * global_setting::offset_num + 1;k++)
//			//{
//			//	//vtkSmartPointer<vtkPlane> cutter_plane =
//			//	//	vtkSmartPointer<vtkPlane>::New();
//			//	//plane_center[0] = plane_center[0] + global_setting::cutter_offset * plane_normal[0];
//			//	//plane_center[1] = plane_center[1] + global_setting::cutter_offset * plane_normal[1];
//			//	//plane_center[2] = plane_center[2] + global_setting::cutter_offset * plane_normal[2];
//			//	//cutter_plane->SetOrigin(plane_center);
//			//	//cutter_plane->SetNormal(plane_normal);
//			//	//vtkSmartPointer<vtkCutter> cutter =
//			//	//	vtkSmartPointer<vtkCutter>::New();
//			//	//cutter->SetCutFunction(cutter_plane);
//			//	//cutter->SetInputData(ref_contour_PD);//切原STL数据，尽量使数据完整
//			//	//cutter->SetNumberOfContours(2 * global_setting::offset_num + 1);
//			//	//cutter->SetValue(0,)
//			//	//cutter->GenerateTrianglesOn();
//			//	//cutter->Update();
//
//			//	//vtkNew<vtkStripper> stripper;
//			//	//stripper->SetInputConnection(cutter->GetOutputPort());
//			//	//stripper->JoinContiguousSegmentsOn();
//			//	//stripper->Update();
//
//			//	//vtkSmartPointer<vtkPolyDataConnectivityFilter> connectivityFilter =
//			//	//	vtkSmartPointer<vtkPolyDataConnectivityFilter>::New();
//			//	//connectivityFilter->SetInputData(stripper->GetOutput());
//			//	//connectivityFilter->SetExtractionModeToClosestPointRegion();
//			//	//connectivityFilter->SetClosestPoint(plane_center); //select the region to extract here
//			//	//connectivityFilter->Update();
//
//			//	//std::cout<< cutter->GetOutput()->GetPoints()->GetNumberOfPoints()<<std::endl;
//			//	tmp_PD.at(k) = cutter->GetOutput();
//			//	//std::cout << ref_cutter_des.at(j).at(k).getNormal()[0] << std::endl;
//			//	//std::cout << "is NULL:" << bool(ref_cutter_des.at(j).at(k).getLeft() == NULL)<< std::endl;
//			//	//std::cout << getCutterDistance(cutter->GetOutput(), plane_center, plane_normal, ref_cutter_dist.at(j).at(k))->getPosition()[0] << std::endl;
//			//    ref_cutter_des.at(j).push_back(getCutterDistance(cutter->GetOutput(), plane_center, plane_normal, ref_cutter_dist.at(j).at(k)));
//			//	//std::cout << "return NULL?:" << (ref_cutter_des.at(j).at(0) == NULL) << std::endl;
//			//}
//			
//			ref_cutter_des.at(j) = getCutterDistance(ref_cutter->GetOutput(), plane_center, plane_normal, ref_cutter_dist.at(j));
//			ref_cutter_PD.at(j) = ref_cutter->GetOutput();
//			
//			std::cout << i << " " << j << " " << std::endl;
//		}
//		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//		getDistanceIndex(ref_downSample_PD, downSampleSTLs, distance_idx, tar_index);
//		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//		int tmp = 0;
//		for (int idx = 0;idx < STLs.size();idx++)
//		{
//			//std::cout <<distance_idx[idx]<<" ";
//			if (std::find(tar_index.begin(), tar_index.end(), distance_idx.at(idx)) == tar_index.end())
//			{
//				tmp = distance_idx.at(idx);
//				*std::find((tar_index.begin()++), tar_index.end(), 0) = tmp;
//				break;
//			}
//		}
//		vtkSmartPointer<vtkPolyData> tar_downsample_PD = downSampleSTLs.at(tmp);
//		vtkSmartPointer<vtkPolyData> tar_PD = STLs.at(tmp);
//		vtkSmartPointer<vtkPolyData> tar_contour_PD = contour_downSampleSTLs.at(tmp);
//		vtkSmartPointer<vtkPolyDataNormals> tar_PD_normals =
//			vtkSmartPointer<vtkPolyDataNormals>::New();
//		tar_PD_normals->SetInputData(tar_PD);
//		tar_PD_normals->ComputePointNormalsOn();
//		tar_PD_normals->ComputeCellNormalsOff();
//		tar_PD_normals->SetAutoOrientNormals(1);
//		tar_PD_normals->SetSplitting(0);
//		tar_PD_normals->Update();
//		vtkSmartPointer<vtkPoints> tar_points = tar_downsample_PD->GetPoints();
//		vtkSmartPointer<vtkKdTree> tar_kd_tree =
//			vtkSmartPointer<vtkKdTree>::New();
//		tar_kd_tree->BuildLocatorFromPoints(tar_PD->GetPoints());
//		tar_kd_tree->Update();
//		vtkSmartPointer<vtkDoubleArray> tar_blur_normals = getBlurNormal(tar_kd_tree, tar_PD_normals->GetOutput()->GetPointData()->GetNormals(), tar_points, global_setting::kdTree_num);
//		std::vector< vtkSmartPointer<vtkPolyData>> tar_cutter_PD(tar_points->GetNumberOfPoints());
//		//std::vector<vtkSmartPointer<vtkPolyData>> tmp_PD(int(2 * global_setting::offset_num + 1));
//		std::vector< std::vector<double>> tar_cutter_dist(tar_points->GetNumberOfPoints());
//		std::vector<cutter_descriptor*> tar_cutter_des(tar_points->GetNumberOfPoints());
//		
//		for (int j = 0;j < tar_points->GetNumberOfPoints();++j)
//		{
//			//std::vector<vtkSmartPointer<vtkPolyData>> tmp_PD(int(2 * global_setting::offset_num + 1));
//			double plane_normal[3] = { 0 };
//			tar_blur_normals->GetTuple(j, plane_normal);
//			double* plane_center = tar_points->GetPoint(j);
//		/*	plane_center[0] = plane_center[0] - global_setting::offset_num * global_setting::cutter_offset * plane_normal[0];
//			plane_center[1] = plane_center[1] - global_setting::offset_num * global_setting::cutter_offset * plane_normal[1];
//			plane_center[2] = plane_center[2] - global_setting::offset_num * global_setting::cutter_offset * plane_normal[2];*/
//			vtkSmartPointer<vtkPlane> cutter_plane =
//				vtkSmartPointer<vtkPlane>::New();
//			/*plane_center[0] = plane_center[0] + global_setting::cutter_offset * plane_normal[0];
//			plane_center[1] = plane_center[1] + global_setting::cutter_offset * plane_normal[1];
//			plane_center[2] = plane_center[2] + global_setting::cutter_offset * plane_normal[2];*/
//			cutter_plane->SetOrigin(plane_center);
//			cutter_plane->SetNormal(plane_normal);
//			vtkSmartPointer<vtkCutter> tar_cutter =
//				vtkSmartPointer<vtkCutter>::New();
//			tar_cutter->RemoveAllInputs();
//			tar_cutter->SetCutFunction(cutter_plane);
//			tar_cutter->SetInputData(tar_contour_PD);//切原STL数据，尽量使数据完整
//			//cutter->SetNumberOfContours(2 * global_setting::offset_num + 1);
//			tar_cutter->GenerateValues(2 * global_setting::offset_num + 1, -global_setting::offset_num * global_setting::cutter_offset, global_setting::offset_num* global_setting::cutter_offset);
//			tar_cutter->GenerateTrianglesOn();
//			tar_cutter->Update();
//			//for (int k = 0;k < 2 * global_setting::offset_num + 1;k++)
//			//{
//			//	vtkSmartPointer<vtkPlane> cutter_plane =
//			//		vtkSmartPointer<vtkPlane>::New();
//			//	plane_center[0] = plane_center[0] + global_setting::cutter_offset * plane_normal[0];
//			//	plane_center[1] = plane_center[1] + global_setting::cutter_offset * plane_normal[1];
//			//	plane_center[2] = plane_center[2] + global_setting::cutter_offset * plane_normal[2];
//			//	cutter_plane->SetOrigin(plane_center);
//			//	cutter_plane->SetNormal(plane_normal);
//			//	vtkSmartPointer<vtkCutter> cutter =
//			//		vtkSmartPointer<vtkCutter>::New();
//			//	cutter->SetCutFunction(cutter_plane);
//			//	cutter->SetInputData(tar_contour_PD);//切原STL数据，尽量使数据完整
//			//	cutter->Update();
//
//			//	//vtkNew<vtkStripper> stripper;
//			//	//stripper->SetInputConnection(cutter->GetOutputPort());
//			//	//stripper->JoinContiguousSegmentsOn();
//			//	//stripper->Update();
//
//			//	//vtkSmartPointer<vtkPolyDataConnectivityFilter> connectivityFilter =
//			//	//	vtkSmartPointer<vtkPolyDataConnectivityFilter>::New();
//			//	//connectivityFilter->SetInputData(stripper->GetOutput());
//			//	//connectivityFilter->SetExtractionModeToClosestPointRegion();
//			//	//connectivityFilter->SetClosestPoint(plane_center); //select the region to extract here
//			//	//connectivityFilter->Update();
//
//			//	tmp_PD.at(k) = cutter->GetOutput();
//			//	tar_cutter_des.at(j).push_back(getCutterDistance(cutter->GetOutput(), plane_center, plane_normal, tar_cutter_dist.at(j).at(k)));
//			//}
//			
//			tar_cutter_des.at(j) = getCutterDistance(tar_cutter->GetOutput(), plane_center, plane_normal, tar_cutter_dist.at(j));
//			tar_cutter_PD.at(j) = tar_cutter->GetOutput();
//
//			std::cout << i << " " << j << " " << std::endl;
//		}
//		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//		double effect_index[2] = { 0 };
//		double bestScore = 0;
//	   /* size_t contour_tmp_num = global_setting::offset_num;
//		size_t contour_num = global_setting::offset_num;*/
//		//std::vector<double> fit_points(18);
//		vtkSmartPointer<vtkMatrix4x4> right_tf_matrix =
//			vtkSmartPointer<vtkMatrix4x4>::New();
//		for (int j = 0;j < ref_points->GetNumberOfPoints();++j)
//		{
//		/*	std::vector<bool> ref0flag(2 * size_t(global_setting::offset_num) + 1, true);
//			for (int k = 0;k < 2 * size_t(global_setting::offset_num) + 1;k++)
//			{
//				if (ref_cutter_dist.at(j).at(k).size() == 0)
//					ref0flag.at(k) = false;
//			}
//			if (std::find(ref0flag.begin(), ref0flag.end(), true) == ref0flag.end())
//				continue;*/
//			vtkSmartPointer<vtkKdTree> ref_kd_tree =
//				vtkSmartPointer<vtkKdTree>::New();
//			ref_kd_tree->BuildLocatorFromPoints(ref_cutter_PD.at(j)->GetPoints());
//			ref_kd_tree->Update();
//			for (int k = 0;k < tar_points->GetNumberOfPoints();++k)
//			{
//				double last_score = 0;
//				/*std::vector<double> cur_fit_points1(18);
//				std::vector<double> cur_fit_points2(18);
//				std::vector<double> cur_fit_points3(18);*/
//				vtkSmartPointer<vtkMatrix4x4> cur_tf_matrix =
//					vtkSmartPointer<vtkMatrix4x4>::New();
//				//std::cout << "bestScore:" << bestScore << " ";
//				last_score = match_score(ref_cutter_PD.at(j),tar_cutter_PD.at(k),ref_kd_tree, ref_cutter_dist.at(j), tar_cutter_dist.at(k), ref_cutter_des.at(j), tar_cutter_des.at(k), cur_tf_matrix, global_setting::delta_angle,  global_setting::bias);
//				//for (size_t u = 0;u < global_setting::offset_num;u++)
//				//{
//				//	double cur_score1 = 0;
//				//	double cur_score2 = 0;
//				//	if (ref0flag.at(u))
//				//		cur_score1 = match_score(ref_cutter_dist.at(j).at(u), tar_cutter_dist.at(k).at(size_t(global_setting::offset_num) * 2 - u), ref_cutter_des.at(j).at(u), tar_cutter_des.at(k).at(size_t(global_setting::offset_num) * 2 - u), cur_fit_points2, global_setting::sparse_num, global_setting::bias);
//				//	if (ref0flag.at(size_t(global_setting::offset_num) * 2 - u))
//				//		cur_score2 = match_score(ref_cutter_dist.at(j).at(size_t(global_setting::offset_num) * 2 - u), tar_cutter_dist.at(k).at(u), ref_cutter_des.at(j).at(size_t(global_setting::offset_num) * 2 - u), tar_cutter_des.at(k).at(u), cur_fit_points3, global_setting::sparse_num, global_setting::bias);
//				//	bool flag = false;
//				//	if (cur_score1 > cur_score2)
//				//	{
//				//		//std::cout << "cur_score1 is larger than cur_score2!" << std::endl;
//				//		cur_fit_points3.assign(cur_fit_points2.begin(), cur_fit_points2.end());
//				//		cur_score2 = cur_score1;
//				//	}
//				//	if (cur_score2 > last_score)
//				//	{
//				//		//std::cout << "cur_score2 is larger last_score!" << std::endl;
//				//		last_score = cur_score2;
//				//		cur_fit_points1.assign(cur_fit_points3.begin(), cur_fit_points3.end());
//				//		contour_tmp_num = 2 * size_t(global_setting::offset_num) - u;
//				//		//std::cout << "bestScore:" << bestScore << "   last score:" << last_score << "   cur_score1:" << cur_score1 << "   cur_score2:" << cur_score2 << std::endl;
//				//	}
//				//}
//				if (last_score > bestScore)
//				{
//					std::cout << "bestScore:" << last_score<< "   last score:" << bestScore << std::endl;
//					//contour_num = contour_tmp_num;
//					bestScore = last_score;
//					right_tf_matrix = cur_tf_matrix;
//				    //fit_points.assign(cur_fit_points1.begin(), cur_fit_points1.end());
//					/*for (int u = 0;u < 6;++u)
//					{
//						std::cout << cur_fit_points1.at(3 * u) << " " << cur_fit_points1.at(3 * u + 1) << " " << cur_fit_points1.at(3 * u + 2) << " ";
//					}
//					std::cout << std::endl;
//					for (int u = 0;u < 6;++u)
//					{
//						std::cout << fit_points.at(3 * u) << " " << fit_points.at(3 * u + 1) << " " << fit_points.at(3 * u + 2) << " ";
//					}
//					std::cout << std::endl;*/
//					//std::cout << "a larger score appear!" << std::endl;
//					/*for (int n = 0;n < 4;n++)
//					{
//						for (int m = 0;m < 4;m++)
//						{
//							std::cout << right_tf_matrix->GetElement(n, m) << " ";
//						}
//						std::cout << std::endl;
//					}*/
//					effect_index[0] = k;
//					effect_index[1] = j;
//				}
//				/*if (bestScore == 1/ global_setting::bias)
//					break;*/
//			}
//			/*if (bestScore == 1/ global_setting::bias)
//				break;*/
//			//std::vector<bool> ref0flag(2*size_t(global_setting::offset_num)+1,true);
//			//for (int k = 0;k < 2 * global_setting::offset_num + 1;k++)
//			//{
//			//	if (ref_cutter_dist.at(j).at(k).size() == 0)
//			//		ref0flag.at(k) = false;
//			//}
//			//if (std::find(ref0flag.begin(), ref0flag.end(), true) == ref0flag.end())
//			//	continue;
//			//for (int k = 0;k < tar_points->GetNumberOfPoints();++k)
//			//{
//			//	double last_score = -32400;
//			//	std::vector<double> cur_fit_points1(18);
//			//	std::vector<double> cur_fit_points2(18);
//			//	std::vector<double> cur_fit_points3(18);
//			//	/*if (ref_cutter_des.at(j).at(global_setting::offset_num)->isEmpty())
//			//	{
//			//		std::cout << "descriptor is empty!" << std::endl;
//			//	}*/
//			//	if (ref0flag.at(global_setting::offset_num))
//			//		last_score = match_score(ref_cutter_dist.at(j).at(global_setting::offset_num), tar_cutter_dist.at(k).at(global_setting::offset_num), ref_cutter_des.at(j).at(global_setting::offset_num), tar_cutter_des.at(k).at(global_setting::offset_num), cur_fit_points1, global_setting::sparse_num);
//			//	for (size_t u = 0;u < global_setting::offset_num;u++)
//			//	{
//			//		double cur_score1 = -32400;
//			//		double cur_score2 = -32400;
//			//		if (ref0flag.at(u))
//			//			cur_score1 = double(match_score(ref_cutter_dist.at(j).at(u), tar_cutter_dist.at(k).at(size_t(global_setting::offset_num) * 2 - u), ref_cutter_des.at(j).at(u), tar_cutter_des.at(k).at(size_t(global_setting::offset_num) * 2 - u), cur_fit_points2, global_setting::sparse_num));
//			//		if (ref0flag.at(size_t(global_setting::offset_num) * 2 - u))
//			//			cur_score2 = double(match_score(ref_cutter_dist.at(j).at(size_t(global_setting::offset_num) * 2 - u), tar_cutter_dist.at(k).at(u), ref_cutter_des.at(j).at(size_t(global_setting::offset_num) * 2 - u), tar_cutter_des.at(k).at(u), cur_fit_points3, global_setting::sparse_num));
//			//		bool flag = false;
//			//	    //std::cout <<"bestScore:"<<bestScore<<"   last score:" << last_score << "   cur_score1:" << cur_score1 << "   cur_score2:" << cur_score2 << std::endl;
//			//		/*for (int u = 0;u < 6;++u)
//			//		{
//			//			std::cout << cur_fit_points1.at(3 * u) << " " << cur_fit_points1.at(3 * u + 1) << " " << cur_fit_points1.at(3 * u + 2) << " ";
//			//		}
//			//		std::cout << std::endl;
//			//		for (int u = 0;u < 6;++u)
//			//		{
//			//			std::cout << cur_fit_points2.at(3 * u) << " " << cur_fit_points2.at(3 * u + 1) << " " << cur_fit_points2.at(3 * u + 2) << " ";
//			//		}
//			//		std::cout << std::endl;
//			//		std::cout << std::endl;*/
//			//		if (cur_score1 > cur_score2)
//			//		{
//			//			//std::cout << "cur_score1 is larger than cur_score2!" << std::endl;
//			//			cur_fit_points3.assign(cur_fit_points2.begin(), cur_fit_points2.end());
//			//			cur_score2 = cur_score1;
//			//		}
//			//		if (cur_score2 > last_score)
//			//		{
//			//			//std::cout << "cur_score2 is larger last_score!" << std::endl;
//			//			last_score = cur_score2;
//			//			cur_fit_points1.assign(cur_fit_points3.begin(), cur_fit_points3.end());
//			//			contour_tmp_num = 2 * size_t(global_setting::offset_num) - u;
//			//			std::cout << "bestScore:" << bestScore << "   last score:" << last_score << "   cur_score1:" << cur_score1 << "   cur_score2:" << cur_score2 << std::endl;
//			//		}
//			//		/*for (int u = 0;u < 6;++u)
//			//		{
//			//			std::cout << cur_fit_points1.at(3 * u) << " " << cur_fit_points1.at(3 * u + 1) << " " << cur_fit_points1.at(3 * u + 2) << " ";
//			//		}
//			//		std::cout << std::endl;*/
//			//	}
//			//	if (last_score > bestScore)
//			//	{
//			//		contour_num = contour_tmp_num;
//			//		bestScore = last_score;
//			//		fit_points.assign(cur_fit_points1.begin(), cur_fit_points1.end());
//			//		for (int u = 0;u < 6;++u)
//			//		{
//			//			std::cout << cur_fit_points1.at(3 * u) << " " << cur_fit_points1.at(3 * u + 1) << " " << cur_fit_points1.at(3 * u + 2) << " ";
//			//		}
//			//		std::cout << std::endl;
//			//		for (int u = 0;u < 6;++u)
//			//		{
//			//			std::cout << fit_points.at(3 * u) << " " << fit_points.at(3 * u + 1) << " " << fit_points.at(3 * u + 2) << " ";
//			//		}
//			//		std::cout << std::endl;
//			//		//std::cout << "a larger score appear!" << std::endl;
//			//		effect_index[0][0] = k;
//			//		effect_index[0][1] = j;
//			//	}
//			//	if (bestScore == 0)
//			//		break;
//			//}
//			//if (bestScore == 0)
//			//	break;
//		}
//		/*double ref_points1[3] = { fit_points.at(0),fit_points.at(1),fit_points.at(2) };
//		double ref_points2[3] = { fit_points.at(3),fit_points.at(4),fit_points.at(5) };
//		double ref_points3[3] = { fit_points.at(6),fit_points.at(7),fit_points.at(8) };
//
//		double tar_points1[3] = { fit_points.at(9),fit_points.at(10),fit_points.at(11) };
//		double tar_points2[3] = { fit_points.at(12),fit_points.at(13),fit_points.at(14) };
//		double tar_points3[3] = { fit_points.at(15),fit_points.at(16),fit_points.at(17) };*/
//		/*for (int u = 0;u < 6;++u)
//		{
//			std::cout << fit_points.at(3 * u) << " " << fit_points.at(3 * u + 1) << " " << fit_points.at(3 * u + 2) << std::endl;
//		}*/
//
//		/*vtkSmartPointer<vtkPoints> tar_tf_points =
//			vtkSmartPointer<vtkPoints>::New();
//		tar_tf_points->InsertNextPoint(tar_points1);
//		tar_tf_points->InsertNextPoint(tar_points2);
//		tar_tf_points->InsertNextPoint(tar_points3);
//	
//		vtkSmartPointer<vtkPoints> ref_tf_points =
//			vtkSmartPointer<vtkPoints>::New();
//		ref_tf_points->InsertNextPoint(ref_points1);
//		ref_tf_points->InsertNextPoint(ref_points2);
//		ref_tf_points->InsertNextPoint(ref_points3);
//		vtkSmartPointer<vtkLandmarkTransform> tf =
//			vtkSmartPointer<vtkLandmarkTransform>::New();
//		tf->SetSourceLandmarks(tar_tf_points);
//		tf->SetTargetLandmarks(ref_tf_points);
//		tf->SetModeToRigidBody();
//		tf->Modified();
//		tf->Update();*/
//		
//
//		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//		//vtkSmartPointer<vtkTransform> tf =
//		//	vtkSmartPointer<vtkTransform>::New();
//		//tf->PostMultiply();
//		//tf->Translate(-tar_points1[0], -tar_points1[1], -tar_points1[2]);
//		//double axis1[3] = { 1,0,0 };
//		//vtkMath::Cross(ref_normal, tar_normal, axis1);
//		//vtkMath::Normalize(tar_normal);
//		//vtkMath::Normalize(ref_normal);
//		//double angle1 = 180 - acos(vtkMath::Dot(ref_normal, tar_normal)) * 180 / acos(-1);
//		//tf->RotateWXYZ(angle1,axis1);
//		//double ref_dir[3] = { ref_points2[0] - ref_points1[0],ref_points2[1] - ref_points1[1], ref_points2[2] - ref_points1[2] };
//		//double tar_dir[3] = { tar_points2[0] - tar_points1[0],tar_points2[1] - tar_points1[1], tar_points2[2] - tar_points1[2] };
//		//double ref_length = vtkMath::Dot(ref_normal, ref_dir);
//		//double ref_plane_vec[3] = { ref_dir[0] - ref_length * ref_normal[0],ref_dir[1] - ref_length * ref_normal[1], ref_dir[2] - ref_length * ref_normal[2] };
//		//double tar_length = vtkMath::Dot(ref_normal, tar_dir);
//		//double tar_plane_vec[3] = { ref_dir[0] - tar_length * tar_normal[0],ref_dir[1] - tar_length * tar_normal[1], ref_dir[2] - tar_length * tar_normal[2] };
//		////double axis2[3] = { 1,0,0 };
//		////vtkMath::Cross(ref_dir, tar_dir, axis2);
//		//double angle2 = acos(vtkMath::Dot(ref_plane_vec, tar_plane_vec)) * 180 / acos(-1);
//		//double axis2[3] = { 1,0,0 };
//		//vtkMath::Cross( tar_plane_vec, ref_plane_vec, axis2);
//		//tf->RotateWXYZ(angle2, axis2);
//		//tf->Translate(ref_points1[0], ref_points1[1], ref_points1[2]);
//		//tf->Update();
//  //      /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//		vtkSmartPointer<vtkTransform> tf =
//			vtkSmartPointer<vtkTransform>::New();
//		tf->PostMultiply();
//		tf->SetMatrix(right_tf_matrix);
//		tf->Update();
//		tf_matrix.at(tmp) = tf->GetMatrix();
//
//
//		vtkSmartPointer<vtkTransformPolyDataFilter> tf_PD =
//			vtkSmartPointer<vtkTransformPolyDataFilter>::New();
//		tf_PD->SetInputData(STLs.at(tmp));
//		tf_PD->SetTransform(tf);
//		tf_PD->Update();
//
//		vtkSmartPointer<vtkTransformPolyDataFilter> tf_downSample_PD =
//			vtkSmartPointer<vtkTransformPolyDataFilter>::New();
//		tf_downSample_PD->SetInputData(downSampleSTLs.at(tmp));
//		tf_downSample_PD->SetTransform(tf);
//		tf_downSample_PD->Update();
//
//		vtkSmartPointer<vtkTransformPolyDataFilter> tf_downSmaple_contour_PD =
//			vtkSmartPointer<vtkTransformPolyDataFilter>::New();
//		tf_downSmaple_contour_PD->SetInputData(contour_downSampleSTLs.at(tmp));
//		tf_downSmaple_contour_PD->SetTransform(tf);
//		tf_downSmaple_contour_PD->Update();
//
//		std::vector<vtkSmartPointer<vtkPolyData>> ref_merged_PD(2);
//		ref_merged_PD.at(0) = ref_PD;
//		ref_merged_PD.at(1) = tf_PD->GetOutput();
//
//		std::vector<vtkSmartPointer<vtkPolyData>> ref_downSample_merged_PD(2);
//		ref_downSample_merged_PD.at(0) = ref_downSample_PD;
//		ref_downSample_merged_PD.at(1) = tf_downSample_PD->GetOutput();
//
//		std::vector<vtkSmartPointer<vtkPolyData>> ref_contour_merged_PD(2);
//		ref_contour_merged_PD.at(0) = ref_contour_PD;
//		ref_contour_merged_PD.at(1) = tf_downSmaple_contour_PD->GetOutput();
//
//		mergeSTLs(ref_downSample_PD, ref_downSample_merged_PD);
//		mergeSTLs(ref_PD, ref_merged_PD);
//		mergeSTLs(ref_contour_PD, ref_contour_merged_PD);
//		std::cout << i << std::endl;
//        
//		//ref_cutter_PD.at(effect_index[0][1]);
//		//std::cout << contour_num << std::endl;
//		//std::cout << ref_cutter_PD.at(effect_index[0][1]).size() << std::endl;
//		//ref_cutter_PD.at(effect_index[0][1]).at(contour_num);
//		vtkSmartPointer<vtkPolyDataMapper> ref_contour_mapper =
//			vtkSmartPointer<vtkPolyDataMapper>::New();
//		ref_contour_mapper->SetInputData(ref_cutter_PD.at(effect_index[1]));
//		ref_contour_mapper->Update();
//
//		vtkSmartPointer<vtkActor> ref_contour_actor =
//			vtkSmartPointer<vtkActor>::New();
//		ref_contour_actor->SetMapper(ref_contour_mapper);
//		ref_contour_actor->GetProperty()->SetColor(1, 0, 0);
//
//		vtkSmartPointer<vtkTransformPolyDataFilter> tf_contour_PD =
//			vtkSmartPointer<vtkTransformPolyDataFilter>::New();
//		tf_contour_PD->SetInputData(tar_cutter_PD.at(effect_index[0]));
//		tf_contour_PD->SetTransform(tf);
//		tf_contour_PD->Update();
//		vtkSmartPointer<vtkPolyDataMapper> tar_contour_mapper =
//			vtkSmartPointer<vtkPolyDataMapper>::New();
//		tar_contour_mapper->SetInputData(tf_contour_PD->GetOutput());
//		tar_contour_mapper->Update();
//
//		vtkSmartPointer<vtkActor> tar_contour_actor =
//			vtkSmartPointer<vtkActor>::New();
//		tar_contour_actor->SetMapper(tar_contour_mapper);
//		tar_contour_actor->GetProperty()->SetColor(0, 1, 1);
//
//		/*vtkSmartPointer<vtkSphereSource> ref_sphere =
//			vtkSmartPointer<vtkSphereSource>::New();
//		ref_sphere->SetCenter(ref_cutter_des.at(effect_index[1])->getOrigin());
//		ref_sphere->SetRadius(2);
//		ref_sphere->Update();
//
//		vtkSmartPointer<vtkPolyDataMapper> ref_sp_mapper =
//			vtkSmartPointer<vtkPolyDataMapper>::New();
//		ref_sp_mapper->SetInputData(ref_sphere->GetOutput());
//
//		vtkSmartPointer<vtkActor> ref_sp_actor =
//			vtkSmartPointer<vtkActor>::New();
//		ref_sp_actor->SetMapper(ref_sp_mapper);
//		ref_sp_actor->GetProperty()->SetColor(1, 1, 1);
//		ref_sp_actor->GetProperty()->SetOpacity(3);
//
//		vtkSmartPointer<vtkSphereSource> tar_sphere =
//			vtkSmartPointer<vtkSphereSource>::New();
//		tar_sphere->SetCenter(tar_cutter_des.at(effect_index[0])->getOrigin());
//		tar_sphere->SetRadius(5);
//		tar_sphere->Update();
//
//		vtkSmartPointer<vtkTransformPolyDataFilter> tf_sp_PD =
//			vtkSmartPointer<vtkTransformPolyDataFilter>::New();
//		tf_sp_PD->SetInputData(tar_sphere->GetOutput());
//		tf_sp_PD->SetTransform(tf);
//		tf_sp_PD->Update();
//
//		vtkSmartPointer<vtkPolyDataMapper> tar_sp_mapper =
//			vtkSmartPointer<vtkPolyDataMapper>::New();
//		tar_sp_mapper->SetInputData(tf_sp_PD->GetOutput());
//
//		vtkSmartPointer<vtkActor> tar_sp_actor =
//			vtkSmartPointer<vtkActor>::New();
//		tar_sp_actor->SetMapper(tar_sp_mapper);
//		tar_sp_actor->GetProperty()->SetColor(0, 0, 1);
//		tar_sp_actor->GetProperty()->SetOpacity(0.6);
//		std::cout << "ref_center: " << ref_cutter_des.at(effect_index[1])->getOrigin()[0] << " " << ref_cutter_des.at(effect_index[1])->getOrigin()[1] << " " << ref_cutter_des.at(effect_index[1])->getOrigin()[2] << std::endl;
//		std::cout << "tar_center: " << tar_cutter_des.at(effect_index[0])->getOrigin()[0] << " " << tar_cutter_des.at(effect_index[0])->getOrigin()[1] << " " << tar_cutter_des.at(effect_index[0])->getOrigin()[2] << std::endl;*/
//		/*vtkSmartPointer<vtkLineSource> line1 =
//			vtkSmartPointer<vtkLineSource>::New();
//		line1->SetPoint1(ref_points1);
//		line1->SetPoint2(ref_points2);
//		line1->Update();
//
//		vtkSmartPointer<vtkLineSource> line2 =
//			vtkSmartPointer<vtkLineSource>::New();
//		line2->SetPoint1(ref_points1);
//		line2->SetPoint2(ref_points3);
//		line2->Update();
//
//		vtkSmartPointer<vtkLineSource> line3 =
//			vtkSmartPointer<vtkLineSource>::New();
//		line3->SetPoint1(ref_points3);
//		line3->SetPoint2(ref_points2);
//		line3->Update();
//
//		vtkSmartPointer<vtkDataSetMapper> line_mapper =
//			vtkSmartPointer<vtkDataSetMapper>::New();
//		line_mapper->AddInputConnection(line1->GetOutputPort());
//		line_mapper->AddInputConnection(line2->GetOutputPort());
//		line_mapper->AddInputConnection(line3->GetOutputPort());
//		line_mapper->Update();
//
//		vtkSmartPointer<vtkActor> line_actor =
//			vtkSmartPointer<vtkActor>::New();
//		line_actor->SetMapper(line_mapper);
//		line_actor->GetProperty()->SetColor(1, 1, 1);
//		line_actor->GetProperty()->SetLineWidth(1);*/
//
//
//		vtkSmartPointer<vtkRenderer> contour_renderer =
//			vtkSmartPointer<vtkRenderer>::New();
//		contour_renderer->AddActor(ref_contour_actor);
//		contour_renderer->AddActor(tar_contour_actor);
//		//contour_renderer->AddActor(ref_sp_actor);
//		//contour_renderer->AddActor(tar_sp_actor);
//		//contour_renderer->AddActor(line_actor);
//		renderWindow->AddRenderer(contour_renderer);
//		contour_renderer->SetViewport(i * 0.4, 0, (i + 1) * 0.4, 0.4);
//		contour_renderer->SetBackground(0, 0, 0);
//		/*vtkSmartPointer<vtkRenderer> tar_contour_renderer =
//			vtkSmartPointer<vtkRenderer>::New();
//		tar_contour_renderer->AddActor(tar_contour_actor);
//		renderWindow->AddRenderer(tar_contour_renderer);
//		tar_contour_renderer->SetViewport(i * 0.2, 0, (i + 1) * 0.2, 0.2);
//		tar_contour_renderer->SetBackground(1, 1, 1);*/
//	}
//	vtkSmartPointer<vtkPolyDataMapper> result_mapper =
//		vtkSmartPointer<vtkPolyDataMapper>::New();
//	result_mapper->SetInputData(ref_PD);
//	result_mapper->Update();
//
//	vtkSmartPointer<vtkActor> result_actor =
//		vtkSmartPointer<vtkActor>::New();
//	result_actor->SetMapper(result_mapper);
//	result_actor->GetProperty()->SetColor(0, 1, 0);
//	result_actor->GetProperty()->SetOpacity(1);
//
//	result_renderer->AddActor(result_actor);
//	renderWindow->AddRenderer(result_renderer);
//	renderWindow->AddRenderer(origin_renderer);
//	renderWindow->Render();
//	render_inter->Initialize();
//	render_inter->Start();
//	
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//	//for (size_t i = 0;i < downSampleSTLs.size();++i)
//	//{
//	//	vtkSmartPointer<vtkPolyData> pd = downSampleSTLs.at(i);
//	//	vtkSmartPointer<vtkPolyDataNormals> pd_normals = 
//	//		vtkSmartPointer<vtkPolyDataNormals>::New();
//	//	pd_normals->SetInputData(downSampleSTLs.at(i));
//	//	pd_normals->ComputePointNormalsOn();
//	//	pd_normals->ComputeCellNormalsOff();
//	//	pd_normals->SetAutoOrientNormals(1);
//	//	pd_normals->SetSplitting(0);
//	//	pd_normals->Update();
//
//	//	//vtkSmartPointer<vtkCurvatures> pd_cur =
//	//	//	vtkSmartPointer<vtkCurvatures>::New();
//	//	//pd_cur->SetInputData(pd);
//	//	////pd_cur->SetCurvatureTypeToMinimum();
//	//	//pd_cur->SetCurvatureTypeToMaximum();          //计算最大主曲率
//	//	////pd_cur->SetCurvatureTypeToGaussian();
//	//	////pd_cur->SetCurvatureTypeToMean();
//	//	//pd_cur->Update();
//
//	//	//std::cout << pd_normals->GetOutput()->GetPointData()->GetNormals()->GetTuple(10)[1] << std::endl;
//	//	fragments_normals.at(i) = pd_normals->GetOutput()->GetPointData()->GetNormals();
//	//	fragments_points.at(i) = pd_normals->GetOutput()->GetPoints();
//	//    //fragments_cur.at(i) = static_cast<vtkDataArray*>(pd_cur->GetOutput()->GetPointData()->GetArray("Maximum_Curvature"));
//	//	std::cout << "origin points size:" << pd->GetNumberOfPoints() << "  normal point size:" << fragments_points.at(i)->GetNumberOfPoints() <<"   normal size:"<<fragments_normals.at(i)->GetNumberOfTuples()<< std::endl;
//	//	for (size_t j = 0;j < fragments_points.at(i)->GetNumberOfPoints();++j)
//	//	{
//	//		std::vector<vtkSmartPointer<vtkPolyData>> tmd_pd;
//	//		double plane_normal[3] = { 0 };
//	//		fragments_normals.at(i)->GetTuple(j, plane_normal);
//	//		//vtkMath::Normalize(plane_normal);
//	//		//std::cout << plane_normal[0] << " " << plane_normal[1] << " " << plane_normal[2] << std::endl;
//	//		double* plane_center = fragments_points.at(i)->GetPoint(j);
//	//		//std::cout << plane_center[0] << " " << plane_center[1] << " " << plane_center[2] << std::endl;
//	//		plane_center[0] = plane_center[0] - global_setting::offset_num * global_setting::cutter_offset * plane_normal[0];
//	//		plane_center[1] = plane_center[1] - global_setting::offset_num * global_setting::cutter_offset * plane_normal[1];
//	//		plane_center[2] = plane_center[2] - global_setting::offset_num * global_setting::cutter_offset * plane_normal[2];
//	//		for (int k = 0;k < 2 * global_setting::offset_num + 1;k++)
//	//		{
//	//			vtkSmartPointer<vtkPlane> cutter_plane =
//	//				vtkSmartPointer<vtkPlane>::New();
//	//			plane_center[0] = plane_center[0] + global_setting::cutter_offset * plane_normal[0];
//	//			plane_center[1] = plane_center[1] + global_setting::cutter_offset * plane_normal[1];
//	//			plane_center[2] = plane_center[2] + global_setting::cutter_offset * plane_normal[2];
//	//			cutter_plane->SetOrigin(plane_center);
//	//			cutter_plane->SetNormal(plane_normal);
//	//			//std::cout << plane_center[0] << " " << plane_center[1] << " " << plane_center[2] << std::endl;
//	//			//std::cout << plane_normal[0] << " " << plane_normal[1] << " " << plane_normal[2] << std::endl;
//	//			vtkSmartPointer<vtkCutter> cutter =
//	//				vtkSmartPointer<vtkCutter>::New();
//	//			cutter->SetCutFunction(cutter_plane);
//	//			cutter->SetInputData(STLs.at(i));//切原STL数据，尽量使数据完整
//	//			cutter->Update();
//	//			tmd_pd.push_back(cutter->GetOutput());
//	//			getCutterDistance(cutter->GetOutput(),plane_center,plane_normal,cutter_pd_dist.at(i).at(j).at(k));
//	//		    //std::cout << STLs.at(i)->GetNumberOfPoints()<<" ";
//	//			//std::cout << cutter->GetOutput()->GetNumberOfPoints() << std::endl;
//	//		}
//	//		cutter_pd.at(i).push_back(tmd_pd);
//	//		//std::cout << tmd_pd.at(2)->GetNumberOfPoints() << std::endl;
//	//	}
//	//}
//
//	//std::vector<std::vector<vtkSmartPointer<vtkPolyData>>> ref_cutter_pd;
//	//ref_cutter_pd.assign(cutter_pd.at(distance_idx.at(0)).begin(), cutter_pd.at(distance_idx.at(0)).end());
//	//std::vector<std::vector<std::vector<double>>> ref_dist;
//	//ref_dist.assign(cutter_pd_dist.at(0).begin(), cutter_pd_dist.at(0).end());
//	//double effect_index[2][4] = { 0 };
//	//double best2score[3] = { 0 };
//	//double cur_score = 0;
//	//double last_score = 0;
//	//int p_index = 0;
//	//for (int i = 1;i < downSampleSTLs.size();++i)
//	//{
//
//	//	for (int j = 0;j < ref_cutter_pd.size();++j)
//	//	{
//	//		for (int k = 0;k < fragments_normals.at(distance_idx.at(i))->GetNumberOfTuples();++k)
//	//		{
//	//			last_score = match_dist(ref_dist.at(j).at(global_setting::offset_num), cutter_pd_dist.at(i).at(k).at(global_setting::offset_num), global_setting::sparse_num);
//	//			for (int u = 0;u < global_setting::offset_num;u++)
//	//			{
//	//				cur_score = match_dist(ref_dist.at(j).at(u), cutter_pd_dist.at(i).at(k).at(2 * global_setting::offset_num - u), global_setting::sparse_num);
//	//				if (cur_score > last_score)
//	//				{
//	//					last_score = cur_score;
//	//					p_index = k;
//	//				}
//	//			}
//	//			if (last_score > best2score[0])
//	//			{
//	//				best2score[1] = best2score[0];
//	//				best2score[0] = last_score;
//	//				effect_index[1][0] = effect_index[0][0];
//	//				effect_index[1][1] = effect_index[0][1];
//	//				effect_index[1][2] = effect_index[0][2];
//	//				effect_index[1][3] = effect_index[0][3];
//	//				effect_index[0][0] = distance_idx.at(i);
//	//				effect_index[0][1] = p_index;
//	//				effect_index[0][2] = distance_idx.at(i-1);//？？？？
//	//				effect_index[0][3] = j;
//	//			}
//	//			else if (last_score > best2score[1])
//	//			{
//	//				best2score[1] = last_score;
//	//				effect_index[1][0] = distance_idx.at(i);
//	//				effect_index[1][1] = p_index;
//	//				effect_index[1][2] = distance_idx.at(i - 1);//？？？？
//	//				effect_index[1][3] = j;
//	//			}
//
//	//		}
//	//	}
//	//	double* tar_points1 = fragments_points.at(effect_index[0][0])->GetPoint(effect_index[0][1]);
//	//	double* tar_points2 = fragments_points.at(effect_index[1][0])->GetPoint(effect_index[1][1]);
//	//	double* tar_normal1 = fragments_normals.at(effect_index[0][0])->GetTuple(effect_index[0][1]);
//	//
//	//	double* ref_points1 = fragments_points.at(effect_index[0][2])->GetPoint(effect_index[0][3]);
//	//	double* ref_points2 = fragments_points.at(effect_index[1][2])->GetPoint(effect_index[1][3]);
//	//	double* ref_normal1 = fragments_normals.at(effect_index[0][2])->GetTuple(effect_index[0][3]);
//
//	//	STLs.at(effect_index[0][0]);
//
//	//}
//
//	//vtkSmartPointer<vtkRenderer> pelvic_renderer = vtkSmartPointer<vtkRenderer>::New();
//	//pelvic_renderer->SetViewport(0, 0, 0.5, 1);
//	//vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
//	//renderWindow->Render();
//	//renderWindow->SetWindowName("fragments reduction planning");
//	//renderWindow->AddRenderer(pelvic_renderer);
//
//	///*vtkSmartPointer<vtkRenderWindow> renderWindow2 = vtkSmartPointer<vtkRenderWindow>::New();
//	//renderWindow2->Render();
//	//renderWindow2->SetWindowName("fragments reduction planning");
//	//renderWindow2->AddRenderer(pelvic_renderer);*/
//
//	//vtkSmartPointer<vtkPointPicker> pointPicker = vtkSmartPointer<vtkPointPicker>::New();
//
//	//vtkSmartPointer< vtkInteractorStyleTrackballCamera> InteracterStyle =
//	//	vtkSmartPointer< vtkInteractorStyleTrackballCamera>::New();
//
//	//vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
//	//renderWindowInteractor->SetPicker(pointPicker);
//	//renderWindowInteractor->SetRenderWindow(renderWindow);
//	//renderWindowInteractor->SetInteractorStyle(InteracterStyle);
//
//	/*vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor2 = vtkSmartPointer<vtkRenderWindowInteractor>::New();
//	renderWindowInteractor2->SetPicker(pointPicker);
//	renderWindowInteractor2->SetRenderWindow(renderWindow2);
//	renderWindowInteractor2->SetInteractorStyle(InteracterStyle);*/
//
//	//vtkSmartPointer<vtkCurvatures> pelvic_cur =
//	//	vtkSmartPointer<vtkCurvatures>::New();
//	//pelvic_cur->SetInputData(pelvic_polydata);
//	////pelvic_cur->SetCurvatureTypeToMinimum();
//	//pelvic_cur->SetCurvatureTypeToMaximum();          //计算最大主曲率
//	////pelvic_cur->SetCurvatureTypeToGaussian();
//	////pelvic_cur->SetCurvatureTypeToMean();
//
//
//	//获取最大曲率数据
//	//四种曲率属性数据分别对应属性名字为 Minimum_Curvature、Maximum_Curvature、Gauss_Curvature和Mean_Curvature
//	//vtkDataArray* maximum_cur = static_cast<vtkDataArray*>(pelvic_cur->GetOutput()->GetPointData()->GetArray("Maximum_Curvature"));
//	
//	//double scalarRange[2];
//	//pelvic_cur->GetOutput()->GetScalarRange(scalarRange);
//	//vtkSmartPointer<vtkLookupTable> lut =
//	//	vtkSmartPointer<vtkLookupTable>::New();
//	//lut->SetHueRange(0.4, 0);
//	//lut->SetAlphaRange(1.0, 1.0);
//	//lut->SetValueRange(1.0, 1.0);
//	//lut->SetSaturationRange(1.0, 1.0);
//	//lut->SetNumberOfTableValues(256);
//	//lut->SetRange(scalarRange);         //设置范围
//	//lut->Build();
//
//	/*vtkSmartPointer<vtkPolyDataNormals> pelvic_normals =
//		vtkSmartPointer<vtkPolyDataNormals>::New();
//	pelvic_normals->SetInputData();
//	pelvic_normals->SetComputePointNormals(1);
//	pelvic_normals->SetComputeCellNormals(0);
//	pelvic_normals->SetSplitting(0);
//	pelvic_normals->SetAutoOrientNormals(1);
//	pelvic_normals->Update();
//
//	std::cout << std::endl << "normal size of whole pelvic:" << pelvic_normals->GetOutput()->GetPointData()->GetNumberOfTuples() << std::endl;*/
//
//	//vtkSmartPointer<vtkPolyDataMapper> pelvic_mapper =
//	//	vtkSmartPointer<vtkPolyDataMapper>::New();
//	//pelvic_mapper->SetInputData(pelvic_polydata);//cutter_pd.at(1).at(10).at(2));
//	//pelvic_mapper->Update();
//	////pelvic_mapper->SetLookupTable(lut);
//	////std::cout << cutter_pd.at(1).at(1).at(2)->GetNumberOfPoints() << std::endl;
//	//vtkSmartPointer<vtkActor> pelvic_actor =
//	//	vtkSmartPointer<vtkActor>::New();
//	//pelvic_actor->SetMapper(pelvic_mapper);
//	//pelvic_actor->GetProperty()->SetColor(0, 1, 0);
//	//pelvic_actor->GetProperty()->SetOpacity(1);
//
//	////vtkScalarBarActor类，该类支持将一个颜色映射表转换为一个Actor对象
// //   //将颜色表以图形的形式显示，并支持设置图形相应的名字和显示的数据Label个数
//	//vtkSmartPointer<vtkScalarBarActor> scalarBar =
//	//	vtkSmartPointer<vtkScalarBarActor>::New();
//	//scalarBar->SetLookupTable(pelvic_mapper->GetLookupTable());
// //   scalarBar->SetTitle("normalized curvature");
//	//scalarBar->SetNumberOfLabels(5);
//	///*scalarBar->SetOrientationToHorizontal();
//	//scalarBar->SetWidth(1);
//	//scalarBar->SetHeight(0.1);*/
//
//	//// Add the actor to the scene
//	//pelvic_renderer->AddActor(pelvic_actor);
//	////pelvic_renderer->AddActor2D(scalarBar);
//	//pelvic_renderer->SetBackground(1.0, 1.0, 1.0);                        // 设置页面底部颜色值
//	//pelvic_renderer->SetBackground2(0.529, 0.8078, 0.92157);    // 设置页面顶部颜色值
//	//pelvic_renderer->SetGradientBackground(1);                           // 开启渐变色背景设置
//	//pelvic_renderer->ResetCamera();
//	//std::cout << "number of actors:" << pelvic_renderer->GetActors()->GetNumberOfItems() << std::endl;
//
//	////vtkSmartPointer<vtkPolyDataMapper> fragment_mapper2 =
//	////	vtkSmartPointer<vtkPolyDataMapper>::New();
//	////fragment_mapper2->SetInputData(downSampleSTLs.at(1));
//	////fragment_mapper2->Update();
//	//////vtkScalarBarActor类，该类支持将一个颜色映射表转换为一个Actor对象
//	//////将颜色表以图形的形式显示，并支持设置图形相应的名字和显示的数据Label个数
//	/////*vtkSmartPointer<vtkScalarBarActor> scalarBar2 =
//	////	vtkSmartPointer<vtkScalarBarActor>::New();
//	////scalarBar2->SetLookupTable(fragment_mapper2->GetLookupTable());
//	////scalarBar2->SetNumberOfLabels(5);*/
//	////vtkSmartPointer<vtkActor> fragment_actor2 =
//	////	vtkSmartPointer<vtkActor>::New();
//	////fragment_actor2->SetMapper(fragment_mapper2);
//	////vtkSmartPointer<vtkRenderer> fragment_renderer2 =
//	////	vtkSmartPointer<vtkRenderer>::New();
//	////fragment_renderer2->AddActor(fragment_actor2);
//	////fragment_renderer2->SetBackground(0, 0, 0);
//	//////std::cout <<i<<":"<< 0.5 + 0.25 * col << " " << double(row) / 3 << " " << 0.75 + 0.25 * col << " " << 1.0 / 3 + 1.0 / 3 * row << std::endl;
//	////renderWindow2->AddRenderer(fragment_renderer2);
//
//	//
//	//for (int i = 0;i < downSampleSTLs.size();++i)
//	//{
//	//	std::cout << " "<<distance_pd[i];
//	//	//vtkSmartPointer<vtkCurvatures> fragment_cur =
//	//	//	vtkSmartPointer<vtkCurvatures>::New();
//	//	//fragment_cur->SetInputData(downSampleSTLs.at(i));
//	//	//fragment_cur->SetCurvatureTypeToMaximum();          //计算最大主曲率
//	//	//fragment_cur->Update();
//	//	//vtkDataArray* gauss = static_cast<vtkDataArray*>(fragment_cur->GetOutput()->GetPointData()->GetArray("Maximum_Curvature"));
//	//	vtkSmartPointer<vtkPolyDataMapper> fragment_mapper =
//	//		vtkSmartPointer<vtkPolyDataMapper>::New();
//	//	fragment_mapper->SetInputData(downSampleSTLs.at(i));
//	//	//fragment_mapper->SetLookupTable(lut);
//	//	//vtkScalarBarActor类，该类支持将一个颜色映射表转换为一个Actor对象
//	//    //将颜色表以图形的形式显示，并支持设置图形相应的名字和显示的数据Label个数
//	//	vtkSmartPointer<vtkScalarBarActor> scalarBar =
//	//		vtkSmartPointer<vtkScalarBarActor>::New();
//	//	scalarBar->SetLookupTable(fragment_mapper->GetLookupTable());
//	//	//scalarBar->SetTitle(fragment_cur->GetOutput()->GetPointData()->GetScalars()->GetName());
//	//	scalarBar->SetNumberOfLabels(5);
//	//	vtkSmartPointer<vtkActor> fragment_actor =
//	//		vtkSmartPointer<vtkActor>::New();
//	//	fragment_actor->SetMapper(fragment_mapper);
//	//	fragment_actor->GetProperty()->SetColor(0, 1, 0);
//	//	vtkSmartPointer<vtkRenderer> fragment_renderer =
//	//		vtkSmartPointer<vtkRenderer>::New();
//	//	fragment_renderer->AddActor(fragment_actor);
//	//	fragment_renderer->SetBackground(0, 0, 0);
//	//	int col = i % 2;
//	//	int row = int(i / 2);
//	//	//std::cout <<i<<":"<< 0.5 + 0.25 * col << " " << double(row) / 3 << " " << 0.75 + 0.25 * col << " " << 1.0 / 3 + 1.0 / 3 * row << std::endl;
//	//	fragment_renderer->SetViewport(0.5 + 0.25 * col, double(row) / 3, 0.75 + 0.25 * col, 1.0 / 3 + 1.0 / 3 * row);
//	//	renderWindow->AddRenderer(fragment_renderer);
//	//}
//	//
//	//// Render and interact
//	//renderWindow->Render();
//	////renderWindow2->Render();
//	//renderWindowInteractor->Initialize();
//	//renderWindowInteractor->Start();
//
//	return 0;
//}

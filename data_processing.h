#pragma once
#include <vtkSTLReader.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkAppendPolyData.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkActor.h>
#include <vtkMapper.h>
#include <vtkCamera.h>
#include <vtkMath.h>
#include <qstring.h>
#include <qmessagebox.h>
#include <vtkMassProperties.h>
#include <vtkTriangleFilter.h>
#include <file_fillter.h>
#include <vtkDistancePolyDataFilter.h>
#include <vtkPointData.h>
#include <vtkContourFilter.h>
#include <vtkPoints.h>
#include <vtkSurfaceReconstructionFilter.h>
#include <vtkCleanPolyData.h>
#include <vtkKdTree.h>
#include <float.h>
#include <algorithm>
#include <cutter_descriptor.h>
#include <vtkLandmarkTransform.h>
#include <vtkIdList.h>
#include <vtkDoubleArray.h>
#include <vtkTransform.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkActor.h>
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

/*******************************************************************************
* Function Name   : transformSTL2PolyData
* Description	  : Transform STLs to PolyDatas
*
* Input		      : string  file_name， STLs to restore the results
* Return		  : none
*******************************************************************************/
void transformSTL2PolyData(std::vector<std::string>& stl_file_names,std::vector<vtkSmartPointer<vtkPolyData>>& STLs, std::vector<vtkSmartPointer<vtkSTLReader>> &STL_readers)
{
	int j = 0;
	std::cout << "reading files:" << std::endl;
	for (auto i = stl_file_names.begin();i < stl_file_names.end();++i,++j)
	{
		std::string cur_file = *i;
		QString q_file_name = QString::fromStdString(cur_file);
		//支持中文路径s
		QByteArray ba = q_file_name.toLocal8Bit();
		const char* fileName_str = ba.data();
		STL_readers.at(j)->SetFileName(fileName_str);
		STL_readers.at(j)->Update();
		//vtkSmartPointer<vtkPolyData> stl_poly = stl_reader->GetOutput();
		STLs.at(j) = STL_readers.at(j)->GetOutput();
		std::cout <<"----"<< cur_file << std::endl;
	}
}

/*******************************************************************************
* Function Name   : extractSurfacePoints
* Description	  : extract the points in the surface of STL
*
* Input		      : string  pathOfFolder
* Return		  : The polyData of the STL
*******************************************************************************/
void downSample(std::vector<vtkSmartPointer<vtkPolyData>>& STLs, std::vector<vtkSmartPointer<vtkPolyData>>& downSampleSTLs, std::vector<vtkSmartPointer<vtkPoints>> &surface_points, double tolerance)
{
	
	for (size_t i = 0;i < STLs.size();++i)
	{
		/*vtkSmartPointer<vtkPolyData> points =
			vtkSmartPointer<vtkPolyData>::New();
		points->SetPoints(STLs.at(i)->GetPoints());
		vtkSmartPointer<vtkSurfaceReconstructionFilter> sur_fillter =
			vtkSmartPointer<vtkSurfaceReconstructionFilter>::New();
		sur_fillter->SetInputData(points);
		sur_fillter->SetNeighborhoodSize(neighborhood_size);
		sur_fillter->SetSampleSpacing(sample_spacing);
		sur_fillter->Update();*/

		vtkSmartPointer<vtkCleanPolyData> cl_data =
			vtkSmartPointer<vtkCleanPolyData>::New();
		cl_data->SetInputData(STLs.at(i));
		cl_data->SetTolerance(tolerance);
		cl_data->Update();

		/*vtkSmartPointer<vtkContourFilter> sur_contour =
			vtkSmartPointer<vtkContourFilter>::New();
		sur_contour->SetInputConnection(cl_data->GetOutputPort());
		sur_contour->SetValue(0, 0.0);
		sur_contour->Update();*/
		surface_points.at(i) = cl_data->GetOutput()->GetPoints();
		downSampleSTLs.at(i) = cl_data->GetOutput();
		//std::cout << "before reconstruction:" << STLs.at(i)->GetNumberOfPoints() << "  after reconstruction:" << downSampleSTLs.at(i)->GetNumberOfPoints() << std::endl;
	}
}


/*******************************************************************************
* Function Name   : mergeSTLs
* Description	  : Merge STL data as one file
*
* Input		      : string  pathOfFolder
* Return		  : The polyData of the STL
*******************************************************************************/
void mergeSTLs(vtkSmartPointer<vtkPolyData>& whole_STL,std::vector<vtkSmartPointer<vtkPolyData>>& STLs)
{
	vtkSmartPointer<vtkAppendPolyData> STL =
		vtkSmartPointer<vtkAppendPolyData>::New();
	for (auto i = STLs.begin();i < STLs.end();++i)
	{
		STL->AddInputData(i->GetPointer());
	}
	STL->Update();
	//std::cout << "number of points of merge STL file:" << STL->GetOutput()->GetNumberOfPoints() << std::endl;
	whole_STL = STL->GetOutput();
}


/*******************************************************************************
* Function Name   : compute_volum
* Description	  : Compute the volum of the fragments via obb bounding box
*
* Input		      : bone_descriptor* bone, double threshold
* Return		  : The volum of the input fragment 
*******************************************************************************/
double compute_volum(vtkSmartPointer<vtkPolyData> volum)
{
	vtkSmartPointer<vtkTriangleFilter> tri_volum =
		vtkSmartPointer<vtkTriangleFilter>::New();
	tri_volum->SetInputData(volum);
	tri_volum->Update();
	vtkSmartPointer<vtkMassProperties> vtkMP =
		vtkSmartPointer<vtkMassProperties>::New();
	vtkMP->SetInputData(tri_volum->GetOutput());
	vtkMP->Update();
	double V = vtkMP->GetVolume();
	return V;
}

/*******************************************************************************
* Function Name   : compute_distance
* Description	  : Compute the nearest distance between 2 fragments
*
* Input		      : bone_descriptor* bone, double threshold
* Return		  : None
*******************************************************************************/
double compute_distance(vtkSmartPointer<vtkPolyData> reference_volum, vtkSmartPointer<vtkPolyData> target_volum)
{
	double* reference_center = reference_volum->GetCenter();
	double* target_center = target_volum->GetCenter();
	double vector[3] = { reference_center[0] - target_center[0], reference_center[1] - target_center[1], reference_center[2] - target_center[2] };
	return vtkMath::Norm(vector);
	//if (reference_center[0] == target_center[0] && reference_center[1] == target_center[1] && reference_center[2] == target_center[2])
	//	return 0;
	//vtkSmartPointer<vtkDistancePolyDataFilter> dis_fillter =
	//	vtkSmartPointer<vtkDistancePolyDataFilter>::New();
	//dis_fillter->SetInputData(0,reference_volum);
	//dis_fillter->SetInputData(1,target_volum);
	//dis_fillter->Update();
	//double min_distance;
	//min_distance = abs(dis_fillter->GetOutput()->GetPointData()->GetScalars()->GetRange()[0]);//序号0指最小距离，1指最大距离
	//return min_distance;
}

/*******************************************************************************
* Function Name   : getClosestPD
* Description	  : Compute the nearest distance between 2 fragments
*
* Input		      : bone_descriptor* bone, double threshold
* Return		  : None
*******************************************************************************/
void getDistanceIndex(vtkSmartPointer<vtkPolyData> reference_PD, std::vector<vtkSmartPointer<vtkPolyData>>& STLs, std::vector<size_t>& distance_idx, std::vector<int>& tar_index)
{
	std::vector<double> distance(STLs.size());
	for (size_t i = 0;i < STLs.size();++i)
	{
		if (std::find(tar_index.begin(), tar_index.end(), i) == tar_index.end())
		{
			distance.at(i) = compute_distance(reference_PD, STLs.at(i));
			std::cout << i << " distance:" << distance.at(i) << std::endl;
		}
	}
	/**********按照距离大小进行先后匹配*************/
	//scores为std::vector<float> 型数组
	distance_idx.resize(distance.size());
	std::iota(distance_idx.begin(), distance_idx.end(), 0);
	std::sort(distance_idx.begin(), distance_idx.end(),
		[&distance](size_t index_1, size_t index_2) { return distance[index_1] < distance[index_2]; });
	return;
	//distance_idx保存的是数组排完序后下标
}

/*******************************************************************************
* Function Name   : getBlurNormal
* Description	  : get blur normal
*
* Input		      : see the parameters
* Return		  : vtkSmartPointer<vtkDataArray>
*******************************************************************************/
vtkSmartPointer<vtkDoubleArray> getBlurNormal(vtkSmartPointer<vtkKdTree> ref_kd_tree,vtkSmartPointer<vtkDataArray> ref_normals, vtkSmartPointer<vtkPoints> ref_downsample_points,  int k)
{
	vtkSmartPointer<vtkDoubleArray> ref_blur_normals =
		vtkSmartPointer<vtkDoubleArray>::New();
	ref_blur_normals->SetNumberOfTuples(0);
	ref_blur_normals->SetNumberOfComponents(3);
	ref_blur_normals->SetName("blur normal array");
	ref_blur_normals->Initialize();
	int num = ref_downsample_points->GetNumberOfPoints();
	for (int i = 0;i < num;i++)
	{
		double* blur_center_point = ref_downsample_points->GetPoint(i);
		vtkSmartPointer<vtkIdList> kd_p_list =
			vtkSmartPointer<vtkIdList>::New();
		ref_kd_tree->FindClosestNPoints(k, blur_center_point, kd_p_list);
		double raw_normal[3] = { 0 };
		double filtered_normal[3] = { 0 };
		for (int j = 0;j < kd_p_list->GetNumberOfIds();j++)
		{
			raw_normal[0] += ref_normals->GetTuple(kd_p_list->GetId(j))[0];
			raw_normal[1] += ref_normals->GetTuple(kd_p_list->GetId(j))[1];
			raw_normal[2] += ref_normals->GetTuple(kd_p_list->GetId(j))[2];
		}
		vtkMath::Normalize(raw_normal);
		for (int j = 0;j < kd_p_list->GetNumberOfIds();j++)
		{
			double* cur_normal = ref_normals->GetTuple(kd_p_list->GetId(j));
			vtkMath::Normalize(cur_normal);
			if (vtkMath::AngleBetweenVectors(raw_normal, cur_normal) * 180 / vtkMath::Pi() < 10)
			{
				filtered_normal[0] += ref_normals->GetTuple(kd_p_list->GetId(j))[0];
				filtered_normal[1] += ref_normals->GetTuple(kd_p_list->GetId(j))[1];
				filtered_normal[2] += ref_normals->GetTuple(kd_p_list->GetId(j))[2];
			}
		}
		ref_blur_normals->InsertNextTuple(filtered_normal);
	}
	return ref_blur_normals;
}


/*******************************************************************************
* Function Name   : computeVr
* Description	  : Compute the vr of a point
*
* Input		      : bone_descriptor* bone, double threshold
* Return		  : None
*******************************************************************************/
void plot(pcl::PointCloud<pcl::PointXYZ>::Ptr true_tgt_cloudOut)
{
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();

	for (int i = 0; i < true_tgt_cloudOut->size(); i++)
	{
		vtkIdType pid[1];
		pid[0] = points->InsertNextPoint(true_tgt_cloudOut->at(i).x, true_tgt_cloudOut->at(i).y, true_tgt_cloudOut->at(i).z);
		vertices->InsertNextCell(1, pid);
	}



	vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
	polyData->SetPoints(points);
	polyData->SetVerts(vertices);

	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	mapper->SetInputData(polyData);

	vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
	actor->SetMapper(mapper);
	actor->GetProperty()->SetColor(1.0, 0.0, 0.0);
	actor->GetProperty()->SetPointSize(3);

	vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
	renderer->AddActor(actor);
	renderer->SetBackground(.0, .0, .0);//set background

	vtkSmartPointer < vtkRenderWindow> renderwind = vtkSmartPointer < vtkRenderWindow>::New();
	renderwind->AddRenderer(renderer);

	vtkSmartPointer < vtkInteractorStyleTrackballCamera> style = vtkSmartPointer < vtkInteractorStyleTrackballCamera>::New();

	vtkSmartPointer < vtkRenderWindowInteractor> renderwindIt = vtkSmartPointer < vtkRenderWindowInteractor>::New();
	renderwindIt->SetRenderWindow(renderwind);
	renderwindIt->SetInteractorStyle(style);

	renderwind->Render();
	renderwindIt->Start();

}

/*******************************************************************************
* Function Name   : computeVr
* Description	  : Compute the vr of a point
*
* Input		      : bone_descriptor* bone, double threshold
* Return		  : None
*******************************************************************************/
double compute_vr()
{
	return 0;
}

/*******************************************************************************
* Function Name   : getCutterDistance
* Description	  : get the distance distribution of contour
*
* Input		      : vtkSmartPointer<vtkPolyData> ref_pd, double* point, double*normal,std::vector<double> ref_dist
* Return		  : none
*******************************************************************************/
cutter_descriptor* getCutterDistance(vtkSmartPointer<vtkPolyData> ref_pd,  double* ref_point,  double* ref_normal,std::vector<double>& ref_dist)
{
	if(ref_pd == NULL)
	{
		return NULL;
	}
	//建立距离直方图返回统计特征
	vtkSmartPointer<vtkPoints> ref_points = ref_pd->GetPoints();
	//index 0 储存最小距离，index 1 储存最大距离
	if (ref_points == NULL)
	{
		//std::cout << "ref points is NULL!" << std::endl;
		return NULL;
	}
	int ref_num = ref_points->GetNumberOfPoints();
	if (ref_num == 0)
		return NULL;
	//std::cout << "ref num:" << ref_num << std::endl;
	//ref_dist.resize(std::max(ref_num, 1));
	//防止为空
	cutter_descriptor* cutter_des = new cutter_descriptor();
	vtkMath::Normalize(ref_normal);
	if (vtkMath::Norm(ref_normal) == 0)
		std::cout << 1 << std::endl;;
	for (int i = 0;i < ref_num;i++)
	{
		double tmp = 0;
		double* ref_p = ref_points->GetPoint(i);
		double ref_vector[3] = { ref_p[0] - ref_point[0],ref_p[1] - ref_point[1], ref_p[2] - ref_point[2] };
		//tmp = vtkMath::Dot(ref_vector, ref_normal);
		//double distance = sqrt(abs(pow(vtkMath::Norm(ref_vector),2) - tmp * tmp));
		double distance = vtkMath::Norm(ref_vector);
		ref_dist.push_back(distance);
		cutter_des->insertData(i, distance, NULL, NULL, ref_p, ref_point, ref_normal);
	}
	sort(ref_dist.begin(), ref_dist.end());
	return cutter_des;
}
 

/*******************************************************************************
* Function Name   : match_cutter  2022/10/07
* Description	  : offer the matching level of 2 cutter
*
* Input		      : vtkSmartPointer<vtkPolyData> ref_pd, vtkSmartPointer<vtkPolyData> target_pd
* Return		  : matching level
*******************************************************************************/
//double match_score(std::vector<double>& ref_dist, std::vector<double>& tar_dist, cutter_descriptor* ref_des, cutter_descriptor* tar_des,std::vector<double>& key_points, int sparse_num,double bias)
//{
//	std::vector<double> candidate_dist(2);
//	std::vector<std::vector<double>> candidate_ps1;
//	std::vector<std::vector<double>> candidate_ps2;
//	cutter_descriptor* cutter_des1 = NULL;
//	cutter_descriptor* cutter_des2 = NULL;
//	double bias_rate = 1.5;
//	double angle_score_rate = 1;
//	double angle_score = -32400;
//	double length_tor = -3;
//	double cur_score = 0;
//	double score = 0;
//	key_points.resize(18);
//	//根据sparse_num实现构建距离直方分布图
//	if (ref_dist.size() == 0 || tar_dist.size() == 0)
//	{
//		return score;
//	}
//	double dist_range[2] = { std::max(ref_dist.front(),tar_dist.front()),std::min(ref_dist.back(),tar_dist.back()) };
//	if (dist_range[1] <= dist_range[0])
//	{
//		return score;
//	}
//	size_t ref_num = ref_dist.size();
//	size_t tar_num = tar_dist.size();
//	/*int interval_num = (int(std::max(ref_num, tar_num) / sparse_num) + 1);
//	double delta_dist = (dist_range[1] - dist_range[0]) / interval_num;
//	std::vector<int> ref_hist(interval_num);
//	std::vector<int> tar_hist(interval_num);
//	std::vector<int> score_hist(interval_num);*/
//	if (tar_des == NULL)
//	{
//		return score;
//	}
//	if (ref_des == NULL)
//	{
//		return score;
//	}
//	double ref_center[3] = { ref_des->getOrigin()[0], ref_des->getOrigin()[1], ref_des->getOrigin()[2] };
//	double tar_center[3] = { tar_des->getOrigin()[0], tar_des->getOrigin()[1], tar_des->getOrigin()[2] };
//	std::vector<int> fit_points_idx(2);
//	if (ref_num < tar_num)
//	{
//		for (int i = 0;i < ref_num;i++)
//		{
//			if (ref_dist.at(i) > dist_range[1])
//				break;
//			if (ref_dist.at(i) > dist_range[0])
//			{
//				//std::cout << " candidate_ps1.size:" << candidate_ps1.size() << " candidate_ps2.size:" << candidate_ps2.size() << " ref_dist:" << ref_dist.at(i) << " candidate_dist:" << candidate_dist.at(0) << std::endl;
//				if (candidate_ps1.size() == 0)
//				{
//					candidate_dist.at(0) = ref_dist.at(i);
//					tar_des->findCutters(candidate_dist.at(0), candidate_ps1, 0.2);
//					if (candidate_ps1.size() != 0)
//					{
//						cutter_des1 = ref_des->findCutter(ref_dist.at(i));
//						double* pos = cutter_des1->getPosition();
//						double ref_dir_vec[3] = { pos[0] - ref_center[0],pos[1] - ref_center[1],pos[2] - ref_center[2] };
//						double ref_angle = vtkMath::AngleBetweenVectors(ref_dir_vec, cutter_des1->getNormal());
//						for (int j = 0;j < candidate_ps1.size();j++)
//						{
//							double dir_vec[3] = { -candidate_ps1.at(j).at(0) + tar_center[0],-candidate_ps1.at(j).at(1) + tar_center[1], -candidate_ps1.at(j).at(2) + tar_center[2] };
//							double angle = vtkMath::AngleBetweenVectors(dir_vec, tar_des->getNormal());
//							if (abs(ref_angle - angle) / ref_angle > 0.1)
//							{
//								//剔除这个点
//								candidate_ps1.erase(candidate_ps1.begin() + j);
//								--j;
//							}
//						}
//					}
//				}
//				else if (candidate_ps2.size() == 0 && ref_dist.at(i) > bias_rate * candidate_dist.at(0))
//				{
//					//std::cout << "ref_dist:" << ref_dist.at(i) << "    candidate_dist.at(0) " << bias_rate * candidate_dist.at(0) << std::endl;
//					candidate_dist.at(1) = ref_dist.at(i);
//					tar_des->findCutters(candidate_dist.at(1), candidate_ps2,0.2);
//					if (candidate_ps2.size() != 0)
//					{
//						//std::cout << "candidate_ps1.size:" << candidate_ps1.size() << " candidate_ps2.size:" << candidate_ps2.size() << " candidate_dist:" << candidate_dist.at(0) << "  " << candidate_dist.at(1) << std::endl;
//						cutter_des2 = ref_des->findCutter(candidate_dist.at(1));
//						break;
//					}
//				}
//			}
//		}
//		if (!candidate_ps1.size() || !candidate_ps2.size())
//			return score;
//		if (cutter_des1 && cutter_des2)
//		{
//			//std::cout << " candidate_ps1.size:" << candidate_ps1.size() << " candidate_ps2.size:" << candidate_ps2.size() << " candidate_dist:" << candidate_dist.at(0) << "  " << candidate_dist.at(1) << std::endl;
//			double* cutter_p1 = cutter_des1->getPosition();
//			double* cutter_p2 = cutter_des2->getPosition();
//			double ref_vector1[3] = { cutter_p1[0] - ref_center[0],cutter_p1[1] - ref_center[1],cutter_p1[2] - ref_center[2] };
//			//vtkMath::Normalize(ref_vector1);
//			double ref_vector2[3] = { cutter_p2[0] - ref_center[0],cutter_p2[1] - ref_center[1],cutter_p2[2] - ref_center[2] };
//			//vtkMath::Normalize(ref_vector2);
//			double ref_angle = vtkMath::AngleBetweenVectors(ref_vector1, ref_vector2) * 180 / vtkMath::Pi();
//			//double ref_angle = acos(vtkMath::Dot(ref_vector1, ref_vector2)) * 180 / acos(-1);
//			for (int i = 0;i < candidate_ps1.size();i++)
//			{
//				for (int j = 0;j < candidate_ps2.size();j++)
//				{
//					double tar_vector1[3] = { candidate_ps1.at(i).at(0) - tar_center[0],candidate_ps1.at(i).at(1) - tar_center[1],candidate_ps1.at(i).at(2) - tar_center[2] };
//					//vtkMath::Normalize(tar_vector1);
//					double tar_vector2[3] = { candidate_ps2.at(j).at(0) - tar_center[0],candidate_ps2.at(j).at(1) - tar_center[1],candidate_ps2.at(j).at(2) - tar_center[2] };
//					//vtkMath::Normalize(tar_vector2);
//					double tar_angle = vtkMath::AngleBetweenVectors(tar_vector1, tar_vector2) * 180 / vtkMath::Pi();
//					//double tar_angle = acos(vtkMath::Dot(tar_vector1, tar_vector2)) * 180 / acos(-1);
//					angle_score = -abs(tar_angle - ref_angle);
//					//std::cout << "cur_score:" << cur_score <<"	score"<< score << std::endl;
//					//std::cout << "score:" << score << "  cur_score:" << cur_score << std::endl;
//					if (angle_score >= length_tor)
//					{
//						double ref_mid_vec[3] = { (ref_vector1[0] + ref_vector2[0]) / 2, (ref_vector1[1] + ref_vector2[1]) / 2, (ref_vector1[2] + ref_vector2[2]) / 2 };
//						//double ref_mid_dist = vtkMath::Norm(ref_mid_vec) / 2;
//						//cutter_descriptor* ref_closest_des = ref_des->findClosestAngleCutter(ref_mid_vec);
//						double ref_closest_p[3] = { 0 };
//						ref_des->findClosestAngleCutter(ref_mid_vec, ref_closest_p, cutter_p1, cutter_p2);
//						double tar_mid_vec[3] = { (tar_vector1[0] + tar_vector2[0]) / 2, (tar_vector1[1] + tar_vector2[1]) / 2, (tar_vector1[2] + tar_vector2[2]) / 2 };
//						/*double tar_mid_dist = vtkMath::Norm(tar_mid_vec) / 2;
//						cutter_descriptor* tar_closest_des = tar_des->findClosestAngleCutter(tar_mid_vec);*/
//						double tar_closest_p[3] = { 0 };
//						double tar_p1[3] = { candidate_ps1.at(i)[0],candidate_ps1.at(i)[1],candidate_ps1.at(i)[2] };
//						double tar_p2[3] = { candidate_ps2.at(j)[0],candidate_ps2.at(j)[1],candidate_ps2.at(j)[2] };
//						tar_des->findClosestAngleCutter(tar_mid_vec, tar_closest_p, tar_p1, tar_p2);
//						if (vtkMath::Norm(ref_closest_p)* vtkMath::Norm(tar_closest_p) != 0)
//						{
//							double ref_score_vec[3] = { ref_closest_p[0] - ref_center[0],ref_closest_p[1] - ref_center[1] ,ref_closest_p[2] - ref_center[2] };
//							double tar_score_vec[3] = { tar_closest_p[0] - tar_center[0],tar_closest_p[1] - tar_center[1] ,tar_closest_p[2] - tar_center[2] };
//							//std::cout << pow(vtkMath::Norm(ref_score_vec) - vtkMath::Norm(tar_score_vec), 2) + angle_score_rate * abs(vtkMath::AngleBetweenVectors(ref_mid_vec, ref_score_vec) - vtkMath::AngleBetweenVectors(tar_mid_vec, tar_score_vec)) * 180 / vtkMath::Pi() << std::endl;
//							//double cur_score = pow(vtkMath::Norm(ref_score_vec) - vtkMath::Norm(tar_score_vec), 2) + angle_score_rate * abs(vtkMath::AngleBetweenVectors(ref_mid_vec, ref_score_vec) - vtkMath::AngleBetweenVectors(tar_mid_vec, tar_score_vec)) * 180 / vtkMath::Pi();
//							cur_score = 1 / (bias + pow(vtkMath::Norm(ref_score_vec) - vtkMath::Norm(tar_score_vec), 2) + angle_score_rate * abs(vtkMath::AngleBetweenVectors(ref_mid_vec, ref_score_vec) - vtkMath::AngleBetweenVectors(tar_mid_vec, tar_score_vec)) * 180 / vtkMath::Pi());
//						}
//					}
//					if (cur_score > score)
//					{
//						score = cur_score;
//						fit_points_idx.at(0) = i;
//						fit_points_idx.at(1) = j;
//						if (score == 1 / bias)
//							break;
//					}
//				}
//			}
//			for (int i = 0;i < 3;i++)
//			{
//				key_points.at(i) = ref_center[i];
//				key_points.at(i + 3) = cutter_p1[i];
//				key_points.at(i + 6) = cutter_p2[i];
//				key_points.at(i + 9) = tar_center[i];
//				key_points.at(i + 12) = candidate_ps1.at(fit_points_idx.at(0)).at(i);
//				key_points.at(i + 15) = candidate_ps2.at(fit_points_idx.at(1)).at(i);
//			}
//		}
//		//else
//		//{
//		//	//std::cout << "cutter descriptor is NULL!" << std::endl;
//		//}
//	}
//	else if (ref_num >= tar_num)
//	{
//		for (int i = 0;i < tar_num;i++)
//		{
//			if (tar_dist.at(i) > dist_range[1])
//				break;
//			if (tar_dist.at(i) > dist_range[0])
//			{
//				if (candidate_ps1.size() == 0)
//				{
//					candidate_dist.at(0) = tar_dist.at(i);
//					ref_des->findCutters(candidate_dist.at(0), candidate_ps1,0.2);
//					if (candidate_ps1.size() != 0)
//					{
//						cutter_des1 = tar_des->findCutter(tar_dist.at(i));
//						double* pos = cutter_des1->getPosition();
//						double ref_dir_vec[3] = { pos[0] - tar_center[0],pos[1] - tar_center[1],pos[2] - tar_center[2] };
//						double ref_angle = vtkMath::AngleBetweenVectors(ref_dir_vec, cutter_des1->getNormal());
//						for (int j = 0;j < candidate_ps1.size();j++)
//						{
//							double dir_vec[3] = { -candidate_ps1.at(j).at(0) + ref_center[0],-candidate_ps1.at(j).at(1) + ref_center[1], -candidate_ps1.at(j).at(2) + ref_center[2] };
//							double angle = vtkMath::AngleBetweenVectors(dir_vec, ref_des->getNormal());
//							if (abs(ref_angle - angle) / ref_angle > 0.1)
//							{
//								//剔除这个点
//								candidate_ps1.erase(candidate_ps1.begin() + j);
//								--j;
//							}
//						}
//
//					}
//					//std::cout << "find point1:";
//					//std::cout << cutter_des1->getPosition()[0] << " " << cutter_des1->getPosition()[1] << " " << cutter_des1->getPosition()[2] << std::endl;
//				}
//				else if (candidate_ps2.size() == 0 && tar_dist.at(i) > bias_rate * candidate_dist.at(0))
//				{
//					//std::cout << " candidate_ps1.size:" << candidate_ps1.size() << " candidate_ps2.size:" << candidate_ps2.size() << " ref_dist:" << ref_dist.at(i) << " candidate_dist:" << candidate_dist.at(0) << std::endl;
//					candidate_dist.at(1) = tar_dist.at(i);
//					ref_des->findCutters(candidate_dist.at(1), candidate_ps2,0.2);
//					if (candidate_ps2.size() != 0)
//					{
//						cutter_des2 = tar_des->findCutter(tar_dist.at(i));
//						break;
//					}
//					//std::cout << "find point2:";
//					//std::cout << cutter_des2->getPosition()[0] << " " << cutter_des2->getPosition()[1] << " " << cutter_des2->getPosition()[2] << std::endl;
//				}
//			}
//		}
//		if (!candidate_ps1.size() || !candidate_ps2.size())
//			return score;
//		if (cutter_des1 && cutter_des2)
//		{
//			//std::cout << " candidate_ps1.size:" << candidate_ps1.size() << " candidate_ps2.size:" << candidate_ps2.size() << " candidate_dist:" << candidate_dist.at(1) << "  " << candidate_dist.at(0) << std::endl;
//			double cutter_p1[3] = { cutter_des1->getPosition()[0], cutter_des1->getPosition()[1], cutter_des1->getPosition()[2] };
//			//std::cout << cutter_p1[0] << " " << cutter_p1[1] << " " << std::endl;
//			double cutter_p2[3] = { cutter_des2->getPosition()[0], cutter_des2->getPosition()[1], cutter_des2->getPosition()[2] };
//			//std::cout << cutter_p2[0] << " " << cutter_p2[1] << " " << std::endl;
//			double tar_vector1[3] = { cutter_p1[0] - tar_center[0],cutter_p1[1] - tar_center[1],cutter_p1[2] - tar_center[2] };
//			//vtkMath::Normalize(tar_vector1);
//			double tar_vector2[3] = { cutter_p2[0] - tar_center[0],cutter_p2[1] - tar_center[1],cutter_p2[2] - tar_center[2] };
//			//vtkMath::Normalize(tar_vector2);
//			double ref_angle = vtkMath::AngleBetweenVectors(tar_vector1, tar_vector2) * 180 / vtkMath::Pi();
//			//double ref_angle = acos(vtkMath::Dot(tar_vector1, tar_vector2)) * 180 / acos(-1);
//			for (int i = 0;i < candidate_ps1.size();i++)
//			{
//				for (int j = 0;j < candidate_ps2.size();j++)
//				{
//					double ref_vector1[3] = { candidate_ps1.at(i).at(0) - ref_center[0],candidate_ps1.at(i).at(1) - ref_center[1],candidate_ps1.at(i).at(2) - ref_center[2] };
//					//vtkMath::Normalize(ref_vector1);
//					double ref_vector2[3] = { candidate_ps2.at(j).at(0) - ref_center[0],candidate_ps2.at(j).at(1) - ref_center[1],candidate_ps2.at(j).at(2) - ref_center[2] };
//					//vtkMath::Normalize(ref_vector2);
//					double tar_angle = vtkMath::AngleBetweenVectors(ref_vector1, ref_vector2) * 180 / vtkMath::Pi();
//					//double tar_angle = acos(vtkMath::Dot(ref_vector1, ref_vector2)) * 180 / acos(-1);
//					angle_score = -abs(tar_angle - ref_angle);
//					//std::cout << "cur_score:" << cur_score << "	score" << score << std::endl;
//					//std::cout <<"score:"<<score<< "  cur_score:" << cur_score << std::endl;
//					if (angle_score >= length_tor && tar_angle < 150)
//					{
//						double ref_mid_vec[3] = { (ref_vector1[0] + ref_vector2[0]) / 2, (ref_vector1[1] + ref_vector2[1]) / 2, (ref_vector1[2] + ref_vector2[2]) / 2 };
//						/*double ref_mid_dist = vtkMath::Norm(ref_mid_vec) / 2;
//						cutter_descriptor* ref_closest_des = ref_des->findClosestAngleCutter(ref_mid_vec);*/
//						double ref_closest_p[3] = { 0 };
//						double ref_p1[3] = { candidate_ps1.at(i)[0],candidate_ps1.at(i)[1],candidate_ps1.at(i)[2] };
//						double ref_p2[3] = { candidate_ps2.at(j)[0],candidate_ps2.at(j)[1],candidate_ps2.at(j)[2] };
//						ref_des->findClosestAngleCutter(ref_mid_vec, ref_closest_p, ref_p1, ref_p2);
//						double tar_mid_vec[3] = { (tar_vector1[0] + tar_vector2[0]) / 2, (tar_vector1[1] + tar_vector2[1]) / 2, (tar_vector1[2] + tar_vector2[2]) / 2 };
//						/*double tar_mid_dist = vtkMath::Norm(tar_mid_vec) / 2;
//						cutter_descriptor* tar_closest_des = tar_des->findClosestAngleCutter(tar_mid_dist);*/
//						double tar_closest_p[3] = { 0 };
//						tar_des->findClosestAngleCutter(tar_mid_vec, tar_closest_p, cutter_p1, cutter_p2);
//						if (vtkMath::Norm(ref_closest_p) * vtkMath::Norm(tar_closest_p) != 0)
//						{
//							double ref_score_vec[3] = { ref_closest_p[0] - ref_center[0],ref_closest_p[1] - ref_center[1] ,ref_closest_p[2] - ref_center[2] };
//							double tar_score_vec[3] = { tar_closest_p[0] - tar_center[0],tar_closest_p[1] - tar_center[1] ,tar_closest_p[2] - tar_center[2] };
//							//std::cout << pow(vtkMath::Norm(ref_score_vec) - vtkMath::Norm(tar_score_vec), 2) + angle_score_rate * abs(vtkMath::AngleBetweenVectors(ref_mid_vec, ref_score_vec) - vtkMath::AngleBetweenVectors(tar_mid_vec, tar_score_vec)) * 180 / vtkMath::Pi()<< std::endl;
//							//double cur_score = pow(vtkMath::Norm(ref_score_vec) - vtkMath::Norm(tar_score_vec), 2) + angle_score_rate * abs(vtkMath::AngleBetweenVectors(ref_mid_vec, ref_score_vec) - vtkMath::AngleBetweenVectors(tar_mid_vec, tar_score_vec) )* 180 / vtkMath::Pi();
//							cur_score = 1 / (bias + pow(vtkMath::Norm(ref_score_vec) - vtkMath::Norm(tar_score_vec), 2) + angle_score_rate * abs(vtkMath::AngleBetweenVectors(ref_mid_vec, ref_score_vec) - vtkMath::AngleBetweenVectors(tar_mid_vec, tar_score_vec)) * 180 / vtkMath::Pi());
//						}
//					}
//					if (cur_score > score)
//					{
//						score = cur_score;
//						fit_points_idx.at(0) = i;
//						fit_points_idx.at(1) = j;
//						if (score == 1 / bias)
//							break;
//					}
//				}
//			}
//			for (int i = 0;i < 3;i++)
//			{
//				key_points.at(i) = ref_center[i];
//				key_points.at(i + 3) = candidate_ps1.at(fit_points_idx.at(0)).at(i);
//				key_points.at(i + 6) = candidate_ps2.at(fit_points_idx.at(1)).at(i);
//				key_points.at(i + 9) = tar_center[i];
//				key_points.at(i + 12) = cutter_p1[i];
//				key_points.at(i + 15) = cutter_p2[i];
//			}
//		}
//	}
//	return score;
//}

///*******************************************************************************
//* Function Name   : match_cutter
//* Description	  : offer the matching level of 2 cutter
//*
//* Input		      : vtkSmartPointer<vtkPolyData> ref_pd, vtkSmartPointer<vtkPolyData> target_pd
//* Return		  : matching level
//*******************************************************************************/
//double match_score(std::vector<double>& ref_dist, std::vector<double>& tar_dist, cutter_descriptor* ref_des, cutter_descriptor* tar_des, std::vector<double>& key_points, int sparse_num)
//{
//	double score = 0;
//	double sign_idx = 0;
//	double bias = 0.01;
//	double ref_best_idx = 0;
//	double tar_beat_idx = 0;
//	//根据sparse_num实现构建距离直方分布图
//	if (ref_dist.size() == 0 || tar_dist.size() == 0|| tar_des == NULL|| ref_des == NULL)
//	{
//		return score;
//	}
//	double dist_range[2] = { std::max(ref_dist.front(),tar_dist.front()),std::min(ref_dist.back(),tar_dist.back()) };
//	if (dist_range[1] <= dist_range[0])
//	{
//		return score;
//	}
//	size_t ref_num = ref_dist.size();
//	size_t tar_num = tar_dist.size();
//	double ref_center[3] = { ref_des->getOrigin()[0], ref_des->getOrigin()[1], ref_des->getOrigin()[2] };
//	double tar_center[3] = { tar_des->getOrigin()[0], tar_des->getOrigin()[1], tar_des->getOrigin()[2] };
//	if (ref_num < tar_num)
//	{
//		int effect_num = int(ref_num / 3);
//		for (int i = 0;i <effect_num;i++)
//		{
//			if (ref_dist.at(i) > dist_range[1])
//				break;
//			for (int j = sign_idx;j < tar_num;j++)
//			{
//				double delta_dist = tar_dist.at(j) - ref_dist.at(i);
//				if (delta_dist >= 0)
//				{
//					if (j == sign_idx)
//					{
//						sign_idx = j + 1;
//						score = score + 1 / (delta_dist + bias);
//						break;
//					}
//					double last_delta_dist = tar_dist.at(j - 1) - ref_dist.at(i);
//					if (abs(delta_dist) < abs(last_delta_dist))
//					{
//						sign_idx = j + 1;
//						score = score + 1 / (delta_dist + bias);
//						break;
//					}
//					else
//					{
//						sign_idx = j;
//						score = score + 1 / (abs(last_delta_dist) + bias);
//						break;
//					}
//				}
//			}
//		}
//		score = score / effect_num * log10(effect_num);
//	}
//	else
//	{
//		int effect_num = int(tar_num / 3);
//		for (int i = 0;i < effect_num;i++)
//		{
//			if (tar_dist.at(i) > dist_range[1])
//				break;
//			for (int j = sign_idx;j < ref_num;j++)
//			{
//				double delta_dist = ref_dist.at(j) - tar_dist.at(i);
//				if (delta_dist >= 0)
//				{
//					if (j == sign_idx)
//					{
//						sign_idx = j + 1;
//						score = score + 1 / (delta_dist + bias);
//						break;
//					}
//					double last_delta_dist = ref_dist.at(j - 1) - tar_dist.at(i);
//					if (abs(delta_dist) < abs(last_delta_dist))
//					{
//						sign_idx = j + 1;
//						score = score + 1 / (delta_dist + bias);
//						break;
//					}
//					else
//					{
//						sign_idx = j;
//						score = score + 1 / (abs(last_delta_dist) + bias);
//						break;
//					}
//				}
//			}
//		}
//		score = score / effect_num * log10(effect_num);
//	}
//	
//	return score;
//}

/*******************************************************************************
* Function Name   : match_cutter  2022/10/16
* Description	  : offer the matching level of 2 cutter
*
* Input		      : vtkSmartPointer<vtkPolyData> ref_pd, vtkSmartPointer<vtkPolyData> target_pd
* Return		  : matching level
*******************************************************************************/
double match_score(vtkSmartPointer<vtkPolyData> ref_cutter_PD, vtkSmartPointer<vtkPolyData> tar_cutter_PD, vtkSmartPointer<vtkKdTree> ref_kd_tree, std::vector<double>& ref_dist,std::vector<double>& tar_dist, cutter_descriptor* ref_des, cutter_descriptor* tar_des, vtkSmartPointer<vtkMatrix4x4>& tf_matrix, double delta_angle, double bias)
{
	//double cur_score = 0;
	double best_score = 0;
	vtkSmartPointer<vtkPoints> ref_points = ref_cutter_PD->GetPoints();
	int minNumOfPoints = std::min(int(tar_dist.size()), int(ref_points->GetNumberOfPoints()));
	if (minNumOfPoints <= 50 || tar_des == NULL || ref_des == NULL)
	{
		//std::cout << "last_score:" << best_score << std::endl;
		return best_score;
	}

	double* ref_normal = ref_des->getNormal();
	double* tar_normal = tar_des->getNormal();
	double rotate_axis[3] = { 1,0,0 };
	vtkMath::Cross(ref_normal, tar_normal, rotate_axis);
	vtkMath::Normalize(tar_normal);
	vtkMath::Normalize(ref_normal);
	double match_angle = 180 - acos(vtkMath::Dot(ref_normal, tar_normal)) * 180 / acos(-1);
	vtkSmartPointer<vtkTransform> tf =
		vtkSmartPointer<vtkTransform>::New();
	tf->PostMultiply();
	tf->Translate(-tar_des->getOrigin()[0], -tar_des->getOrigin()[1], -tar_des->getOrigin()[2]);
	tf->RotateWXYZ(match_angle, rotate_axis);
	tf->Update();

	//tf->RotateWXYZ(delta_angle, ref_normal);
	vtkSmartPointer<vtkMatrix4x4> tf_mtx = tf->GetMatrix();
	cutter_descriptor* tmp_tar_des = tar_des->transform(tf_mtx);
	double ref_center[3] = { 0 };
	double tar_center[3] = { 0 };
	for (int i = 0;i < minNumOfPoints;i++)
	{
		double cur_ref_dist = ref_dist.at(i);
		double* cur_ref_point = ref_des->findCutter(cur_ref_dist)->getPosition();
		ref_center[0] += cur_ref_point[0];
		ref_center[1] += cur_ref_point[1];
		ref_center[2] += cur_ref_point[2];

		double cur_tar_dist = tar_dist.at(i);
		double* cur_tar_point = tar_des->findCutter(cur_tar_dist)->getPosition();
		tar_center[0] += cur_tar_point[0];
		tar_center[1] += cur_tar_point[1];
		tar_center[2] += cur_tar_point[2];
	}
	for (int i = 0;i < 3;i++)
	{
		ref_center[i] = ref_center[i] / minNumOfPoints;
		tar_center[i] = tar_center[i] / minNumOfPoints;
	}
	double ref_center_vec[3] = { ref_center[0] - ref_des->getOrigin()[0],ref_center[1] - ref_des->getOrigin()[1],ref_center[2] - ref_des->getOrigin()[2] };
	double tar_center_vec[3] = { tar_center[0] - tar_des->getOrigin()[0],tar_center[1] - tar_des->getOrigin()[1],tar_center[2] - tar_des->getOrigin()[2] };
	double axis_center_vec[3] = { 1,0,0 };
	vtkMath::Cross(ref_center_vec, tar_center_vec, axis_center_vec);
	double center_angle = -vtkMath::AngleBetweenVectors(ref_center_vec, tar_center_vec) * 180 / vtkMath::Pi();
	vtkSmartPointer<vtkTransform> match_center_tf =
		vtkSmartPointer<vtkTransform>::New();
	match_center_tf->PostMultiply();
	match_center_tf->RotateWXYZ(center_angle, axis_center_vec);
	match_center_tf->Translate(ref_des->getOrigin());
	match_center_tf->Update();
	vtkSmartPointer<vtkMatrix4x4> match_center_mtx = match_center_tf->GetMatrix();
	tmp_tar_des = tmp_tar_des->transform(match_center_mtx);
	
	double RMSE = 0;
	for (int j = 0;j < minNumOfPoints;j++)
	{
		double cur_tar_dist = tar_dist.at(j);
		double* cur_tar_point = tmp_tar_des->findCutter(cur_tar_dist)->getPosition();
		vtkSmartPointer<vtkIdList>  cur_ref_list =
			vtkSmartPointer<vtkIdList>::New();
		ref_kd_tree->FindClosestNPoints(1, cur_tar_point, cur_ref_list);
		double* cur_ref_point = ref_points->GetPoint(cur_ref_list->GetId(0));
		double dist_vector[3] = { cur_ref_point[0] - cur_tar_point[0], cur_ref_point[1] - cur_tar_point[1], cur_ref_point[2] - cur_tar_point[2] };
		double hsdf_distance = vtkMath::Norm(dist_vector);
		RMSE += hsdf_distance;
	}
	RMSE = RMSE / minNumOfPoints;
	best_score = 1 / (RMSE + bias) * log(minNumOfPoints)*log(tar_dist.at((minNumOfPoints-1)));
	vtkSmartPointer<vtkTransform> final_tf =
		vtkSmartPointer<vtkTransform>::New();
	final_tf->PostMultiply();
	final_tf->Translate(-tar_des->getOrigin()[0], -tar_des->getOrigin()[1], -tar_des->getOrigin()[2]);
	final_tf->RotateWXYZ(match_angle, rotate_axis);
	final_tf->RotateWXYZ(center_angle, axis_center_vec);
	final_tf->Translate(ref_des->getOrigin());
	final_tf->Update();
	tf_matrix = final_tf->GetMatrix();
	/*for (int j = 0;j < 4;j++)
	{
		for (int k = 0;k < 4;k++)
		{
			std::cout << tf_matrix->GetElement(j, k) << " ";
		}
		std::cout << std::endl;
	}*/
	
	//std::cout << "ref_origin:" << ref_des->getOrigin()[0] << " " << ref_des->getOrigin()[1] << " " << ref_des->getOrigin()[2] << std::endl;
	//std::cout << "tmp_tar_origin:" << tmp_tar_des->getOrigin()[0] << " " << tmp_tar_des->getOrigin()[1] << " " << tmp_tar_des->getOrigin()[2] << std::endl;
	//vtkMatrix4x4::Multiply4x4(match_center_mtx, tf_mtx, tf_matrix);
	/*if (cur_score > best_score)
	{
		best_score = cur_score;
		tf_matrix = tf_mtx;
	}*/



	/*for (int i = 0;i < 360 / delta_angle;i++)
	{
		tf->RotateWXYZ(delta_angle, ref_normal);
		tf->Translate(ref_des->getOrigin());
		vtkSmartPointer<vtkMatrix4x4> tf_mtx = tf->GetMatrix();
		tmp_tar_des->transform(tf_mtx);
		///get matching level
		double RMSE = 0;
		for (int j = 0;j < minNumOfPoints;j++)
		{
			double cur_tar_dist = tar_dist.at(j);
			double* cur_tar_point = tar_des->findCutter(cur_tar_dist)->getPosition();
			vtkSmartPointer<vtkIdList>  cur_ref_list =
				vtkSmartPointer<vtkIdList>::New();
			ref_kd_tree->FindClosestNPoints(1, cur_tar_point, cur_ref_list);
			double* cur_ref_point = ref_points->GetPoint(cur_ref_list->GetId(0));
			double dist_vector[3] = { cur_ref_point[0] - cur_tar_point[0], cur_ref_point[1] - cur_tar_point[1], cur_ref_point[2] - cur_tar_point[2] };
			double hsdf_distance = vtkMath::Norm(dist_vector);
			RMSE += hsdf_distance;
		}
		RMSE = RMSE / minNumOfPoints;
		cur_score = 1 / (RMSE + bias)*log(minNumOfPoints);
		if (cur_score > best_score)
		{
			best_score = cur_score;
			tf_matrix = tf_mtx;
		}
		tf->Translate(-ref_des->getOrigin()[0], -ref_des->getOrigin()[1], -ref_des->getOrigin()[2]);
	}*/
	//std::cout << "last_score:"<<best_score << std::endl;
	return best_score;
}
#pragma once
//#include <global_setting.h>
#include <volume_descriptor.h>
#include <vector>
#include <Eigen/Core>
#include <vtkPolyData.h>

/*******************************************************************************
* Class Name      : bone_descriptor
* Description	  : a class to restore the properties of the whole bone
*
* Input		      : None
* Output		  : None
* Return		  : None
*******************************************************************************/
class bone_descriptor
{
public:
	double RMSE;
	double HSDF;
	double volum_threshold;
	volume_descriptor* referenceVolum;
	std::vector<volume_descriptor*> fragments;
	vtkSmartPointer<vtkPolyData> origin_polyData = 
		vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkPolyData> final_polyData = 
		vtkSmartPointer<vtkPolyData>::New();

	bone_descriptor(volume_descriptor* reference, vtkSmartPointer<vtkPolyData> origin_polyData = NULL, vtkSmartPointer<vtkPolyData> final_polyData = NULL, double RMSE = 0, double HSDF = 0, double volum_threshold = 0);

	bone_descriptor(vtkSmartPointer<vtkPolyData> origin_polyData = NULL, vtkSmartPointer<vtkPolyData> final_polyData = NULL, double RMSE = 0, double HSDF = 0, double volum_threshold = 0);

	~bone_descriptor();
	
	void setReference(volume_descriptor* ref);

	bool setReference(int index = 0);

	void setFragments(std::vector<volume_descriptor*> fragments);

	void setRMSE(double RMSE);

	void setHSDF(double HSDF);

	void setVolumThreshold(double threshold);

	void setOriginPolyData(vtkSmartPointer<vtkPolyData> originPolyData);

	void setFinalPolyData(vtkSmartPointer<vtkPolyData> finalPolyData);

	double getRMSE();

	double getHSDF();

	double getVolumThreshold();

	size_t getNumberOfFragments();

	volume_descriptor* getReference();

	std::vector<volume_descriptor*> getFragments();

	int getFragmentIndex(volume_descriptor* fragment);

	volume_descriptor* getFragment(int index);

	vtkSmartPointer<vtkPolyData> getOriginPolyData();

	vtkSmartPointer<vtkPolyData> getFinalPolyData();

	void addFragment(volume_descriptor* fragment);

	bool deleteFragment(int index);

	bool deleteFragment(volume_descriptor* fragment);

	void clearFragment();

	void clearAll();

/*******************************************************************************
* Function Name   : volum_HPF
* Description	  : Apply the high-pass fillter to the fragments referring thheir volum
*
* Input		      : bone_descriptor* bone, double threshold
* Return		  : None
*******************************************************************************/
	void volum_HPF();
};
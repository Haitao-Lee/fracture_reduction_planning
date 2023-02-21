#pragma once
#include <single_point_descriptor.h>
#include <surface_descriptor.h>
#include <edge_descriptor.h>
#include <vtkPolyData.h>
#include <vector>
#include <Eigen/Core>
#include <vtkPolyData.h>

/*******************************************************************************
* Class Name      : volum_descriptor
* Description	  : a class to restore the properties of a fragment
*******************************************************************************/
class surface_descriptor;
class edge_descriptor;
class single_point_descriptor;

class volume_descriptor
{
private:
	double volum;
	bool referenceOrFragment;
	double distance2ref;
	Eigen::Matrix4d tf_matrix;
	std::vector<surface_descriptor*> surfaces;
	std::vector<edge_descriptor*> edges;
	std::vector<single_point_descriptor*> points;
	vtkSmartPointer<vtkPolyData> polyData = 
		vtkSmartPointer<vtkPolyData>::New();

public:
	volume_descriptor(double volum = 0, bool reference_fragment = false, double distance2ref = 0, Eigen::Matrix4d tf_matrix = Eigen::MatrixXd::Identity(4, 4), vtkPolyData* polyData = NULL);

	volume_descriptor(std::vector<single_point_descriptor*> points, std::vector<edge_descriptor*> edges, std::vector<surface_descriptor*> surfaces, double volum = 0, bool reference_fragment = false, double distance2ref = 0, Eigen::Matrix4d tf_matrix = Eigen::MatrixXd::Identity(4, 4), vtkPolyData* polyData = NULL);

	~volume_descriptor();

	volume_descriptor& operator=(volume_descriptor& a);

	void setVolum(double volum);

	void setReferenceOrnot(bool reference_frag);

	void setDistance2ref(double distance);

	void setTfMatrix(Eigen::Matrix4d tf_matrix);

	void setSurfaces(std::vector<surface_descriptor*> surfaces);

	void setEdges(std::vector<edge_descriptor*> edges);

	void setPoints(std::vector<single_point_descriptor*> points);

	void setPolyData(vtkSmartPointer<vtkPolyData> polyData);

	double getVolum();

	bool isReferenceOrnot();

	double getDistance2ref();

	Eigen::MatrixX4d getTfMatrix();

	std::vector<surface_descriptor*> getSurfaces();

	//surface_descriptor* getSurface();

	int getSurfaceIndex(surface_descriptor* surface);

	std::vector<edge_descriptor*> getEdges();

	int getEdgeIndex(edge_descriptor* edge);

	std::vector<single_point_descriptor*> getPoints();

	int getPointIndex(double pos[3]);
	
	int getPointIndex(single_point_descriptor* point);

	int getPointIndex(double x, double y, double z);

	vtkSmartPointer<vtkPolyData> getPolyData();
	
	void addSurface(surface_descriptor* surface);

	void addEdge(edge_descriptor* edge);

	void addPoint(single_point_descriptor* point);

	bool deleteSurface(surface_descriptor* surface);

	bool deleteSurface(int index);

	bool deleteEdge(edge_descriptor* edge);

	bool deleteEdge(int index);

	bool deletePoint(int index);

	bool deletePoint(double pos[3]);

	bool deletePoint(double x, double y, double z);

	bool deletePoint(single_point_descriptor* point);

	void clearSurfaces();

	void clearEdges();

	void clearPoints();

	void clearAll();
};
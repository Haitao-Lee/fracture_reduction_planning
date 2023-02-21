#pragma once
#include <vtkCutter.h>
#include <vtkMath.h>
#include <vtkMatrix4x4.h>
#include <vtkSmartPointer.h>;

//class descriptor;
//
//struct node 
//{
//	cutter_descriptor* right;
//	cutter_descriptor* left;
//}node;

class cutter_descriptor
{
private:
	int index;
	double distance;
	//node* node;
	cutter_descriptor* right;
	cutter_descriptor* left;
	double position[3];
	double origin[3];
	double normal[3];

	void setRightData(cutter_descriptor* right);
	void setRightData(int index, double distance, cutter_descriptor* right, cutter_descriptor* left, double* pos, double* center, double* normal);
	void setLeftData(cutter_descriptor* left);
	void setLeftData(int index, double distance, cutter_descriptor* right, cutter_descriptor* left, double* pos, double* center, double* normal);
public:
	cutter_descriptor();
	cutter_descriptor(double pos[3], double center[3], double normal[3], int index = 0, double distance = 0, cutter_descriptor* right = NULL, cutter_descriptor* left = NULL);
	~cutter_descriptor();
	void setIndex(int index);
	void setDistance(double distance);
	void setPosition(double pos[3]);
	void setPosition(double x, double y, double z);
	void setOrigin(double pos[3]);
	void setOrigin(double x, double y, double z);
	void setNormal(double pos[3]);
	void setNormal(double x, double y, double z);
	void insertData(cutter_descriptor* data);
	void insertData(int index, double distance, cutter_descriptor* right, cutter_descriptor* left, double* pos, double* center, double* normal);
	int getIndex();
	double getDistance();
	double* getPosition();
	double* getOrigin();
	double* getNormal();
	void clear();
	cutter_descriptor* findCutter(double distance);
	void findClosestAngleCutter(double vector[3], double result[3],double ref_p1[3],double ref_p2[3], double max_dist = 100, double angle_tolerance = 1, double dist_tolerance = 0.05);
	cutter_descriptor* findClosestDistanceCutter(double distance);
	void findCutters(double distance, std::vector<std::vector<double>>& cutters,double tolerence = 0.05);
	cutter_descriptor* getLeft();
	cutter_descriptor* getRight();
	cutter_descriptor* transform(vtkSmartPointer<vtkMatrix4x4> mtx);
	bool isEmpty();
};
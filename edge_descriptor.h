#pragma once
#include <surface_descriptor.h>
#include <single_point_descriptor.h>
#include <volume_descriptor.h>
#include <vector>


/*******************************************************************************
* Class Name      : edge_descriptor
* Description	  : a class to restore the properties of a edge
*******************************************************************************/
class single_point_descriptor;
class surface_descriptor;

class edge_descriptor
{
private:
	bool frac_or_orig;
	double ws;
	std::vector<single_point_descriptor*> points;
	std::vector<surface_descriptor*> parent_surface;
public:
	edge_descriptor(bool frac_or_orig = false, double ws = 0);

	edge_descriptor(std::vector<single_point_descriptor*> points, bool frac_or_orig = false, double ws = 0);

	edge_descriptor(std::vector<surface_descriptor*> parent_surface, bool frac_or_orig = false, double ws = 0);

	edge_descriptor(std::vector<single_point_descriptor*> points, std::vector<surface_descriptor*> parent_surface, bool frac_or_orig = false, double ws = 0);

	~edge_descriptor();

	edge_descriptor& operator=(edge_descriptor& a);

	void setFractureOrnot(bool frac_or_orig);

	void setWs(double ws);

	void setPoints(std::vector<single_point_descriptor*> points);

	void setParentSurface(std::vector<surface_descriptor*> parent_sur);

	void addParentSurface(surface_descriptor* parent_sur);

	void addPoint(single_point_descriptor* point);

	bool deleteParentSurface(int index);

	bool deletePoint(int index);

	bool deletePoint(double x, double y, double z);

	bool deletePoint(double pos[3]);

	bool isFracOrnot();

	double getWs();

	std::vector<single_point_descriptor*> getPoints();

	single_point_descriptor* getPoint(int index);

	single_point_descriptor* getPoint(double x,double y,double z);

	single_point_descriptor* getPoint(double pos[3]);

	std::vector<surface_descriptor*> getParentSurfaces();

	surface_descriptor* getParentSurface(int index);

	int getPointIndex(single_point_descriptor* point);

	int getPointIndex(double x, double y, double z);

	int getPointIndex(double pos[3]);

	int getSurfaceIndex(surface_descriptor* surface);
};
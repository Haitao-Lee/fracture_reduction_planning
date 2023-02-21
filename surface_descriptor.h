#pragma once
#include <single_point_descriptor.h>
#include <edge_descriptor.h>
#include <volume_descriptor.h>
#include <vector>

/*******************************************************************************
* Class Name      : surface_descriptor
* Description	  : a class to restore the properties of a surface
*******************************************************************************/

//声明
class single_point_descriptor;
class edge_descriptor;
class volume_descriptor;

class surface_descriptor
{
private:
	bool frac_or_orig;
	double s_vol;                                    //volum shapness
	std::vector<single_point_descriptor*>  points;
	std::vector<edge_descriptor*> edges;
	volume_descriptor* parent_volum;

public:
	surface_descriptor(bool frac_or_orig = false, double s_vol = 0);

	surface_descriptor(volume_descriptor* parent_volum = NULL, bool frac_or_orig = false, double s_vol = 0);

	surface_descriptor(std::vector<edge_descriptor*> edges, volume_descriptor* parent_volum = NULL, bool frac_or_orig = false, double s_vol = 0);

	surface_descriptor(std::vector<single_point_descriptor*>  points, volume_descriptor* parent_volum = NULL, bool frac_or_orig = false, double s_vol = 0);

	surface_descriptor(std::vector<single_point_descriptor*>  points, std::vector<edge_descriptor*> edges, volume_descriptor* parent_volum = NULL, bool frac_or_orig = false, double s_vol = 0);

	~surface_descriptor();

	surface_descriptor& operator=(surface_descriptor& a);

	void setFracOrnot(bool frac_or_not);

	void setSvol(double s_vol);

	void setPoints(std::vector<single_point_descriptor*> points);

	void setEdges(std::vector<edge_descriptor*> edges);

	void setVolum(volume_descriptor* parent_volum);

	bool isFracOrnot();

	double getSvol();

	std::vector<single_point_descriptor*> getPoints();

	single_point_descriptor* getPoint(int index);

	single_point_descriptor* getPoint(double x, double y, double z);

	single_point_descriptor* getPoint(double pos[3]);

	int getPointIndex(single_point_descriptor* point);

	int getPointIndex(double x, double y, double z);

	int getPointIndex(double pos[3]);

	void addPoint(single_point_descriptor* point);

	bool deletePoint(int index);

	bool deletePoint(double x, double y, double z);

	bool deletePoint(double pos[3]);

	bool deletePoint(single_point_descriptor* point);

	std::vector<edge_descriptor*> getEdges();

	edge_descriptor* getEdge(int index);

	int getEdgeIndex(edge_descriptor* edge);

	void addEdge(edge_descriptor* edge);

	bool deleteEdge(edge_descriptor* edge);

	bool deleteEdge(int index);

	volume_descriptor* getVolum();
};
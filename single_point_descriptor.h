#pragma once
#include <surface_descriptor.h>
#include <vtkPointData.h>
#include <edge_descriptor.h>

/*******************************************************************************
* Class Name      : single_point_descriptor
* Description	  : a class to restore the properties of a point
*
* Input		      : None
* Output		  : None
* Return		  : None
*******************************************************************************/

//声明
class edge_descriptor;
class surface_descriptor;

class single_point_descriptor
{
private:
	bool surface_ornot;
	bool edge_ornot;
	double pos[3];                                   //position
	double vr;                                       //volum descriptor
	double vdr;                                      //volum distance
	double ekr;                                      //local bending energy
	double ekr_bar;                                  //mean energy
	std::vector<surface_descriptor*> parent_surface;  //the parent surface
	edge_descriptor* parent_edge;                    //the parent edge

public:

	single_point_descriptor(bool surface_ornot = false, bool edge_ornot = 0, double vr = 0, double vdr = 0, double ekr = 0, double ekr_bar = 0, edge_descriptor* parent_edge = NULL);

	single_point_descriptor(double pos[3], bool surface_ornot = false, bool edge_ornot = 0, double vr = 0, double vdr = 0, double ekr = 0, double ekr_bar = 0, edge_descriptor* parent_edge = NULL);

	single_point_descriptor(std::vector<surface_descriptor*> parent_surf, bool surface_ornot = false, bool edge_ornot = 0, double vr = 0, double vdr = 0, double ekr = 0, double ekr_bar = 0, edge_descriptor* parent_edge = NULL);

	single_point_descriptor(double pos[3], std::vector<surface_descriptor*> parent_surf, bool surface_ornot = false, bool edge_ornot = 0, double vr = 0, double vdr = 0, double ekr = 0, double ekr_bar = 0, edge_descriptor* parent_edge = NULL);
	
	~single_point_descriptor();

	single_point_descriptor& operator=(single_point_descriptor& a);

	bool isSurfacePoint();

	bool isEdgePoint();

	double* getPosition();

	double getVr();

	double getVdr();

	double getEkr();

	double getEkrBar();

	std::vector<surface_descriptor*> getParentSurfaces();

	surface_descriptor* getParentSurface(int index);

	int getSurfaceIndex(surface_descriptor* parent_sur);

	edge_descriptor* getParentEdge();

	void setSurfaceOrnot(bool surface_ornot);

	void setEdgeOrnot(bool edge_ornot);

	void setPosition(double pos[3]);

	void setPosition(double x, double y, double z);

	void setVr(double vr);

	void setVdr(double vdr);

	void addParentSurface(surface_descriptor* parent_sur);

	bool deleteParentSurface(int index);

	void clearParentSurface();

	void setParentSurface(std::vector<surface_descriptor*> parent_surf);

	void setParentEdge(edge_descriptor* parent_edge);

	void clearAll();

};
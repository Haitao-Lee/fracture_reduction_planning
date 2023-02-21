#include <volume_descriptor.h>

volume_descriptor::volume_descriptor(double volum, bool reference_fragment, double distance2ref, Eigen::Matrix4d tf_matrix, vtkPolyData* polyData)
{
	this->volum = volum;
	this->referenceOrFragment = reference_fragment;
	this->distance2ref = distance2ref;
	this->tf_matrix = tf_matrix;
	this->polyData = polyData;
}

volume_descriptor::volume_descriptor(std::vector<single_point_descriptor*> points, std::vector<edge_descriptor*> edges, std::vector<surface_descriptor*> surfaces, double volum, bool reference_fragment, double distance2ref, Eigen::Matrix4d tf_matrix, vtkPolyData* polyData)
{
	this->volum = volum;
	this->referenceOrFragment = reference_fragment;
	this->distance2ref = distance2ref;
	this->tf_matrix = tf_matrix;
	this->polyData = polyData;
	this->surfaces.assign(surfaces.begin(), surfaces.end());
	this->edges.assign(edges.begin(), edges.end());
	this->points.assign(points.begin(), points.end());
}

volume_descriptor::~volume_descriptor() 
{
	this->clearAll();
}


volume_descriptor& volume_descriptor::operator=(volume_descriptor& a)
{
	//如果是自身，不变
	if (this == &a)
		return *this;
	static volume_descriptor b(a.points, a.edges, a.surfaces, a.volum, a.referenceOrFragment, a.distance2ref, a.tf_matrix, a.polyData);
	return b;
}

void volume_descriptor::setVolum(double volum)
{
	this->volum = volum;
}

void volume_descriptor::setReferenceOrnot(bool referenceOrnot)
{
	this->referenceOrFragment = referenceOrnot;
}

void volume_descriptor::setDistance2ref(double distance)
{
	this->distance2ref = distance;
}

void volume_descriptor::setTfMatrix(Eigen::Matrix4d tfMatrix)
{
	this->tf_matrix = tfMatrix;
}

void volume_descriptor::setSurfaces(std::vector<surface_descriptor*> surfaces)
{
	this->surfaces.assign(surfaces.begin(), surfaces.end());
}

void volume_descriptor::setEdges(std::vector<edge_descriptor*> edges)
{
	this->edges.assign(edges.begin(), edges.end());
}

void volume_descriptor::setPoints(std::vector<single_point_descriptor*> points)
{
	this->points.assign(points.begin(), points.end());
}

void volume_descriptor::setPolyData(vtkSmartPointer<vtkPolyData> polyData)
{
	this->polyData = polyData;
}

double volume_descriptor::getVolum()
{
	return this->volum;
}

bool volume_descriptor::isReferenceOrnot()
{
	return this->referenceOrFragment;
}

double volume_descriptor::getDistance2ref()
{
	return this->distance2ref;
}

Eigen::MatrixX4d volume_descriptor::getTfMatrix()
{
	return this->tf_matrix;
}

std::vector<surface_descriptor*> volume_descriptor::getSurfaces()
{
	return this->surfaces;
}

int volume_descriptor::getSurfaceIndex(surface_descriptor* surface)
{
	int index = 0;
	int noResult = -1;
	for (auto i = this->surfaces.begin();i < this->surfaces.end();++i, ++index)
	{
		if ((*i)->getSvol() == surface->getSvol() && (*i)->isFracOrnot() == surface->isFracOrnot() && (*i)->getPoints().size() == surface->getPoints().size() && (*i)->getEdges().size() == surface->getEdges().size())
			return index;
	}
	return noResult;
}

std::vector<edge_descriptor*> volume_descriptor::getEdges()
{
	return this->edges;
}

int volume_descriptor::getEdgeIndex(edge_descriptor* edge)
{
	int index = 0;
	int noResult = -1;
	for (auto i = this->edges.begin();i < this->edges.end();++i, ++index)
	{
		if ((*i)->getWs() == edge->getWs() && (*i)->getPoints().size() == edge->getPoints().size())
			return index;
	}
	return noResult;
}

std::vector<single_point_descriptor*> volume_descriptor::getPoints()
{
	return this->points;
}

int volume_descriptor::getPointIndex(double pos[3])
{
	int index = 0;
	int noResult = -1;
	for (auto i = this->points.begin();i < this->points.end();++i, ++index)
	{
		double* cur_pos = (*i)->getPosition();
		if (cur_pos[0] == pos[0] && cur_pos[1] == pos[1] && cur_pos[2] == pos[2])
		{
			return index;
		}
	}
	return noResult;
}

int volume_descriptor::getPointIndex(single_point_descriptor* point)
{
	double* pos = point->getPosition();
	return this->getPointIndex(pos);
}

int volume_descriptor::getPointIndex(double x, double y, double z)
{
	double pos[3] = { x,y,z };
	return this->getPointIndex(pos);
}

vtkSmartPointer<vtkPolyData> volume_descriptor::getPolyData()
{
	return this->polyData;
}

void volume_descriptor::addSurface(surface_descriptor* surface)
{
	this->surfaces.push_back(surface);
}

void volume_descriptor::addEdge(edge_descriptor* edge)
{
	this->edges.push_back(edge);
}

void volume_descriptor::addPoint(single_point_descriptor* point)
{
	this->points.push_back(point);
}

bool volume_descriptor::deleteSurface(int index)
{
	if (index<0 || index>this->surfaces.size() - 1)
	{
		std::cerr << "index error" << std::endl;
		return false;
	}
	auto j = this->surfaces.begin();
	for (int i = 0;i < index;++i)
	{
		++j;
	}
	this->surfaces.erase(j);
	return true;
}

bool volume_descriptor::deleteSurface(surface_descriptor* surface)
{
	int index = this->getSurfaceIndex(surface);
	return this->deleteSurface(index);
}

bool volume_descriptor::deleteEdge(edge_descriptor* edge)
{
	int index = this->getEdgeIndex(edge);
	return this->deleteEdge(index);
}

bool volume_descriptor::deleteEdge(int index)
{
	if (index<0 || index>this->edges.size() - 1)
	{
		std::cerr << "index error" << std::endl;
		return false;
	}
	auto iter = this->edges.begin();
	for (int i = 0;i < index;++i)
	{
		++iter;
	}
	this->edges.erase(iter);
	return true;
}

bool volume_descriptor::deletePoint(int index)
{
	if (index<0 || index>this->points.size() - 1)
	{
		std::cerr << "index error" << std::endl;
		return false;
	}
	auto j = this->points.begin();
	for (int i = 0;i < index;++i)
	{
		++j;
	}
	this->points.erase(j);
	return true;
}

bool volume_descriptor::deletePoint(double x, double y, double z)
{
	double pos[3] = { x,y,z };
	return this->deletePoint(pos);
}

bool volume_descriptor::deletePoint(double pos[3])
{
	for (auto i = this->points.begin();i < this->points.end();++i)
	{
		double* cur_pos = (*i)->getPosition();
		if (cur_pos[0] == pos[0] && cur_pos[1] == pos[1] && cur_pos[2] == pos[2])
		{
			this->points.erase(i);
			return true;
		}
	}
	return false;
}

bool volume_descriptor::deletePoint(single_point_descriptor* point)
{
	int index = this->getPointIndex(point);
	return this->deletePoint(index);
}

void volume_descriptor::clearSurfaces()
{
	this->surfaces.clear();
}

void volume_descriptor::clearEdges()
{
	this->edges.clear();
}

void volume_descriptor::clearPoints()
{
	this->points.clear();
}

void volume_descriptor::clearAll()
{
	this->clearEdges();
	this->clearPoints();
	this->clearSurfaces();
	this->volum = 0;
	this->referenceOrFragment = false;
	this->polyData = NULL;
	this->distance2ref = 0;
	this->tf_matrix = Eigen::MatrixXd::Identity(4, 4);
}
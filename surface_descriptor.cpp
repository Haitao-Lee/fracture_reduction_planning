#include <surface_descriptor.h>

surface_descriptor::surface_descriptor(bool frac_or_orig, double s_vol) {
	this->frac_or_orig = frac_or_orig;
	this->s_vol = s_vol;
}

surface_descriptor::surface_descriptor(volume_descriptor* parent_volum, bool frac_or_orig, double s_vol) {
	this->parent_volum = parent_volum;
	this->frac_or_orig = frac_or_orig;
	this->s_vol = s_vol;
}

surface_descriptor::surface_descriptor(std::vector<edge_descriptor*> edges, volume_descriptor* parent_volum, bool frac_or_orig, double s_vol) {
	this->edges.assign(edges.begin(), edges.end());
	this->frac_or_orig = frac_or_orig;
	this->s_vol = s_vol;
	this->parent_volum = parent_volum;
}

surface_descriptor::surface_descriptor(std::vector<single_point_descriptor*>  points, volume_descriptor* parent_volum, bool frac_or_orig, double s_vol) {
	this->points.assign(points.begin(), points.end());
	this->frac_or_orig = frac_or_orig;
	this->s_vol = s_vol;
	this->parent_volum = parent_volum;
}

surface_descriptor::surface_descriptor(std::vector<single_point_descriptor*>  points, std::vector<edge_descriptor*> edges, volume_descriptor* parent_volum, bool frac_or_orig, double s_vol) {
	this->points.assign(points.begin(), points.end());
	this->edges.assign(edges.begin(), edges.end());
	this->frac_or_orig = frac_or_orig;
	this->s_vol = s_vol;
	this->parent_volum = parent_volum;
}

surface_descriptor::~surface_descriptor() {}

surface_descriptor& surface_descriptor::operator=(surface_descriptor& a)
{
	//如果是自身，不变
	if (this == &a)
		return *this;
	static surface_descriptor b(a.points, a.edges, a.parent_volum, a.frac_or_orig, a.s_vol);
	return b;
}

void surface_descriptor::setFracOrnot(bool frac_ornot)
{
	this->frac_or_orig = frac_ornot;
}

void surface_descriptor::setSvol(double svol)
{
	this->s_vol = svol;
}

void surface_descriptor::setPoints(std::vector <single_point_descriptor*> points)
{
	this->points.assign(points.begin(), points.end());
}

void surface_descriptor::setEdges(std::vector<edge_descriptor*> edges)
{
	this->edges.assign(edges.begin(), edges.end());
}

void surface_descriptor::setVolum(volume_descriptor* volum)
{
	this->parent_volum = volum;
}

bool surface_descriptor::isFracOrnot()
{
	return this->frac_or_orig;
}

double surface_descriptor::getSvol()
{
	return this->s_vol;
}

std::vector<single_point_descriptor*> surface_descriptor::getPoints()
{
	return this->points;
}


single_point_descriptor* surface_descriptor::getPoint(int index)
{
	if (index<0 || index>this->points.size() - 1)
	{
		std::cerr << "index error" << std::endl;
		return NULL;
	}
	auto j = this->points.begin();
	for (int i = 0;i < index;++i)
	{
		++j;
	}
	return *j;
}

single_point_descriptor* surface_descriptor::getPoint(double x, double y, double z)
{
	double pos[3] = { x,y,z };
	return this->getPoint(pos);
}

single_point_descriptor* surface_descriptor::getPoint(double pos[3])
{
	for (auto i = this->points.begin();i < this->points.end();++i)
	{
		double* cur_pos = (*i)->getPosition();
		if (cur_pos[0] == pos[0] && cur_pos[1] == pos[1] && cur_pos[2] == pos[2])
			return *i;
	}
	return NULL;
}

int surface_descriptor::getPointIndex(single_point_descriptor* point)
{
	return this->getPointIndex(point->getPosition());
}

int surface_descriptor::getPointIndex(double x, double y, double z)
{
	double pos[3] = { x,y,z };
	return this->getPointIndex(pos);
}

int surface_descriptor::getPointIndex(double pos[3])
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

void surface_descriptor::addPoint(single_point_descriptor* point)
{
	this->points.push_back(point);
}

bool surface_descriptor::deletePoint(int index)
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

bool surface_descriptor::deletePoint(double x, double y, double z)
{
	double pos[3] = { x,y,z };
	return this->deletePoint(pos);
}

bool surface_descriptor::deletePoint(double pos[3])
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

bool surface_descriptor::deletePoint(single_point_descriptor* point)
{
	int index = this->getPointIndex(point);
	return this->deletePoint(index);
}

std::vector<edge_descriptor*> surface_descriptor::getEdges()
{
	return this->edges;
}

edge_descriptor* surface_descriptor::getEdge(int index)
{
	if (index<0 || index>this->edges.size() - 1)
	{
		std::cerr << "index error" << std::endl;
		return NULL;
	}
	auto iter = this->edges.begin();
	for (int i = 0;i < index;++i)
	{
		++iter;
	}
	return *iter;
}

int surface_descriptor::getEdgeIndex(edge_descriptor* edge)
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

void surface_descriptor::addEdge(edge_descriptor* edge)
{
	this->edges.push_back(edge);
}

bool surface_descriptor::deleteEdge(edge_descriptor* edge)
{
	int index = this->getEdgeIndex(edge);
	return this->deleteEdge(index);
}

bool surface_descriptor::deleteEdge(int index)
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

volume_descriptor* surface_descriptor::getVolum()
{
	return this->parent_volum;
}

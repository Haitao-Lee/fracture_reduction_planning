#include <edge_descriptor.h>

edge_descriptor::edge_descriptor(bool frac_or_orig, double ws)
{
	this->frac_or_orig = frac_or_orig;
	this->ws = ws;

}

edge_descriptor::edge_descriptor(std::vector<single_point_descriptor*> points, bool frac_or_orig, double ws)
{
	this->frac_or_orig = frac_or_orig;
	this->ws = ws;
	this->points.assign(points.begin(), points.end());
}

edge_descriptor::edge_descriptor(std::vector<surface_descriptor*> parent_surface, bool frac_or_orig, double ws)
{
	this->frac_or_orig = frac_or_orig;
	this->ws = ws;
	this->parent_surface.assign(parent_surface.begin(), parent_surface.end());
}

edge_descriptor::edge_descriptor(std::vector<single_point_descriptor*> points, std::vector<surface_descriptor*> parent_surface, bool frac_or_orig, double ws)
{
	this->frac_or_orig = frac_or_orig;
	this->ws = ws;
	this->points.assign(points.begin(), points.end());
	this->parent_surface.assign(parent_surface.begin(), parent_surface.end());
}

edge_descriptor::~edge_descriptor() 
{
	this->parent_surface.clear();
	this->points.clear();
}

edge_descriptor& edge_descriptor::operator=(edge_descriptor& a)
{
	//如果是自身，不变
	if (this == &a)
		return *this;
	static edge_descriptor b(a.points, a.parent_surface, a.frac_or_orig, a.ws);
	return b;
}

void edge_descriptor::setFractureOrnot(bool frac_or_orig)
{
	this->frac_or_orig = frac_or_orig;
}

void edge_descriptor::setWs(double ws)
{
	this->ws = ws;
}

void edge_descriptor::setPoints(std::vector<single_point_descriptor*> points)
{
	this->points.assign(points.begin(), points.end());
}

void edge_descriptor::setParentSurface(std::vector<surface_descriptor*> parent_sur)
{
	this->parent_surface.assign(parent_sur.begin(), parent_sur.end());
}

void edge_descriptor::addParentSurface(surface_descriptor* parent_sur)
{
	this->parent_surface.push_back(parent_sur);
}

void edge_descriptor::addPoint(single_point_descriptor* point)
{
	this->points.push_back(point);
}

bool edge_descriptor::deleteParentSurface(int index)
{
	if (index<0 || index>this->parent_surface.size() - 1)
	{
		std::cerr << "index error" << std::endl;
		return false;
	}
	auto j = this->parent_surface.begin();
	for (int i = 0;i < index;++i)
	{
		++j;
	}
	this->parent_surface.erase(j);
	return true;
}

bool edge_descriptor::deletePoint(int index)
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

bool edge_descriptor::deletePoint(double x,double y,double z)
{
	for (auto i = this->points.begin();i < this->points.end();++i)
	{
		double* cur_pos = (*i)->getPosition();
		if (cur_pos[0] == x && cur_pos[1] == y && cur_pos[2] == z)
		{
			this->points.erase(i);
			return true;
		}
	}
	return false;
}

bool edge_descriptor::deletePoint(double pos[3])
{
	return this -> deletePoint(pos[0], pos[1], pos[2]);
}

bool edge_descriptor::isFracOrnot()
{
	return this->frac_or_orig;
}

double edge_descriptor::getWs()
{
	return this->ws;
}

std::vector<single_point_descriptor*> edge_descriptor::getPoints()
{
	return this->points;
}

single_point_descriptor* edge_descriptor::getPoint(int index)
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

single_point_descriptor* edge_descriptor::getPoint(double x, double y, double z)
{
	double pos[3] = { x,y,z };
	return this->getPoint(pos);
}

single_point_descriptor* edge_descriptor::getPoint(double pos[3])
{
	for (auto i = this -> points.begin();i < this->points.end();++i)
	{
		double* cur_pos = (*i)->getPosition();
		if (cur_pos[0] == pos[0] && cur_pos[1] == pos[1] && cur_pos[2] == pos[2])
			return *i;
	}
	return NULL;
}

std::vector<surface_descriptor*> edge_descriptor::getParentSurfaces()
{
	return this->parent_surface;
}

surface_descriptor* edge_descriptor::getParentSurface(int index)
{
	if (index<0 || index>this->parent_surface.size() - 1)
	{
		std::cerr << "index error" << std::endl;
		return NULL;
	}
	auto j = this->parent_surface.begin();
	for (int i = 0;i < index;++i)
	{
		++j;
	}
	return *j;
}

int edge_descriptor::getPointIndex(single_point_descriptor* point)
{
	double* pos = point->getPosition();
	return this->getPointIndex(pos);
}

int edge_descriptor::getPointIndex(double x, double y, double z)
{
	double pos[3] = { x,y,z };
	return this->getPointIndex(pos);
}

int edge_descriptor::getPointIndex(double pos[3])
{
	int index = 0;
	int noResult = -1;
	for (auto i = this->points.begin();i < this->points.end();++i,++index)
	{
		double* cur_pos = (*i)->getPosition();
		if (cur_pos[0] == pos[0] && cur_pos[1] == pos[1] && cur_pos[2] == pos[2])
		{
			return index;
		}
	}
	return noResult;
}

int edge_descriptor::getSurfaceIndex(surface_descriptor* surface)
{
	int index = 0;
	int noResult = -1;
	for (auto i = this->parent_surface.begin();i < parent_surface.end();++i, ++index)
	{
		if ((*i)->isFracOrnot() == surface->isFracOrnot() && (*i)->getPoints().size() == surface->getPoints().size() && (*i)->getSvol() == surface->getSvol())
			return index;
	}
	return noResult;
}
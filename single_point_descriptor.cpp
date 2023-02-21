#include <single_point_descriptor.h>

single_point_descriptor::single_point_descriptor(bool surface_ornot, bool edge_ornot, double vr, double vdr, double ekr, double ekr_bar, edge_descriptor* parent_edge) {
	this->surface_ornot = surface_ornot;
	this->edge_ornot = edge_ornot;
	this->pos[0] = 0;
	this->pos[1] = 0;
	this->pos[2] = 0;
	this->vr = vr;
	this->vdr = vdr;
	this->ekr = ekr;
	this->ekr_bar = ekr_bar;
	this->parent_edge = parent_edge;
}

single_point_descriptor::single_point_descriptor(double pos[3], bool surface_ornot, bool edge_ornot, double vr, double vdr, double ekr, double ekr_bar, edge_descriptor* parent_edge) {
	this->surface_ornot = surface_ornot;
	this->edge_ornot = edge_ornot;
	this->pos[0] = pos[0];
	this->pos[1] = pos[1];
	this->pos[2] = pos[2];
	this->vr = vr;
	this->vdr = vdr;
	this->ekr = ekr;
	this->ekr_bar = ekr_bar;
	this->parent_edge = parent_edge;
}

single_point_descriptor::single_point_descriptor(std::vector<surface_descriptor*> parent_surf, bool surface_ornot, bool edge_ornot, double vr, double vdr, double ekr, double ekr_bar, edge_descriptor* parent_edge) {
	this->surface_ornot = surface_ornot;
	this->edge_ornot = edge_ornot;
	this->pos[0] = 0;
	this->pos[1] = 0;
	this->pos[2] = 0;
	this->vr = vr;
	this->vdr = vdr;
	this->ekr = ekr;
	this->ekr_bar = ekr_bar;
	this->parent_surface.assign(parent_surf.begin(), parent_surf.end());
	this->parent_edge = parent_edge;
}

single_point_descriptor::single_point_descriptor(double pos[3], std::vector<surface_descriptor*> parent_surf, bool surface_ornot, bool edge_ornot, double vr, double vdr, double ekr, double ekr_bar, edge_descriptor* parent_edge) {
	this->surface_ornot = surface_ornot;
	this->edge_ornot = edge_ornot;
	this->pos[0] = pos[0];
	this->pos[1] = pos[1];
	this->pos[2] = pos[2];
	this->vr = vr;
	this->vdr = vdr;
	this->ekr = ekr;
	this->ekr_bar = ekr_bar;
	this->parent_surface.assign(parent_surf.begin(), parent_surf.end());
	this->parent_edge = parent_edge;
}

single_point_descriptor::~single_point_descriptor()
{
	this->clearAll();
}

single_point_descriptor& single_point_descriptor::operator=(single_point_descriptor& a)
{
	//如果是自身，不变
	if (this == &a)
		return *this;
	static single_point_descriptor b(a.pos, a.parent_surface, a.surface_ornot, a.edge_ornot, a.vr, a.vdr, a.ekr, a.ekr_bar, a.parent_edge);
	return b;
}

bool  single_point_descriptor::isSurfacePoint()
{
	return this->surface_ornot;
}

bool  single_point_descriptor::isEdgePoint()
{
	return this->edge_ornot;
}

double* single_point_descriptor::getPosition()
{
	return this->pos;
}

double single_point_descriptor::getVr()
{
	return this->vr;
}

double single_point_descriptor::getVdr()
{
	return this->vdr;
}

double single_point_descriptor::getEkr()
{
	return this->ekr;
}

double single_point_descriptor::getEkrBar()
{
	return this->ekr_bar;
}

std::vector<surface_descriptor*> single_point_descriptor::getParentSurfaces()
{
	return this->parent_surface;
}

surface_descriptor* single_point_descriptor::getParentSurface(int index)
{
	if (index<0 || index>this->parent_surface.size() - 1)
	{
		std::cerr << "index error" << std::endl;
		return NULL;
	}
	auto i = this->parent_surface.begin();
	for (int j = 0;j < index;j++)
	{
		++i;
	}
	return *i;
}

int single_point_descriptor::getSurfaceIndex(surface_descriptor* parent_sur)
{
	int index = 0;
	int noResult = -1;
	for (auto i = this->parent_surface.begin();i < parent_surface.end();++i, ++index)
	{
		if ((*i)->isFracOrnot() == parent_sur->isFracOrnot() && (*i)->getPoints().size() == parent_sur->getPoints().size() && (*i)->getSvol() == parent_sur->getSvol())
			return index;
	}
	return noResult;
}

edge_descriptor* single_point_descriptor::getParentEdge()
{
	return this->parent_edge;
}

void single_point_descriptor::setSurfaceOrnot(bool surface_ornot)
{
	this->surface_ornot = surface_ornot;
}

void single_point_descriptor::setEdgeOrnot(bool edge_ornot)
{
	this->edge_ornot = edge_ornot;
}

void single_point_descriptor::setPosition(double pos[3])
{
	this->pos[0] = pos[0];
	this->pos[1] = pos[1];
	this->pos[2] = pos[2];
}

void single_point_descriptor::setPosition(double x, double y, double z)
{
	this->pos[0] = x;
	this->pos[1] = y;
	this->pos[2] = z;
}

void single_point_descriptor::setVr(double vr)
{
	this->vr = vr;
}

void single_point_descriptor::setVdr(double vdr)
{
	this->vdr = vdr;
}

void single_point_descriptor::setParentSurface(std::vector<surface_descriptor*> parent_sur)
{
	this->parent_surface.assign(parent_sur.begin(), parent_sur.end());
}

void single_point_descriptor::addParentSurface(surface_descriptor* parent_sur)
{
	this->parent_surface.push_back(parent_sur);
}

bool single_point_descriptor::deleteParentSurface(int index)
{
	if (index<0 || index>this->parent_surface.size() - 1)
	{
		std::cerr << "index error" << std::endl;
		return false;
	}
	auto i = this->parent_surface.begin();
	for (int j = 0;j < index;j++)
	{
		++i;
	}
	this->parent_surface.erase(i);
	return true;
}

void single_point_descriptor::clearParentSurface()
{
	this->parent_surface.clear();
}

void single_point_descriptor::setParentEdge(edge_descriptor* parent_edge)
{
	this->parent_edge = parent_edge;
}

void single_point_descriptor::clearAll()
{
	this->clearParentSurface();
	delete this->parent_edge;
	this->parent_edge = NULL;
	this->surface_ornot = false;
	this->edge_ornot = false;
	this->pos[0] = 0;
	this->pos[1] = 0;
	this->pos[2] = 0;
	this->vr = 0;
	this->vdr = 0;
	this->ekr = 0;
	this->ekr_bar = 0;

}
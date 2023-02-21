#include <bone_descriptor.h>

bone_descriptor::bone_descriptor(volume_descriptor* reference, vtkSmartPointer<vtkPolyData> origin_polyData, vtkSmartPointer<vtkPolyData> final_polyData, double RMSE, double HSDF, double volumThreshold)
{
	this->referenceVolum = reference;
	this->origin_polyData = origin_polyData;
	this->final_polyData = final_polyData;
	this->RMSE = RMSE;
	this->HSDF = HSDF;
	this->volum_threshold = volumThreshold;
}

bone_descriptor::bone_descriptor(vtkSmartPointer<vtkPolyData> origin_polyData, vtkSmartPointer<vtkPolyData> final_polyData, double RMSE, double HSDF, double volumThreshold)
{
	this->origin_polyData = origin_polyData;
	this->final_polyData = final_polyData;
	this->RMSE = RMSE;
	this->HSDF = HSDF;
	this->volum_threshold = volumThreshold;
}

bone_descriptor::~bone_descriptor() 
{
	this->clearAll();
}

void bone_descriptor::setReference(volume_descriptor* ref)
{
	this->referenceVolum = ref;
	if (int i = this->getFragmentIndex(ref) != -1)
	{
		this->fragments.at(i)->setReferenceOrnot(true);
	}
}

bool bone_descriptor::setReference(int index)
{
	if (index<0 || index>this->fragments.size() - 1)
	{
		std::cerr << "index error" << std::endl;
		return false;
	}
	this->referenceVolum = this->getFragment(0);
	return true;
}

void bone_descriptor::setFragments(std::vector<volume_descriptor*> fragments)
{
	this->fragments.assign(fragments.begin(),fragments.end());
}

void bone_descriptor::setRMSE(double rmse)
{
	this->RMSE = rmse;
}

void bone_descriptor::setHSDF(double hsdf)
{
	this->HSDF = hsdf;
}

void bone_descriptor::setVolumThreshold(double threshold)
{
	this->volum_threshold = threshold;
}

void bone_descriptor::setOriginPolyData(vtkSmartPointer<vtkPolyData> originPolydata)
{
	this->origin_polyData = originPolydata;
}

void bone_descriptor::setFinalPolyData(vtkSmartPointer<vtkPolyData> finalPolyData)
{
	this->final_polyData = finalPolyData;
}

double bone_descriptor::getRMSE()
{
	return this->RMSE;
}

double bone_descriptor::getHSDF()
{
	return this->HSDF;
}

double  bone_descriptor::getVolumThreshold()
{
	return this->volum_threshold;
}

size_t bone_descriptor::getNumberOfFragments()
{
	return this->getFragments().size();
}

volume_descriptor* bone_descriptor::getReference()
{
	return this->referenceVolum;
}

std::vector<volume_descriptor*> bone_descriptor::getFragments()
{
	return this->fragments;
}

int bone_descriptor::getFragmentIndex(volume_descriptor* fragment)
{
	int index = 0;
	int noResult = -1;
	for (auto i = this->fragments.begin();i < this->fragments.end();++i, ++index)
	{
		if ((*i)->getVolum() == fragment->getVolum()
			&& (*i)->getDistance2ref() == fragment->getDistance2ref()
			&& (*i)->getPoints().size() == fragment->getPoints().size())
		{
			return index;
		}
	}
	return noResult;
}

volume_descriptor* bone_descriptor::getFragment(int index)
{
	if (index<0 || index>this->fragments.size() - 1)
	{
		std::cerr << "index error" << std::endl;
		return NULL;
	}
	auto j = this->fragments.begin();
	for (int i = 0;i < index;++i)
	{
		++j;
	}
	return *j;
}

vtkSmartPointer<vtkPolyData> bone_descriptor::getOriginPolyData()
{
	return this->origin_polyData;
}

vtkSmartPointer<vtkPolyData> bone_descriptor::getFinalPolyData()
{
	return this->final_polyData;
}

void bone_descriptor::addFragment(volume_descriptor* fragment)
{
	this->fragments.push_back(fragment);
}

bool bone_descriptor::deleteFragment(int index)
{
	if (index<0 || index>this->fragments.size() - 1)
	{
		std::cerr << "index error" << std::endl;
		return false;
	}
	auto j = this->fragments.begin();
	for (int i = 0;i < index;++i)
	{
		++j;
	}
	this->fragments.erase(j);
	return true;
}

bool bone_descriptor::deleteFragment(volume_descriptor* fragment)
{
	int index = this->getFragmentIndex(fragment);
	return this->deleteFragment(index);
}

void bone_descriptor::clearFragment()
{
	this->fragments.clear();
}

void bone_descriptor::clearAll()
{
	this->clearFragment();
	this->RMSE = 0;
	this->HSDF = 0;
	this->referenceVolum = NULL;
	this->origin_polyData = NULL;
	this->final_polyData = NULL;
}

void bone_descriptor::volum_HPF()
{
	int index = 0;
	for (auto i = this->getFragments().begin();i < this -> getFragments().end();++i,++index)
	{
		if ((*i)->getVolum() < this->volum_threshold)
		{
			this->deleteFragment(index);
		}
	}
}
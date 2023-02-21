#include <cutter_descriptor.h>

cutter_descriptor::cutter_descriptor()
{
	this->index = 0;
	this->distance = 0;
	this->right = NULL;
	this->left = NULL;
	for (int i = 0;i < 3;i++)
	{
		this->position[i] = 0;
		this->origin[i] = 0;
		this->normal[i] = 0;
	}
	//std::cout << "create cutterdescriptor!" << std::endl;
}

cutter_descriptor::cutter_descriptor(double pos[3], double center[3], double normal[3],int index, double distance, cutter_descriptor* right, cutter_descriptor* left)
{
	this->index = index;
	this->distance = distance;
	this->right = right;
	this->left = left;
	for (int i = 0;i < 3;i++)
	{
		this->position[i] = pos[i];
		this->origin[i] = center[i];
		this->normal[i] = normal[i];
	}
}

cutter_descriptor::~cutter_descriptor()
{
	std::cout << "clear!" << std::endl;
	this->clear();
}

void cutter_descriptor::setRightData(cutter_descriptor* right)
{
	this->right = right;
}

void cutter_descriptor::setRightData(int index, double distance, cutter_descriptor* right, cutter_descriptor* left, double* pos, double* center, double* normal)
{
	this->right->index = index;
	this->right->distance = distance;
	this->right->right = right;
	this->right->left = left;
	for (int i = 0;i < 3;i++)
	{
		this->right->position[i] = pos[i];
		this->right->origin[i] = center[i];
		this->right->normal[i] = normal[i];
	}
}

void cutter_descriptor::setLeftData(cutter_descriptor* left)
{
	this->left = left;
}

void cutter_descriptor::setLeftData(int index, double distance, cutter_descriptor* right, cutter_descriptor* left, double* pos, double* center, double* normal)
{
	this->left->index = index;
	this->left->distance = distance;
	this->left->right = right;
	this->left->left = left;
	for (int i = 0;i < 3;i++)
	{
		this->left->position[i] = pos[i];
		this->left->origin[i] = center[i];
		this->left->normal[i] = normal[i];
	}
}

void cutter_descriptor::setIndex(int index)
{
	this->index = index;
}

void cutter_descriptor::setDistance(double distance)
{
	this->distance = distance;
}

void cutter_descriptor::setPosition(double pos[3])
{
	this->position[0] = pos[0];
	this->position[1] = pos[1];
	this->position[2] = pos[2];
}

void cutter_descriptor::setPosition(double x,double y,double z)
{
	this->position[0] = x;
	this->position[1] = y;
	this->position[2] = z;
}

void cutter_descriptor::setOrigin(double pos[3])
{
	this->origin[0] = pos[0];
	this->origin[1] = pos[1];
	this->origin[2] = pos[2];
}

void cutter_descriptor::setOrigin(double x, double y, double z)
{
	this->origin[0] = x;
	this->origin[1] = y;
	this->origin[2] = z;
}

void cutter_descriptor::setNormal(double pos[3])
{
	this->normal[0] = pos[0];
	this->normal[1] = pos[1];
	this->normal[2] = pos[2];
}

void cutter_descriptor::setNormal(double x, double y, double z)
{
	this->normal[0] = x;
	this->normal[1] = y;
	this->normal[2] = z;
}

void cutter_descriptor::insertData(cutter_descriptor* data)
{
	cutter_descriptor* tmp = this;
	if (vtkMath::Norm(this->normal) == 0)
	{
		this->index = data->index;
		this->distance = data->distance;
		this->right = data->right;
		this->left = data->left;
		for (int i = 0;i < 3;i++)
		{
			this->position[i] = data->position[i];
			this->origin[i] = data->origin[i];
			this->normal[i] = data->normal[i];
		}
		return;
	}
	while (1)
	{
		if ((data->distance) >= (tmp->distance))
		{
			if (tmp->right == NULL)
			{
				tmp->right = new cutter_descriptor();
				tmp->setRightData(data);
				break;
			}
			tmp = tmp->right;
		}
		if (data->distance < tmp->distance)
		{
			if (tmp->left == NULL)
			{
				tmp->left = new cutter_descriptor();
				tmp->setLeftData(data);
				break;
			}	
			tmp = tmp->left;
		}
	}
}

void cutter_descriptor::insertData(int index, double distance, cutter_descriptor* right, cutter_descriptor* left, double* pos, double* center, double* normal)
{
	cutter_descriptor* tmp = this;
	if (tmp == NULL)
		tmp = new cutter_descriptor();
	//std::cout << vtkMath::Norm(tmp->normal) << std::endl;
	if (vtkMath::Norm(tmp->normal) == 0)
	{
		this->index = index;
		this->distance = distance;
		this->right = right;
		this->left = left;
		for (int i = 0;i < 3;i++)
		{
			this->position[i] = pos[i];
			this->origin[i] = center[i];
			this->normal[i] = normal[i];
		}
		return;
	}
	while (1)
	{
		if (tmp->distance <= distance)
		{
			if (tmp->right == NULL)
			{
				tmp->right = new cutter_descriptor();
				tmp->setRightData(index, distance, right, left, pos,center, normal);
				break;
			}
			tmp = tmp->right;
		}
		if (tmp->distance>distance)
		{
			if (tmp->left == NULL)
			{
				tmp->left = new cutter_descriptor();
				tmp->setLeftData(index, distance, right, left, pos, center, normal);
				break;
			}
			tmp = tmp->left;
		}
	}
}

int cutter_descriptor::getIndex()
{
	return this->index;
}

double cutter_descriptor::getDistance()
{
	return this->distance;
}

double* cutter_descriptor::getPosition()
{
	return this->position;
}

double* cutter_descriptor::getOrigin()
{
	return this->origin;
}

double* cutter_descriptor::getNormal()
{
	return this->normal;
}

void cutter_descriptor::clear()
{
	
	if (this->right)
		this->right->clear();
	if (this->left)
		this->left->clear();
	delete this->right;
	delete this->left;
	this->right = NULL;
	this->left = NULL;
	//cutter_descriptor* tmp = this;
	//delete tmp;
	return;
}

cutter_descriptor* cutter_descriptor::findCutter(double distance)
{
	cutter_descriptor* tmp = this;
	cutter_descriptor* NO_RESULT = NULL;
	while (1)
	{
		//std::cout << "tmp->distance:" << tmp->distance << " distance:" << distance << std::endl;
		if (tmp->distance == distance)
		{
			//std::cout << "success!" << std::endl;
			return tmp;
		}
		if (distance > tmp->distance)
		{
			tmp = tmp->right;
			if (tmp == NULL)
			{
				//std::cout << "no result!" << std::endl;
				return NO_RESULT;
			}	
		}
		else if (distance < tmp->distance)
		{
			tmp = tmp->left;
			if (tmp == NULL)
			{
				//std::cout << "no result!" << std::endl;
				return NO_RESULT;
			}
		}
	}
}

void cutter_descriptor::findClosestAngleCutter(double vector[3], double result[3], double ref_p1[3],double ref_p2[3], double max_dist,double angle_torlerance, double dist_torlerance)
{
	double cur_dist = vtkMath::Norm(vector);
	while (vtkMath::Norm(result) == 0&& cur_dist<= max_dist)
	{
		std::vector<std::vector<double>> tmp_pos;
		this->findCutters(cur_dist, tmp_pos, dist_torlerance);
		for (int i = 0;i < tmp_pos.size();i++)
		{
			double vec[3] = { tmp_pos.at(i).at(0) - this->getOrigin()[0],tmp_pos.at(i).at(1) - this->getOrigin()[1], tmp_pos.at(i).at(2) - this->getOrigin()[2] };
			double delta_angle = vtkMath::AngleBetweenVectors(vec, vector) * 180 / vtkMath::Pi();
			double vec1[3] = { ref_p1[0] - tmp_pos.at(i).at(0),ref_p1[1] - tmp_pos.at(i).at(1),ref_p1[2] - tmp_pos.at(i).at(2) };
			double vec2[3] = { ref_p2[0] - tmp_pos.at(i).at(0),ref_p2[1] - tmp_pos.at(i).at(1),ref_p2[2] - tmp_pos.at(i).at(2) };
			double effect_angle = vtkMath::AngleBetweenVectors(vec1, vec2) * 180 / vtkMath::Pi();
			if (abs(delta_angle) < angle_torlerance && abs(effect_angle) < 150)
			{
				result[0] = tmp_pos.at(i).at(0);
				result[1] = tmp_pos.at(i).at(1);
				result[2] = tmp_pos.at(i).at(2);
				break;
			}
		}
		cur_dist = cur_dist + dist_torlerance;
	}
	return;
}

cutter_descriptor* cutter_descriptor::findClosestDistanceCutter(double distance)
{
	cutter_descriptor* tmp = this;
	double cur_dist = 0;
	double closest_dist = 100000;
	cutter_descriptor* closest_des = tmp;
	while (1)
	{
		//std::cout << "tmp->distance:" << tmp->distance << " distance:" << distance << std::endl;
		cur_dist = tmp->getDistance() - distance;
		if (abs(cur_dist) < abs(closest_dist))
		{
			closest_des = tmp;
		}
		if (cur_dist == 0)
			break;
		else if (cur_dist > 0)
		{
			if (tmp->left == NULL)
				break;
			tmp = tmp->left;
		}
		else
		{
			if (tmp->right == NULL)
				break;
			tmp = tmp->right;
		}
	}
	return closest_des;
}



void cutter_descriptor::findCutters(double distance, std::vector<std::vector<double>>& cutters, double tolerence)
{ 
	if (this == NULL)
	{
		//std::cout <<"cutter size:"<<cutters.size() << std::endl;
		return;
	}
	cutter_descriptor* tmp = this;
	if (abs(tmp->distance - distance) <= tolerence)
	{
		//std::cout << "distance:" << tmp->distance << " " << distance << std::endl;
		std::vector<double> pos(3);
		pos.at(0) = tmp->position[0];
		pos.at(1) = tmp->position[1];
		pos.at(2) = tmp->position[2];
		cutters.push_back(pos);
		tmp->right->findCutters(distance, cutters, tolerence);
		tmp->left->findCutters(distance, cutters, tolerence);
	}
	else if (tmp->distance > distance)
	{
		tmp->left->findCutters(distance, cutters, tolerence);
	}
	else if (tmp->distance < distance)
	{
		tmp->right->findCutters(distance, cutters, tolerence);
	}
}

cutter_descriptor* cutter_descriptor::getLeft()
{
	return this->left;
}

cutter_descriptor* cutter_descriptor::getRight()
{
	return this->right;
}

cutter_descriptor* cutter_descriptor::transform(vtkSmartPointer<vtkMatrix4x4> mtx)
{
	if (this == NULL)
		return NULL;
	cutter_descriptor* new_des = new cutter_descriptor();
	/*for (int j = 0;j < 4;j++)
	{
		for (int k = 0;k < 4;k++)
		{
			std::cout << mtx->GetElement(j, k) << " ";
		}
		std::cout << std::endl;
	}*/
	double cur_position[4] = { this->getPosition()[0],this->getPosition()[1], this->getPosition()[2], 1 };
	double new_position[4] = { 0 };
	mtx->MultiplyPoint(cur_position, new_position);
	//new_des->setPosition(new_position[0], new_position[1], new_position[2]);
	double cur_origin[4] = { this->getOrigin()[0],this->getOrigin()[1], this->getOrigin()[2], 1 };
	double new_origin[4] = { 0 };
	mtx->MultiplyPoint(cur_origin, new_origin);
	//new_des->setOrigin(new_origin[0], new_origin[1], new_origin[2]);
	double cur_normal[4] = { this->getNormal()[0],this->getNormal()[1], this->getNormal()[2], 0 };
	double new_normal[4] = { 0 };
	mtx->MultiplyPoint(cur_normal, new_normal);
	new_des->insertData(this->getIndex(), this->getDistance(), NULL, NULL, new_position, new_origin, new_normal);
	if (this->right != NULL)
	{
		new_des->insertData(this->right->transform(mtx));
	}
	if (this->left != NULL)
	{
		new_des->insertData(this->left->transform(mtx));
	}
	return new_des;
}

bool cutter_descriptor::isEmpty()
{
	return vtkMath::Norm(this->getNormal());
}
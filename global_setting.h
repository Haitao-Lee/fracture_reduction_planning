#pragma once
#include <string.h>
#include <iostream>

namespace global_setting{
	double r_min;
	double r_max;
	double k0;
	double r0;
	double volum_threshold = 0;
	double tolerance = 0.05;
	double cutter_offset = 1;
	double bias = 0.0001;
	double score_tolerance = 0.001;
	double delta_angle = 5;
	int sparse_num = 1;
	int kdTree_num = 100;
	long int offset_num = 0;
	std::string file_name = "E:/prp/fracture restoration/data/Pelvic_marked/009";
	std::string format = ".stl";
};


   
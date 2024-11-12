#pragma once
#include "headfile.h"

double Feature_pooling(std::vector< SubFeature> feature, 
	std::string results_file, 
	std::string dir_reference, 
	std::string dir_reference_png, 
	std::string dir_distortion, 
	std::string dir_distortion_pn, 
	int sub_flag,
	int flag3_cal);
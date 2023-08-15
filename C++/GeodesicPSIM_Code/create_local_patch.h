#pragma once
#include "headfile.h"

extern double Euclidean_distance(Eigen::VectorXd & p1, Eigen::VectorXd & p2);
extern int knnSearch(Eigen::MatrixXd & points, Eigen::VectorXd & keypoint);
Patch_Mesh create_local_patch(Clean_Mesh& ref_mesh_clean, Eigen::VectorXd& keypoint);

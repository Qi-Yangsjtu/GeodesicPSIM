#pragma once
#include "headfile.h"

typedef Eigen::SparseMatrix< double > SparseMatrix;
typedef Eigen::Triplet< double > Triplet;

Eigen::MatrixXd cotmatrix(Eigen::MatrixXd V, Eigen::MatrixXi F);
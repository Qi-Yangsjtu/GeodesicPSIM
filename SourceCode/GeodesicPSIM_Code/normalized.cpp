#include "headfile.h"

Eigen::MatrixXi normalized(Eigen::MatrixXd matrix, int max) {
	int row = matrix.rows();
	int col = matrix.cols();
	Eigen::MatrixXi normalized_matrix(row, col);

	for (int i = 0; i < row; ++i) {
		for (int j = 0; j < col; ++j) {
			int value = (int)(matrix(i, j) + 0.5);
			value = value == 0 ? 1 : value;
			value = value > max ? max : value;
			normalized_matrix(i, j) = value;
		}
	}
	return normalized_matrix;
}
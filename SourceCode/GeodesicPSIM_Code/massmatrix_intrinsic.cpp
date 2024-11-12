#include "headfile.h"


Eigen::MatrixXd massmatrix_intrinsic(Eigen::MatrixXd edge, Eigen::MatrixXi face, int nvert) {


	Eigen::MatrixXi tanspose_face = face.transpose();

	Eigen::MatrixXd l1 = edge.col(0);
	Eigen::MatrixXd l2 = edge.col(1);
	Eigen::MatrixXd l3 = edge.col(2);

	Eigen::MatrixXd s = (l1 + l2 + l3) * 0.5;

	Eigen::MatrixXd dblA(s.rows(), 1);
	for (int i = 0; i < s.rows(); ++i) {
		dblA(i, 0) = 2 * sqrt(s(i,0) * (s(i, 0) - l1(i, 0)) * (s(i, 0) - l2(i, 0)) * (s(i, 0) - l3(i, 0)));
	}

	Eigen::MatrixXi i1 = tanspose_face.row(0);
	Eigen::MatrixXi i2 = tanspose_face.row(1);
	Eigen::MatrixXi i3 = tanspose_face.row(2);

	Eigen::MatrixXd cosines_0(edge.rows(), 1);
	Eigen::MatrixXd cosines_1(edge.rows(), 1);
	Eigen::MatrixXd cosines_2(edge.rows(), 1);

	for (int i = 0; i < edge.rows(); ++i) {
		cosines_0(i, 0) = (edge(i, 2) * edge(i, 2) + edge(i, 1) * edge(i, 1) - edge(i, 0) * edge(i, 0)) / (2 * edge(i, 1) * edge(i, 2));
		cosines_1(i, 0) = (edge(i, 0) * edge(i, 0) + edge(i, 2) * edge(i, 2) - edge(i, 1) * edge(i, 1)) / (2 * edge(i, 0) * edge(i, 2));
		cosines_2(i, 0) = (edge(i, 0) * edge(i, 0) + edge(i, 1) * edge(i, 1) - edge(i, 2) * edge(i, 2)) / (2 * edge(i, 0) * edge(i, 1));
	}


	Eigen::MatrixXd cosines(cosines_0.rows(), cosines_0.cols() *3);
	cosines << cosines_0, cosines_1, cosines_2;

	Eigen::MatrixXd barycentric = cosines.array() * edge.array();

	Eigen::MatrixXd row_sums = barycentric.rowwise().sum();
	Eigen::MatrixXd normalized_barycentric(barycentric.rows(), barycentric.cols());

	for (int i = 0; i < barycentric.rows(); ++i) {
		normalized_barycentric(i, 0) = barycentric(i, 0) / row_sums(i, 0);
		normalized_barycentric(i, 1) = barycentric(i, 1) / row_sums(i, 0);
		normalized_barycentric(i, 2) = barycentric(i, 2) / row_sums(i, 0);
	}

    Eigen::MatrixXd areas =	 (edge.col(0) + edge.col(1) - edge.col(2)).array() *
						     (edge.col(0) - edge.col(1) + edge.col(2)).array() *
                             (-edge.col(0) + edge.col(1) + edge.col(2)).array() *
                             (edge.col(0) + edge.col(1) + edge.col(2)).array();

	areas = 0.25 * areas.array().sqrt();


	Eigen::MatrixXd partial_triangle_areas(normalized_barycentric.rows(), normalized_barycentric.cols());
	for (int i = 0; i < normalized_barycentric.rows(); ++i) {
		partial_triangle_areas(i, 0) = areas(i, 0) * normalized_barycentric(i, 0);
		partial_triangle_areas(i, 1) = areas(i, 0) * normalized_barycentric(i, 1);
		partial_triangle_areas(i, 2) = areas(i, 0) * normalized_barycentric(i, 2);
	}

  
	Eigen::MatrixXd quads(partial_triangle_areas.rows(), partial_triangle_areas.cols());

	for (int i = 0; i < partial_triangle_areas.rows(); ++i) {
		quads(i, 0) = (partial_triangle_areas(i, 1) + partial_triangle_areas(i, 2)) * 0.5;
		quads(i, 1) = (partial_triangle_areas(i, 0) + partial_triangle_areas(i, 2)) * 0.5;
		quads(i, 2) = (partial_triangle_areas(i, 0) + partial_triangle_areas(i, 1)) * 0.5;
	}

    for (int k = 0; k < quads.rows(); k++) {
		if (cosines(k, 0) < 0) {
			quads(k, 0) = areas(k, 0) * 0.5;
			quads(k, 1) = areas(k, 0) * 0.25;
			quads(k, 2) = areas(k, 0) * 0.25;
		}
		if (cosines(k, 1) < 0) {
			quads(k, 0) = areas(k, 0) * 0.25;
			quads(k, 1) = areas(k, 0) * 0.5;
			quads(k, 2) = areas(k, 0) * 0.25;
		}
		if (cosines(k, 2) < 0) {
			quads(k, 0) = areas(k, 0) * 0.25;
			quads(k, 1) = areas(k, 0) * 0.25;
			quads(k, 2) = areas(k, 0) * 0.5;
		}
    }

	Eigen::MatrixXi ii(i1.cols(), 3 * i1.rows());
	Eigen::MatrixXi jj(i1.cols(), 3 * i1.rows());

	i1.transposeInPlace();
	i2.transposeInPlace();
	i3.transposeInPlace();

	ii.col(0) = i1; ii.col(1) = i2; ii.col(2) = i3;
	jj.col(0) = i1; jj.col(1) = i2; jj.col(2) = i3;

	ii.resize(1, ii.rows() * ii.cols());
	jj.resize(1, jj.rows() * jj.cols());

	SparseMatrix M(nvert, nvert);

	quads.resize(1, quads.rows() * quads.cols());

	Eigen::MatrixXd full_M(nvert, nvert);
	full_M.setZero();

	for (int i = 0; i < ii.rows() * ii.cols(); ++i) {
		full_M(ii(0, i), jj(0, i)) += quads(0, i);
	}

	return full_M;
}
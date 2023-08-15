#include "headfile.h"

Eigen::MatrixXd cotmatrix(Eigen::MatrixXd V, Eigen::MatrixXi F)
{
	int ss = F.cols();
	int num_vertices = V.rows();
	int num_faces = F.rows();

	SparseMatrix L(num_vertices, num_vertices);

	Eigen::MatrixXd full_v(num_vertices, num_vertices);
	full_v.setZero();


	if (ss == 3) {
		Eigen::MatrixXi FT = F.transpose();

		Eigen::MatrixXi i1 = FT.row(0);
		Eigen::MatrixXi i2 = FT.row(1);
		Eigen::MatrixXi i3 = FT.row(2);

		Eigen::MatrixXd v1(F.rows(), F.cols());
		Eigen::MatrixXd v2(F.rows(), F.cols());
		Eigen::MatrixXd v3(F.rows(), F.cols());

		for(int i = 0; i < F.rows(); i++) {
			v1.row(i) << V.row(i3(0, i)).array() - V.row(i2(0, i)).array();
			v2.row(i) << V.row(i1(0, i)).array() - V.row(i3(0, i)).array();
			v3.row(i) << V.row(i2(0, i)).array() - V.row(i1(0, i)).array();
		}

		Eigen::MatrixXd dblA(v1.rows(), 1);
		if (V.cols() == 2)
		{
			
			dblA = (v1.col(0).cwiseProduct(v2.col(1)) - v1.col(1).cwiseProduct(v2.col(0))).cwiseAbs();
		}
		else if (V.cols() == 3)
		{
			for (int i = 0; i < v1.rows(); ++i) {
				Eigen::Vector3d vec_a(v1(i, 0), v1(i, 1), v1(i, 2));
				Eigen::Vector3d vec_b(v2(i, 0), v2(i, 1), v2(i, 2));

				Eigen::Vector3d cross_product = vec_a.cross(vec_b);
				dblA(i, 0) = cross_product.norm();
			}
		}

		Eigen::MatrixXd cot12(dblA.rows(), 1);

		for (int i = 0; i < dblA.rows(); ++i) {
			double sums = 0;
			for (int j = 0; j < v1.cols(); ++j) {
				sums += v1(i, j) * v2(i, j);
			}
			cot12(i, 0) = -sums / ((dblA(i, 0) * 2));
		}

		Eigen::MatrixXd cot23(dblA.rows(), 1);
		for (int i = 0; i < dblA.rows(); ++i) {
			double sums = 0;
			for (int j = 0; j < v2.cols(); ++j) {
				sums += v2(i, j) * v3(i, j);
			}
			cot23(i, 0) = -sums / ((dblA(i, 0) * 2));
		}

		Eigen::MatrixXd cot31(dblA.rows(), 1);

		for (int i = 0; i < dblA.rows(); ++i) {
			double sums = 0;
			for (int j = 0; j < v3.cols(); ++j) {
				sums += v3(i, j) * v1(i, j);
			}
			cot31(i, 0) = -sums / ((dblA(i, 0) * 2));
		}

		Eigen::MatrixXd diag1 = -cot12.array() - cot31.array();
		Eigen::MatrixXd diag2 = -cot12.array() - cot23.array();
		Eigen::MatrixXd diag3 = -cot31.array() - cot23.array();

		Eigen::MatrixXi ii(i1.cols(), 9);
		Eigen::MatrixXi	jj(i1.cols(), 9);
		Eigen::MatrixXd v(cot12.rows(), 9);
		
		i1.transposeInPlace();
		i2.transposeInPlace();
		i3.transposeInPlace();

		ii.col(0) = i1.col(0); ii.col(1) = i2.col(0); ii.col(2) = i2.col(0);
		ii.col(3) = i3.col(0); ii.col(4) = i3.col(0); ii.col(5) = i1.col(0);
		ii.col(6) = i1.col(0); ii.col(7) = i2.col(0); ii.col(8) = i3.col(0);

		jj.col(0) = i2.col(0); jj.col(1) = i1.col(0); jj.col(2) = i3.col(0);
		jj.col(3) = i2.col(0); jj.col(4) = i1.col(0); jj.col(5) = i3.col(0);
		jj.col(6) = i1.col(0); jj.col(7) = i2.col(0); jj.col(8) = i3.col(0);

		v.col(0) = cot12; v.col(1) = cot12; v.col(2) = cot23;
		v.col(3) = cot23; v.col(4) = cot31; v.col(5) = cot31;
		v.col(6) = diag1; v.col(7) = diag2; v.col(8) = diag3;

		ii.resize(1, ii.rows() * ii.cols());
		jj.resize(1, ii.rows() * ii.cols());
		v.resize(1, 9 * cot12.rows());

		for (int i = 0; i < ii.rows() * ii.cols(); ++i) {
			full_v(ii(0, i), jj(0, i)) += v(0, i);
		}
	}

	return full_v;
}

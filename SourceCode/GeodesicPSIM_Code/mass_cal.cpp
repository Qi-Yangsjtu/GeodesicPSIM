#include "headfile.h"

Eigen::MatrixXd mass_cal(Patch_Mesh ref_patch_mesh) {

	Eigen::MatrixXi face = ref_patch_mesh.face;

	int nvert = ref_patch_mesh.vertexInFace_count;

	Eigen::MatrixXd pointInface = ref_patch_mesh.vertexInFace;

	Eigen::MatrixXd face_point1(face.rows(), face.cols());
	Eigen::MatrixXd face_point2(face.rows(), face.cols());
	Eigen::MatrixXd face_point3(face.rows(), face.cols());

	for (int i = 0; i < face.rows(); ++i) {
		face_point1.row(i) = pointInface.row(face(i, 0));
		face_point2.row(i) = pointInface.row(face(i, 1));
		face_point3.row(i) = pointInface.row(face(i, 2));
	}

	Eigen::MatrixXd edge1 = (face_point2 - face_point3).rowwise().norm();
	Eigen::MatrixXd edge2 = (face_point1 - face_point3).rowwise().norm();
	Eigen::MatrixXd edge3 = (face_point1 - face_point2).rowwise().norm();

	Eigen::MatrixXd edge(edge1.rows(), 3);
	edge << edge1, edge2, edge3;

	Eigen::MatrixXd massmatric = massmatrix_intrinsic(edge, face, nvert);

	return massmatric;
}
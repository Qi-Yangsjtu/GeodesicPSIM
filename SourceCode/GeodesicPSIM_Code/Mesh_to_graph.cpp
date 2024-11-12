#include "headfile.h"

bool compare(const std::vector<int>& a, const std::vector<int>& b) {
	if (a[0] != b[0]) {
		return a[0] < b[0];  
	}
	else {
		return a[1] < b[1]; 
	}
}

Mesh2Graph Mesh_to_graph(Patch_Mesh patch_mesh) {
	double eps = 1e-6;
	Mesh2Graph ret_mesh2graph;

	ret_mesh2graph.dis_w.conservativeResize(patch_mesh.vertexInFace_count, patch_mesh.vertexInFace_count);
	ret_mesh2graph.W.conservativeResize(patch_mesh.vertexInFace_count, patch_mesh.vertexInFace_count);
	ret_mesh2graph.W_nor.conservativeResize(patch_mesh.vertexInFace_count, patch_mesh.vertexInFace_count);

	ret_mesh2graph.dis_w.setZero();
	ret_mesh2graph.W.setZero();
	ret_mesh2graph.W_nor.setZero();
	
	Eigen::MatrixXi face_matrix = patch_mesh.face;

	Eigen::MatrixXi edge1(face_matrix.rows(), 2);
	edge1.col(0) = face_matrix.col(0); edge1.col(1) = face_matrix.col(1);

	Eigen::MatrixXi edge2(face_matrix.rows(), 2);
	edge2.col(0) = face_matrix.col(0); edge2.col(1) = face_matrix.col(2);

	Eigen::MatrixXi edge3(face_matrix.rows(), 2);
	edge3.col(0) = face_matrix.col(1); edge3.col(1) = face_matrix.col(2);

	Eigen::MatrixXi edge_un_unique(face_matrix.rows() * 3, 2);
	edge_un_unique << edge1, edge2, edge3;

	std::vector< std::vector<int> > unique_index;
	for (int i = 0; i < face_matrix.rows() * 3; ++i) {
		std::vector<int> this_edge = { (int)(edge_un_unique(i,0)), (int)(edge_un_unique(i,1)) };
		bool in_flag = false;
		for (int j = 0; j < unique_index.size(); ++j) {
			if (this_edge == unique_index[j]) {
				in_flag = true;
			}
		}
		if (!in_flag) {
			unique_index.push_back(this_edge);
		}
	}

	std::sort(unique_index.begin(), unique_index.end(), compare);

	Eigen::MatrixXi edge(unique_index.size(), 2);
	for (int i = 0; i < edge.rows(); ++i) {
		edge(i, 0) = unique_index[i][0]; edge(i, 1) = unique_index[i][1];
	}

	Eigen::MatrixXd coord_point_1(edge.rows(), patch_mesh.vertexInFace.cols());
	Eigen::MatrixXd coord_point_2(edge.rows(), patch_mesh.vertexInFace.cols());

	for (int i = 0; i < edge.rows(); ++i) {
		coord_point_1.row(i) = patch_mesh.vertexInFace.row(edge(i, 0));
		coord_point_2.row(i) = patch_mesh.vertexInFace.row(edge(i, 1));
	}

	Eigen::MatrixXd Euclidean_distance = ((coord_point_1.col(0).array() - coord_point_2.col(0).array()).pow(2)
										+ (coord_point_1.col(1).array() - coord_point_2.col(1).array()).pow(2)
										+ (coord_point_1.col(2).array() - coord_point_2.col(2).array()).pow(2)).sqrt();

	double radius = Euclidean_distance.mean();

	Eigen::MatrixXd distance = (-Euclidean_distance / (radius * 2.0 + eps)).array().exp();

	for (int i = 0; i < distance.rows(); ++i) {
		ret_mesh2graph.W((int)(edge(i, 0)), (int)(edge(i, 1))) = distance(i, 0);
		ret_mesh2graph.W((int)(edge(i, 1)), (int)(edge(i, 0))) = distance(i, 0);
		ret_mesh2graph.dis_w((int)(edge(i, 0)), (int)(edge(i, 1))) = Euclidean_distance(i, 0);
		ret_mesh2graph.dis_w((int)(edge(i, 1)), (int)(edge(i, 0))) = Euclidean_distance(i, 0);
	}
	ret_mesh2graph.W_flage = ret_mesh2graph.W.array().unaryExpr([](const auto& x) {
		return (x > 0) ? 1 : 0;
	});

	ret_mesh2graph.D.conservativeResize(ret_mesh2graph.W.rows(), ret_mesh2graph.W.cols());
	ret_mesh2graph.D.setZero();
	for (int i = 0; i < ret_mesh2graph.D.rows(); ++i) {
		ret_mesh2graph.D(i, i) = ret_mesh2graph.W.row(i).sum();
		ret_mesh2graph.W_nor.row(i) = ret_mesh2graph.W.row(i) / ret_mesh2graph.D(i, i);
	}

	ret_mesh2graph.D_flage = ret_mesh2graph.W_flage.rowwise().sum();

	return ret_mesh2graph;
}
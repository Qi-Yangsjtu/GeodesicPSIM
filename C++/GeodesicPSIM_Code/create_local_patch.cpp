#include "headfile.h"

double Euclidean_distance(Eigen::VectorXd & p1, Eigen::VectorXd & p2) {
	Eigen::VectorXd dis_vec = p1 - p2;
	return dis_vec.norm();
}

int knnSearch(Eigen::MatrixXd & points, Eigen::VectorXd & keypoint) {
	std::vector<std::pair<double, int>> distances;
	Eigen::VectorXd point;

	for (int i = 0; i < points.rows(); i++) {
		point = points.row(i);
		double distance = Euclidean_distance(point, keypoint);
		distances.push_back(std::make_pair(distance, i));
	}
	std::sort(distances.begin(), distances.end());
	int nearest_index = distances[0].second;
	return nearest_index;
}

Patch_Mesh create_local_patch(Clean_Mesh &ref_mesh_clean, Eigen::VectorXd &keypoint)
{
	Patch_Mesh ref_patch_mesh;

	int coarse_index_ref;

	coarse_index_ref = knnSearch(ref_mesh_clean.vertexInFace, keypoint);

	
	Eigen::MatrixXd patch_center = ref_mesh_clean.vertexInFace.row(coarse_index_ref);
	Eigen::MatrixXi faces = ref_mesh_clean.face;

	Eigen::MatrixXi face_selected;
	int cnt = 0;
	for (int i = 0; i < faces.rows(); ++i) {
		if ( (faces(i, 0) == coarse_index_ref) || (faces(i, 1) == coarse_index_ref) || (faces(i, 2) == coarse_index_ref) ) {
			cnt += 1;
			face_selected.conservativeResize(cnt, faces.cols());
			face_selected.row(cnt - 1) = faces.row(i);
		}
	}

	std::unordered_set<int> vertex_set;
	int face_selected_row = face_selected.rows();
	int face_selected_col = face_selected.cols();
	for (int i = 0; i < face_selected_row; ++i) {
		for (int j = 0; j < face_selected_col; ++j) {
			vertex_set.insert(face_selected(i, j));
		}
	}

	std::vector<int> vertex_index(vertex_set.begin(), vertex_set.end());
	std::sort(vertex_index.begin(), vertex_index.end());

	int vertex_index_size = vertex_index.size();

	ref_patch_mesh.vertexInFace = ref_mesh_clean.vertexInFace.transpose()(Eigen::placeholders::all, vertex_index).transpose();


	ref_patch_mesh.vertexInFace_count = ref_patch_mesh.vertexInFace.rows();


	ref_patch_mesh.vertexcolor = ref_mesh_clean.vertexcolor.transpose()(Eigen::placeholders::all, vertex_index).transpose();


	ref_patch_mesh.texture_coor = ref_mesh_clean.texture_coor.transpose()(Eigen::placeholders::all, vertex_index).transpose();

	ref_patch_mesh.patch_center_index.conservativeResize(ref_patch_mesh.vertexInFace_count, 1);
	for (int i = 0; i < ref_patch_mesh.vertexInFace_count; ++i) {
		if (patch_center == ref_patch_mesh.vertexInFace.row(i))
			ref_patch_mesh.patch_center_index(i, 0) = 1;
		else
			ref_patch_mesh.patch_center_index(i, 0) = 0;
	}


	std::vector<int> Rowindex;
	for (int i = 0; i < ref_patch_mesh.vertexInFace_count; ++i) {
		for (int j = 0; j < ref_mesh_clean.vertexInFace.rows(); ++j) {
			if (ref_patch_mesh.vertexInFace.row(i) == ref_mesh_clean.vertexInFace.row(j)){
				Rowindex.push_back(j);
				break;
			}
		}
	}
	face_selected.resize(1, face_selected.size());

	Eigen::MatrixXi rearray_face_patch = face_selected;

	
	std::unordered_map<Eigen::MatrixXi, int, MatrixXiHash> new_face_index_hash_map;
	std::vector<int> new_face_index;
	
	
	for (int i = 0; i < rearray_face_patch.cols(); ++i) {
		for (int j = 0; j < Rowindex.size(); ++j) {
			if (rearray_face_patch(0, i) == Rowindex[j]) {
				new_face_index.push_back(j);
			}
		}
	}

	Eigen::MatrixXi new_face_index_matrix(1, new_face_index.size());
	for (int i = 0; i < new_face_index.size(); ++i) {
		new_face_index_matrix(0, i) = new_face_index[i];
	}

	new_face_index_matrix.resize(face_selected_row, face_selected_col);

	Eigen::MatrixXi face_patch_re = new_face_index_matrix;

	ref_patch_mesh.face = face_patch_re;
	ref_patch_mesh.face_count = face_patch_re.rows();

	return ref_patch_mesh;
}
#include "headfile.h"

void uv2yuv(Eigen::MatrixXd& vertexcolor, Image3& dis_texture, Eigen::MatrixXd uv_group) {

	Eigen::MatrixXd dis_u2x = (1 - uv_group.col(1).array()) * dis_texture.row;
	Eigen::MatrixXd dis_v2y = uv_group.col(0).array() * dis_texture.col;

	Eigen::MatrixXi dis_x_coor = normalized(dis_u2x, dis_texture.row);
	Eigen::MatrixXi dis_y_coor = normalized(dis_v2y, dis_texture.col);

	Eigen::MatrixXi dis_color_index = (dis_x_coor.array() - 1) * dis_texture.col + dis_y_coor.array();

	Eigen::MatrixXi dis_vertex_r(1, dis_color_index.rows());
	Eigen::MatrixXi dis_vertex_g(1, dis_color_index.rows());
	Eigen::MatrixXi dis_vertex_b(1, dis_color_index.rows());

	for (int i = 0; i < dis_color_index.rows(); ++i) {

		int row = (dis_color_index(i, 0) / dis_texture.col);
		row = row < 0 ? 0 : row;
		row = row > dis_texture.row - 1 ? dis_texture.row - 1 : row;
		int col = dis_color_index(i, 0) - (row * dis_texture.col) - 1;
		col = col < 0 ? 0 : col;
		col = col > dis_texture.col - 1 ? dis_texture.col - 1 : col;

		dis_vertex_r(0, i) = dis_texture.r(row, col);
		dis_vertex_g(0, i) = dis_texture.g(row, col);
		dis_vertex_b(0, i) = dis_texture.b(row, col);
	}

	Eigen::MatrixXi dis_vertexcolor(dis_vertex_r.cols(), 3);
	dis_vertexcolor.col(0) = dis_vertex_r.transpose();
	dis_vertexcolor.col(1) = dis_vertex_g.transpose();
	dis_vertexcolor.col(2) = dis_vertex_b.transpose();

	Eigen::MatrixXd YUV_matrix(3, 3);

	YUV_matrix << 0.299, 0.587, 0.114, -0.1678, -0.3313, 0.5, 0.5, -0.4187, -0.0813;
	YUV_matrix.transposeInPlace();

	Eigen::MatrixXd yuv = dis_vertexcolor.cast<double>() * YUV_matrix;

	yuv.col(1) = yuv.col(1).array() + 128;
	yuv.col(2) = yuv.col(2).array() + 128;

	vertexcolor = yuv;

}


void rescalePatch(Patch_Mesh& dis_patch_mesh, double scale, Image3& dis_texture) {

	// update the vertex coordinates

	Eigen::MatrixXd old_dis_vertex = dis_patch_mesh.vertexInFace;

	Eigen::MatrixXd vec_from_neighbor_to_center(old_dis_vertex.rows(), old_dis_vertex.cols());
	int idx = 0;
	for (int i = 0; i < dis_patch_mesh.patch_center_index.rows(); ++i) {
		if (dis_patch_mesh.patch_center_index(i, 0)) {
			idx = i;
		}
	}

	vec_from_neighbor_to_center = old_dis_vertex.rowwise() - old_dis_vertex.row(idx);

	Eigen::MatrixXd new_dis_vertex_shift = vec_from_neighbor_to_center.array() / scale;

	Eigen::MatrixXd new_dis_vertex = new_dis_vertex_shift.rowwise() + old_dis_vertex.row(idx);

	// update the uv coordinates

	// add new update for each face

	Eigen::MatrixXd patch_center_index = dis_patch_mesh.patch_center_index;

	int a = 0;
	for (int i = 0; i < patch_center_index.rows(); ++i) {
		if (patch_center_index(i, 0) == 1) a = i;
	}

	// search UV of patch center "a"

	Eigen::MatrixXi face = dis_patch_mesh.face;
	Eigen::MatrixXd uv = dis_patch_mesh.texture_coor;
	Eigen::MatrixXi face_texture = dis_patch_mesh.face_texture;

	std::vector<Eigen::MatrixXd> per_face_uv_tri;

	for (int i = 0; i < face.rows(); ++i) {

		int center_index;
		std::vector<int> neib_index;

		for (int j = 0; j < 3; ++j) {
			if (face(i, j) == a) center_index = j;
			else neib_index.push_back(j);
		}

		int center_uv_index = face_texture(i, center_index);
		Eigen::MatrixXd center_uv = uv.row(center_uv_index);

		int neib1_uv_index = face_texture(i, neib_index[0]);
		int neib2_uv_index = face_texture(i, neib_index[1]);
		Eigen::MatrixXd neib1_uv = uv.row(neib1_uv_index);
		Eigen::MatrixXd neib2_uv = uv.row(neib2_uv_index);

		Eigen::MatrixXd uv_from_neib1_to_center = neib1_uv - center_uv;
		Eigen::MatrixXd uv_from_neib2_to_center = neib2_uv - center_uv;

		Eigen::MatrixXd uv_neib1_shift = uv_from_neib1_to_center / scale;
		Eigen::MatrixXd uv_neib2_shift = uv_from_neib2_to_center / scale;
		Eigen::MatrixXd uv_neib1_new = center_uv + uv_neib1_shift;
		Eigen::MatrixXd uv_neib2_new = center_uv + uv_neib2_shift;

		// sort uv based on face vertex information

		Eigen::MatrixXd per_face_uv(3, 2);
		per_face_uv.row(center_index) = center_uv;
		per_face_uv.row(neib_index[0]) = uv_neib1_new;
		per_face_uv.row(neib_index[1]) = uv_neib2_new;

		per_face_uv_tri.push_back(per_face_uv);
	}

	dis_patch_mesh.vertexInFace = new_dis_vertex;
	dis_patch_mesh.per_face_uv_tri = per_face_uv_tri;

	// update the vertex color based on the new uv

	// each vertex might have multiple UVs

	Eigen::MatrixXd vertexcolor(old_dis_vertex.rows(), 3);

	for (int i = 0; i < vertexcolor.rows(); ++i) {

		// seach all the uv of this vertex

		std::vector<std::pair<int, int>> coor;
		for (int j = 0; j < face.rows(); ++j) {

			for (int k = 0; k < 3; ++k) {
				if (face(j, k) == i) {
					std::pair<int, int> this_coor = std::make_pair(j, k);
					coor.push_back(this_coor);
				}
			}
		}

		Eigen::MatrixXd uv_group(coor.size(), 2);

		for (int k = 0; k < coor.size(); ++k) {
			Eigen::MatrixXd face_uv_check = per_face_uv_tri[coor[k].first];
			uv_group.row(k) = face_uv_check.row(coor[k].second);
		}

		Eigen::MatrixXd vertex_color_group(uv_group.rows(), 3);
		uv2yuv(vertex_color_group, dis_texture, uv_group);
		vertexcolor.row(i) = vertex_color_group.colwise().mean();

	}

	dis_patch_mesh.vertexcolor = vertexcolor;

}
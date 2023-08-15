#include "headfile.h"

void rescalePatch(Patch_Mesh &dis_patch_mesh, double scale, Image3& dis_texture) {

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

	Eigen::MatrixXi old_dis_uv = dis_patch_mesh.texture_coor;

	Eigen::MatrixXi uv_from_neighbor_to_center = old_dis_uv.rowwise() - old_dis_uv.row(idx);

	Eigen::MatrixXi new_dis_uv_shift = uv_from_neighbor_to_center;
	for (int i = 0; i < new_dis_uv_shift.rows(); ++i) {
		for (int j = 0; j < new_dis_uv_shift.cols(); ++j) {
			new_dis_uv_shift(i, j) = round(1.0 * new_dis_uv_shift(i, j) / scale);
		}
	}

	Eigen::MatrixXi new_dis_uv = new_dis_uv_shift.rowwise() + old_dis_uv.row(idx);

	dis_patch_mesh.vertexInFace = new_dis_vertex;
	dis_patch_mesh.texture_coor = new_dis_uv;

	Eigen::MatrixXi dis_x_coor = new_dis_uv.col(0);
	Eigen::MatrixXi dis_y_coor = new_dis_uv.col(1);

	Eigen::MatrixXi dis_color_index = (dis_x_coor.array() - 1) * dis_texture.col + dis_y_coor.array();

	std::vector<int> dis_color_index_vec;
	for (int i = 0; i < dis_color_index.rows(); ++i) {
		dis_color_index_vec.push_back(dis_color_index(i, 0));
	}

	Eigen::MatrixXi dis_vertex_r(1, dis_color_index.rows());
	Eigen::MatrixXi dis_vertex_g(1, dis_color_index.rows());
	Eigen::MatrixXi dis_vertex_b(1, dis_color_index.rows());

	for (int i = 0; i < dis_color_index.rows(); ++i) {
	
		int row = (dis_color_index(i, 0) / dis_texture.col);
		row = row < 0 ? 0 : row;
		row = row > dis_texture.row - 1 ? dis_texture.row - 1 : row;
		int col = dis_color_index(i, 0) - (row *  dis_texture.col) - 1;
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

	dis_patch_mesh.vertexcolor = yuv;
}
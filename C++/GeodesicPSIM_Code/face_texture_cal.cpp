#include "headfile.h"


void face_texture_cal(Patch_Mesh &ref_patch_mesh, Image3& texturemap)
{
	Eigen::MatrixXi face = ref_patch_mesh.face;

	Eigen::MatrixXi face_re = face;
	face_re.resize(1, face.rows() * face.cols());

	Eigen::MatrixXi face_x_coor(face_re.cols(), 1);
	Eigen::MatrixXi face_y_coor(face_re.cols(), 1);

	for (int i = 0; i < face_re.cols(); ++i) {
		face_x_coor(i, 0) = ref_patch_mesh.texture_coor(face_re(0, i), 0);
		face_y_coor(i, 0) = ref_patch_mesh.texture_coor(face_re(0, i), 1);
	}
	
	Eigen::MatrixXi face_x_coor_re = face_x_coor;
	Eigen::MatrixXi face_y_coor_re = face_y_coor;

	face_x_coor_re.resize(ref_patch_mesh.face.rows(), ref_patch_mesh.face.cols());
	face_y_coor_re.resize(ref_patch_mesh.face.rows(), ref_patch_mesh.face.cols());

	Eigen::MatrixXd YUV_matrix(3, 3);
	YUV_matrix << 0.299, 0.587, 0.114, -0.1678, -0.3313, 0.5, 0.5, -0.4187, -0.0813;
	YUV_matrix.transposeInPlace();

	Eigen::MatrixXi per_face_x_max = face_x_coor_re.rowwise().maxCoeff();
	Eigen::MatrixXi per_face_y_max = face_y_coor_re.rowwise().maxCoeff();

	Eigen::MatrixXi per_face_x_min = face_x_coor_re.rowwise().minCoeff();
	Eigen::MatrixXi per_face_y_min = face_y_coor_re.rowwise().minCoeff();
	
	Eigen::MatrixXd l_mean(face.rows(), 1);
	Eigen::MatrixXd l_var(face.rows(), 1);

	Eigen::MatrixXd c1_mean(face.rows(), 1);
	Eigen::MatrixXd c1_var(face.rows(), 1);

	Eigen::MatrixXd c2_mean(face.rows(), 1);
	Eigen::MatrixXd c2_var(face.rows(), 1);
	
	int allpixel = texturemap.row * texturemap.col;

	Eigen::MatrixXi pointInface_num(face.rows(), 1);
	Eigen::MatrixXd face_center(face.rows(), 3);

	std::vector<Eigen::MatrixXi> pointInface_cell;
	
	for (int i = 0; i < face.rows(); ++i) {

		Eigen::MatrixXd vertices1 = ref_patch_mesh.vertexInFace.row(face(i, 0));
		Eigen::MatrixXd vertices2 = ref_patch_mesh.vertexInFace.row(face(i, 1));
		Eigen::MatrixXd vertices3 = ref_patch_mesh.vertexInFace.row(face(i, 2));

		Eigen::MatrixXd center = (vertices1 + vertices2 + vertices3) / 3;

		face_center.row(i) = center;
		
		Eigen::MatrixXi coarse_area_x(per_face_x_max(i, 0) - per_face_x_min(i, 0) + 1, 1);
		for (int j = 0; j < per_face_x_max(i, 0) - per_face_x_min(i, 0) + 1; ++j) {
			coarse_area_x(j, 0) = per_face_x_min(i, 0) + j;
		}
		coarse_area_x.transposeInPlace();

		Eigen::MatrixXi coarse_area_y(per_face_y_max(i, 0) - per_face_y_min(i, 0) + 1, 1);
		for (int j = 0; j < per_face_y_max(i, 0) - per_face_y_min(i, 0) + 1; ++j) {
			coarse_area_y(j, 0) = per_face_y_min(i, 0) + j;
		}
		coarse_area_y.transposeInPlace();
	
		Eigen::MatrixXi m(coarse_area_y.cols(), coarse_area_x.cols());
		Eigen::MatrixXi n(coarse_area_y.cols(), coarse_area_x.cols());

		for (int j = 0; j < coarse_area_y.cols(); ++j) {
			m.row(j) = coarse_area_x.row(0);
		}
		for (int j = 0; j < coarse_area_x.cols(); ++j) {
			n.col(j) = coarse_area_y.row(0);
		}

		Eigen::MatrixXi a(coarse_area_y.cols() * coarse_area_x.cols(), 1);
		Eigen::MatrixXi b(coarse_area_y.cols() * coarse_area_x.cols(), 1);

		for (int j = 0; j < m.cols(); ++j) {
			for (int k = 0; k < m.rows(); ++k) {
				a(j * m.rows() + k, 0) = m(k, j);
				b(j * m.rows() + k, 0) = n(k, j);
			}
		}
		
		Eigen::MatrixXi x_vertex = face_x_coor_re.row(i);
		Eigen::MatrixXi y_vertex = face_y_coor_re.row(i);

		Eigen::MatrixXi index(a.rows(), 1);

		std::vector<Point> range;
	
		for (int j = 0; j < x_vertex.cols(); ++j) {
			Point range_point = { x_vertex(0, j), y_vertex(0,j) };
			range.push_back(range_point);
		}

		for (int j = 0; j < a.rows(); ++j) {
			Point verify_point = {a(j, 0), b(j, 0)};
			if (isPointInsidePolygon(verify_point, range)) {
				index(j, 0) = 1;
			}
			else {
				index(j, 0) = 0;
			}

		}

		Eigen::MatrixXi res(a.rows(), 2);
		res.col(0) = a; res.col(1) = b;

		int cnt = 0;
		for (int j = 0; j < index.rows(); ++j) {
			if (index(j, 0) == 1) {
				cnt += 1;
			}
		}

		Eigen::MatrixXi pointInface(cnt, 2);
		cnt = 0;
		for (int j = 0; j < index.rows(); ++j) {
			if (index(j, 0)) {
				pointInface.row(cnt) = res.row(j);
				cnt += 1;
			}
		}

		pointInface_cell.push_back(pointInface);

		pointInface_num(i, 0) = pointInface.rows();

		Eigen::MatrixXi color_index = (pointInface.col(0).array() - 1) * texturemap.col + pointInface.col(1).array();

		for (int j = 0; j < color_index.rows(); ++j) {
			int value = color_index(j, 0);
			value = value > allpixel ? allpixel : value;
			value = value < 1 ? 1 : value;
			color_index(j, 0) = value;
		}
	
		Eigen::MatrixXi vertex_r(1, color_index.rows());
		Eigen::MatrixXi vertex_g(1, color_index.rows());
		Eigen::MatrixXi vertex_b(1, color_index.rows());

		for (int j = 0; j < color_index.rows(); ++j) {
			int row = color_index(j, 0) / texturemap.col;
			row = row < 0 ? 0 : row;
			row = row > texturemap.row - 1 ? texturemap.row - 1 : row;
			int col = color_index(j, 0) - (row * texturemap.col) - 1;
			col = col < 0 ? 0 : col;
			col = col > texturemap.col - 1 ? texturemap.col - 1 : col;
			vertex_r(0, j) = texturemap.r(row, col);
			vertex_g(0, j) = texturemap.g(row, col);
			vertex_b(0, j) = texturemap.b(row, col);
		}

		Eigen::MatrixXi color_map(vertex_r.cols(), 3);
		color_map.col(0) = vertex_r.transpose();
		color_map.col(1) = vertex_g.transpose();
		color_map.col(2) = vertex_b.transpose();

		Eigen::MatrixXd yuv_map_tri = color_map.cast<double>() * YUV_matrix;
		yuv_map_tri.col(1) = yuv_map_tri.col(1).array() + 128;
		yuv_map_tri.col(2) = yuv_map_tri.col(2).array() + 128;

		l_mean(i, 0) = yuv_map_tri.col(0).mean();
		l_var(i, 0) = (yuv_map_tri.col(0).array() - yuv_map_tri.col(0).mean()).pow(2).sum() / yuv_map_tri.rows();

		c1_mean(i, 0) = yuv_map_tri.col(1).mean();
		c1_var(i, 0) = (yuv_map_tri.col(1).array() - yuv_map_tri.col(1).mean()).pow(2).sum() / yuv_map_tri.rows();

		c2_mean(i, 0) = yuv_map_tri.col(2).mean();
		c2_var(i, 0) = (yuv_map_tri.col(2).array() - yuv_map_tri.col(2).mean()).pow(2).sum() / yuv_map_tri.rows();

	}

	ref_patch_mesh.faceNormal = compute_mesh_normals(ref_patch_mesh);
	ref_patch_mesh.l_mean = l_mean;
	ref_patch_mesh.l_var = l_var;

	ref_patch_mesh.c1_mean = c1_mean;
	ref_patch_mesh.c1_var = c1_var;

	ref_patch_mesh.c2_mean = c2_mean;
	ref_patch_mesh.c2_var = c2_var;

	ref_patch_mesh.face_center = face_center;
	ref_patch_mesh.pointInface_cell = pointInface_cell;
	ref_patch_mesh.pointInface_num = pointInface_num;

}


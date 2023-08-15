#include "headfile.h"

void patch_cropping(Patch_Mesh &ref_patch_mesh, Patch_Mesh &dis_patch_mesh, 
						 Mesh2Graph ref_Mesh_graph_ini, Mesh2Graph dis_Mesh_graph_ini, 
						 Image3& ref_texture, Image3& dis_texture, double threshold) {

	Eigen::MatrixXd ref_vec_a(ref_patch_mesh.face.rows(), ref_patch_mesh.face.cols());
	Eigen::MatrixXd ref_vec_b(ref_patch_mesh.face.rows(), ref_patch_mesh.face.cols());

	for (int i = 0; i < ref_patch_mesh.face.rows(); ++i) {
		ref_vec_a.row(i) = ref_patch_mesh.vertexInFace.row(ref_patch_mesh.face(i, 0)) - ref_patch_mesh.vertexInFace.row(ref_patch_mesh.face(i, 1));
		ref_vec_b.row(i) = ref_patch_mesh.vertexInFace.row(ref_patch_mesh.face(i, 2)) - ref_patch_mesh.vertexInFace.row(ref_patch_mesh.face(i, 1));
	}
	Eigen::MatrixXd ref_tri_area(ref_vec_a.rows(), 1);
	for (int i = 0; i < ref_vec_a.rows(); ++i) {
		Eigen::Vector3d vec_a(ref_vec_a(i,0), ref_vec_a(i, 1), ref_vec_a(i, 2));
		Eigen::Vector3d vec_b(ref_vec_b(i, 0), ref_vec_b(i, 1), ref_vec_b(i, 2));

		Eigen::Vector3d cross_product = vec_a.cross(vec_b);
		ref_tri_area(i, 0) = cross_product.norm();
	}
	Eigen::MatrixXd dis_vec_a(dis_patch_mesh.face.rows(), dis_patch_mesh.face.cols());
	Eigen::MatrixXd dis_vec_b(dis_patch_mesh.face.rows(), dis_patch_mesh.face.cols());

	for (int i = 0; i < dis_patch_mesh.face.rows(); ++i) {
		dis_vec_a.row(i) = dis_patch_mesh.vertexInFace.row(dis_patch_mesh.face(i, 0)) - dis_patch_mesh.vertexInFace.row(dis_patch_mesh.face(i, 1));
		dis_vec_b.row(i) = dis_patch_mesh.vertexInFace.row(dis_patch_mesh.face(i, 2)) - dis_patch_mesh.vertexInFace.row(dis_patch_mesh.face(i, 1));
	}

	Eigen::MatrixXd dis_tri_area(dis_vec_a.rows(), 1);
	for (int i = 0; i < dis_vec_a.rows(); ++i) {
		Eigen::Vector3d vec_a(dis_vec_a(i, 0), dis_vec_a(i, 1), dis_vec_a(i, 2));
		Eigen::Vector3d vec_b(dis_vec_b(i, 0), dis_vec_b(i, 1), dis_vec_b(i, 2));

		Eigen::Vector3d cross_product = vec_a.cross(vec_b);
		dis_tri_area(i, 0) = cross_product.norm();
	}
	Eigen::MatrixXd ref_normal_weight = ref_tri_area.array() / ref_tri_area.sum();
	Eigen::MatrixXd dis_normal_weight = dis_tri_area.array() / dis_tri_area.sum();

	ref_patch_mesh.face_size = ref_tri_area;
	dis_patch_mesh.face_size = dis_tri_area;

	ref_patch_mesh.normal_weight = ref_normal_weight;
	dis_patch_mesh.normal_weight = dis_normal_weight;

	Eigen::MatrixXd ref_center_to_neighbor_distance(1, ref_Mesh_graph_ini.dis_w.cols());
	Eigen::MatrixXd dis_center_to_neighbor_distance(1, dis_Mesh_graph_ini.dis_w.cols());
	for (int i = 0; i < ref_patch_mesh.patch_center_index.rows(); ++i) {
		if (ref_patch_mesh.patch_center_index(i, 0)) {
			ref_center_to_neighbor_distance.row(0) = ref_Mesh_graph_ini.dis_w.row(i);
			break;
		}
	}

	for (int i = 0; i < dis_patch_mesh.patch_center_index.rows(); ++i) {
		if (dis_patch_mesh.patch_center_index(i, 0)) {
			dis_center_to_neighbor_distance.row(0) = dis_Mesh_graph_ini.dis_w.row(i);
			break;
		}
	}
	double ref_ave_distance = ref_center_to_neighbor_distance.mean();
	double dis_ave_distance = dis_center_to_neighbor_distance.mean();

	double scale = dis_ave_distance / ref_ave_distance;
	if (scale > 1) {
		rescalePatch(dis_patch_mesh, scale, dis_texture);
	}
	if (ref_ave_distance > threshold) {
		double scale1 = ref_ave_distance / threshold;
		rescalePatch(ref_patch_mesh, scale1, ref_texture);

		rescalePatch(dis_patch_mesh, scale1, dis_texture);

	}
	Eigen::MatrixXd dis_vec_a_re(dis_patch_mesh.face.rows(), dis_patch_mesh.face.cols());
	Eigen::MatrixXd dis_vec_b_re(dis_patch_mesh.face.rows(), dis_patch_mesh.face.cols());

	for (int i = 0; i < dis_patch_mesh.face.rows(); ++i) {
		dis_vec_a_re.row(i) = dis_patch_mesh.vertexInFace.row(dis_patch_mesh.face(i, 0)) - dis_patch_mesh.vertexInFace.row(dis_patch_mesh.face(i, 1));
		dis_vec_b_re.row(i) = dis_patch_mesh.vertexInFace.row(dis_patch_mesh.face(i, 2)) - dis_patch_mesh.vertexInFace.row(dis_patch_mesh.face(i, 1));
	}
	Eigen::MatrixXd dis_tri_area_re(dis_vec_a_re.rows(), 1);
	for (int i = 0; i < dis_vec_a_re.rows(); ++i) {
		Eigen::Vector3d vec_a(dis_vec_a_re(i, 0), dis_vec_a_re(i, 1), dis_vec_a_re(i, 2));
		Eigen::Vector3d vec_b(dis_vec_b_re(i, 0), dis_vec_b_re(i, 1), dis_vec_b_re(i, 2));

		Eigen::Vector3d cross_product = vec_a.cross(vec_b);
		dis_tri_area_re(i, 0) = cross_product.norm();
	}

	Eigen::MatrixXd ref_vec_a_re(ref_patch_mesh.face.rows(), ref_patch_mesh.face.cols());
	Eigen::MatrixXd ref_vec_b_re(ref_patch_mesh.face.rows(), ref_patch_mesh.face.cols());

	for (int i = 0; i < ref_patch_mesh.face.rows(); ++i) {
		ref_vec_a_re.row(i) = ref_patch_mesh.vertexInFace.row(ref_patch_mesh.face(i, 0)) - ref_patch_mesh.vertexInFace.row(ref_patch_mesh.face(i, 1));
		ref_vec_b_re.row(i) = ref_patch_mesh.vertexInFace.row(ref_patch_mesh.face(i, 2)) - ref_patch_mesh.vertexInFace.row(ref_patch_mesh.face(i, 1));
	}

	Eigen::MatrixXd ref_tri_area_re(ref_vec_a_re.rows(), 1);
	for (int i = 0; i < ref_vec_a_re.rows(); ++i) {
		Eigen::Vector3d vec_a(ref_vec_a_re(i, 0), ref_vec_a_re(i, 1), ref_vec_a_re(i, 2));
		Eigen::Vector3d vec_b(ref_vec_b_re(i, 0), ref_vec_b_re(i, 1), ref_vec_b_re(i, 2));

		Eigen::Vector3d cross_product = vec_a.cross(vec_b);
		ref_tri_area_re(i, 0) = cross_product.norm();
	}
	ref_patch_mesh.face_size_rescale = ref_tri_area_re;
	dis_patch_mesh.face_size_rescale = dis_tri_area_re;

}
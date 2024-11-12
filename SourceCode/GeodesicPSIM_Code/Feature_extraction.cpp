#include "headfile.h"

#define DEFAULT_PARALLEL_NUM	10

std::vector<SubFeature> Feature_extraction(int N, Mesh ref_mesh, Extra ref_extra, Image3 ref_texture, Mesh dis_mesh, Extra dis_extra, Image3 dis_texture, int& sub_flag, int& flag3_cal) {



	std::cout << "Cleaning reference mesh ..." << std::endl;

	Clean_Mesh ref_mesh_cleaned = mesh_clean(ref_mesh, ref_extra, ref_texture);

	std::cout << "Cleaning distorted mesh ..." << std::endl;

	Clean_Mesh dis_mesh_cleaned = mesh_clean(dis_mesh, dis_extra, dis_texture);


	int ref_face_num = ref_mesh_cleaned.face_count;
	int dis_face_num = dis_mesh_cleaned.face_count;
	std::cout << "Initial face of reference/distorted mesh: " << ref_face_num << "/" << dis_face_num << std::endl;
	if (ref_face_num > 1000000 || dis_face_num > 1000000) {
		std::cout << "The reference/distorted mesh has more than 1,000,000 faces, the prediction time may be long." << std::endl;
	}
	int ratio = 0;
	int bigobj_flag = 0;
	int vertex_num;

	Eigen::MatrixXd keypoints;
	

	if (dis_face_num > ref_face_num) {
		sub_flag = 1;
		std::cout << "Subdivided reference mesh..." << std::endl;
		int N = 0;
		while (std::pow(4, N) < ((float)dis_face_num / ref_face_num)) {
			N++;
		}
		ratio = N - 1;
		if (ratio >= 1) {
			std::cout << "Mid-point subdivision iteration: " << ratio << std::endl;
			Mesh ref_sub;
			Extra ref_sub_extra;
			subdivision(ref_mesh_cleaned, ref_sub, ref_sub_extra, ratio);
			std::cout << "Cleaning mid-point subdivided reference mesh..." << std::endl;
			Clean_Mesh  ref_sub_clean = mesh_clean(ref_sub, ref_sub_extra, ref_texture);
			std::cout << "Face of mid-point subdivided reference mesh: " << ref_sub_clean.face_count << std::endl;
			if (dis_face_num - ref_sub_clean.face_count > 0) {
				Mesh ref_sub2;
				Extra ref_sub_extra2;
				Targetsubdivision(ref_sub_clean, ref_sub2, ref_sub_extra2, dis_face_num);
				std::cout << "Cleaning target subdivided reference mesh..." << std::endl;
				Clean_Mesh  ref_sub_clean2 = mesh_clean(ref_sub2, ref_sub_extra2, ref_texture);
				ref_mesh_cleaned = ref_sub_clean2;
			}
			else {
				ref_mesh_cleaned = ref_sub_clean;
			}
		}
		else {
			std::cout << "Subdivided reference mesh for target number..." << std::endl;
			std::cout << "Target reference mesh faces: " << ref_face_num << std::endl;
			Mesh ref_sub;
			Extra ref_sub_extra;
			Targetsubdivision(ref_mesh_cleaned, ref_sub, ref_sub_extra, dis_face_num);
			std::cout << "Cleaning target subdivided reference mesh..." << std::endl;
			Clean_Mesh  ref_sub_clean = mesh_clean(ref_sub, ref_sub_extra, ref_texture);
			ref_mesh_cleaned = ref_sub_clean;
		}
	}
	else {
		if (dis_face_num < ref_face_num) {
			sub_flag = 2;
			int N = 0;
			while (std::pow(4, N) < ((float)ref_face_num / dis_face_num)) {
				N++;
			}
			ratio = N - 1;
			if (ratio >= 1) {
				std::cout << "Subdivided distorted mesh..." << std::endl;
				std::cout << "Mid-point subdivision iteration: " << ratio << std::endl;
				Mesh dis_sub;
				Extra dis_sub_extra;
				subdivision(dis_mesh_cleaned, dis_sub, dis_sub_extra, ratio);
				std::cout << "Cleaning mid-point subdivided distorted mesh..." << std::endl;
				Clean_Mesh  dis_sub_clean = mesh_clean(dis_sub, dis_sub_extra, dis_texture);
				std::cout << "Face of mid-point subdivided distorted mesh: " << dis_sub_clean.face_count << std::endl;
				if (ref_face_num - dis_sub_clean.face_count > 0) {
					Mesh dis_sub2;
					Extra dis_sub_extra2;
					Targetsubdivision(dis_sub_clean, dis_sub2, dis_sub_extra2, ref_face_num);
					std::cout << "Cleaning target subdivided distorted mesh..." << std::endl;
					Clean_Mesh  dis_sub_clean2 = mesh_clean(dis_sub2, dis_sub_extra2, dis_texture);
					dis_mesh_cleaned = dis_sub_clean2;
				}
				else {
					dis_mesh_cleaned = dis_sub_clean;
				}
			}
			else {
				std::cout << "Subdivided distorted mesh for target number..." << std::endl;
				std::cout << "Distorted mesh faces: " << dis_face_num << std::endl;
				std::cout << "Target distorted mesh faces: " << ref_face_num << std::endl;
				Mesh dis_sub;
				Extra dis_sub_extra;
				Targetsubdivision(dis_mesh_cleaned, dis_sub, dis_sub_extra, ref_face_num);
				std::cout << "Cleaning target subdivided distorted mesh..." << std::endl;
				Clean_Mesh  dis_sub_clean = mesh_clean(dis_sub, dis_sub_extra, dis_texture);
				dis_mesh_cleaned = dis_sub_clean;
			}
		}
	}





	vertex_num = ref_mesh_cleaned.vertexInFace_count;
	if (vertex_num < N) {
		N = vertex_num;
	}

	std::cout << "Maximum available keypoint: " << N << std::endl;
	std::cout << "Sampling keypoint (FPS) ..." << std::endl;

	keypoints = farthest_point_sampling(ref_mesh_cleaned, N);








	std::vector< SubFeature > feature(N);

	int keynum = N;

	double x_limit = ref_mesh_cleaned.vertexInFace.col(0).maxCoeff() - ref_mesh_cleaned.vertexInFace.col(0).minCoeff();
	double y_limit = ref_mesh_cleaned.vertexInFace.col(1).maxCoeff() - ref_mesh_cleaned.vertexInFace.col(1).minCoeff();
	double z_limit = ref_mesh_cleaned.vertexInFace.col(2).maxCoeff() - ref_mesh_cleaned.vertexInFace.col(2).minCoeff();

	double max_bbox = MAX(x_limit, MAX(y_limit, z_limit));

	double PatchRediux = 0.001 * max_bbox * 0.5;
	double threshold = PatchRediux;
	int max_local_patch_point = 50;

	std::cout << "Extracting  features ..." << std::endl;

	int PARALLEL_NUM;
	int PARALLEL_BLOCK;
	
	if (N < DEFAULT_PARALLEL_NUM) {
		PARALLEL_NUM = N;
		PARALLEL_BLOCK = 1;
	}
	else {
		PARALLEL_BLOCK = N / DEFAULT_PARALLEL_NUM;
		PARALLEL_NUM = DEFAULT_PARALLEL_NUM;
	}
	
	#pragma omp parallel for num_threads(PARALLEL_NUM+1)
	for (int parallel_idx = 0; parallel_idx < PARALLEL_NUM+1; ++parallel_idx) {
		Image3 private_ref_texture = ref_texture;
		Image3 private_dis_texture = dis_texture;
		Eigen::MatrixXd private_keypoints = keypoints;
		Clean_Mesh private_ref_mesh_cleaned = ref_mesh_cleaned;
		Clean_Mesh private_dis_mesh_cleaned = dis_mesh_cleaned;
		double private_threshold = threshold;
		for (int index_keypoint = parallel_idx * PARALLEL_BLOCK; index_keypoint < (parallel_idx + 1) * PARALLEL_BLOCK; ++index_keypoint) {
			if (index_keypoint >= N ) {
				break;
			}
			Eigen::VectorXd keypoint(3);

			keypoint << private_keypoints(index_keypoint, 0), private_keypoints(index_keypoint, 1), private_keypoints(index_keypoint, 2);

			Patch_Mesh ref_patch_mesh = create_local_patch(private_ref_mesh_cleaned, keypoint);
			Patch_Mesh dis_patch_mesh = create_local_patch(private_dis_mesh_cleaned, keypoint);
		
			int patch_num_ref = ref_patch_mesh.vertexInFace.rows();

			int patch_num_dis = dis_patch_mesh.vertexInFace.rows();

			bool patch_num_flag_ref = patch_num_ref > 3 ? true : false;
			bool patch_num_flag_dis = patch_num_dis > 3 ? true : false;
			double eps = 1e-6;
			if (patch_num_flag_ref && patch_num_flag_dis) {
				SubFeature subfeature;
				double ref_shift = ref_patch_mesh.center_shift;
				double dis_shift = dis_patch_mesh.center_shift;
				double curvature_shift = 0.0;
				if (ref_shift < dis_shift) {
					curvature_shift = dis_shift - ref_shift;
				}
				else {
					curvature_shift = ref_shift - dis_shift;
				}
				

				Mesh2Graph ref_Mesh_graph_ini = Mesh_to_graph(ref_patch_mesh);
				
				Mesh2Graph dis_Mesh_graph_ini = Mesh_to_graph(dis_patch_mesh);

				double ref_patch_size = 0.0;
				patch_cropping(ref_patch_mesh, dis_patch_mesh, ref_Mesh_graph_ini, dis_Mesh_graph_ini, private_ref_texture, private_dis_texture, private_threshold, ref_patch_size);

				if (0 <= curvature_shift && curvature_shift <= 1 * ref_patch_size) {
					subfeature.flag = 1;
					if (flag3_cal == 1) {
						flag3_cal = 0;
					}
				}
				else if (1 * ref_patch_size < curvature_shift && curvature_shift <= 2 * ref_patch_size)
				{
					subfeature.flag = 2;
					if (flag3_cal == 1) {
						flag3_cal = 0;
					}
				}
				else {
					subfeature.flag = 3;
				}

				if (subfeature.flag == 3 && flag3_cal == 0) {
					feature[index_keypoint] = subfeature;
					continue;
				}

				Mesh2Graph ref_Mesh_graph = Mesh_to_graph(ref_patch_mesh);
				
				Mesh2Graph dis_Mesh_graph = Mesh_to_graph(dis_patch_mesh);
				
				face_texture_cal(ref_patch_mesh, private_ref_texture);

				face_texture_cal(dis_patch_mesh, private_dis_texture);

				Eigen::MatrixXd ref_face_normal = ref_patch_mesh.faceNormal;
				Eigen::MatrixXd dis_face_normal = dis_patch_mesh.faceNormal;

				Eigen::MatrixXd ref_keypoint_normal(1, ref_face_normal.cols());
				int ref_face_normal_col = ref_face_normal.cols();
				for (int i = 0; i < ref_face_normal_col; ++i) {
					ref_keypoint_normal(0, i) = ref_patch_mesh.normal_weight.col(0).cwiseProduct(ref_face_normal.col(i)).sum();
				}
				
				int dis_face_normal_col = dis_face_normal.cols();
				Eigen::MatrixXd dis_keypoint_normal(1, dis_face_normal_col);
				for (int i = 0; i < dis_face_normal_col; ++i) {
					dis_keypoint_normal(0, i) = dis_patch_mesh.normal_weight.col(0).cwiseProduct(dis_face_normal.col(i)).sum();
				}






				Eigen::MatrixXd ref_graph_signal_c1 = ref_patch_mesh.vertexcolor.col(0);
				Eigen::MatrixXd ref_graph_signal_c2 = ref_patch_mesh.vertexcolor.col(1);
				Eigen::MatrixXd ref_graph_signal_c3 = ref_patch_mesh.vertexcolor.col(2);

				Eigen::MatrixXd ref_Laplacian = ref_Mesh_graph.D - ref_Mesh_graph.W;

				Eigen::MatrixXd fix_ref_D = ref_Mesh_graph.D;

				int ref_Mesh_graph_D_rows = ref_Mesh_graph.D.rows();
				for (int i = 0; i < ref_Mesh_graph_D_rows; ++i) {
					fix_ref_D(i, i) = 1 / (sqrt(fix_ref_D(i, i)+ eps ));
				}

				Eigen::MatrixXd ref_Laplacian_nor = fix_ref_D * ref_Laplacian * fix_ref_D;

				Eigen::MatrixXd dis_graph_signal_c1 = dis_patch_mesh.vertexcolor.col(0);
				Eigen::MatrixXd dis_graph_signal_c2 = dis_patch_mesh.vertexcolor.col(1);
				Eigen::MatrixXd dis_graph_signal_c3 = dis_patch_mesh.vertexcolor.col(2);
			
				Eigen::MatrixXd dis_Laplacian = dis_Mesh_graph.D - dis_Mesh_graph.W;

				Eigen::MatrixXd fix_dis_D = dis_Mesh_graph.D;

				int dis_Mesh_graph_D_row = dis_Mesh_graph.D.rows();
				for (int i = 0; i < dis_Mesh_graph_D_row; ++i) {
					fix_dis_D(i, i) = 1 / (sqrt(fix_dis_D(i, i) + eps));
				}

				Eigen::MatrixXd dis_Laplacian_nor = fix_dis_D * dis_Laplacian * fix_dis_D;
				
				Eigen::MatrixXd ref_smoothness1_nor = ref_graph_signal_c1.transpose() * ref_Laplacian_nor * ref_graph_signal_c1;
				Eigen::MatrixXd ref_smoothness2_nor = ref_graph_signal_c2.transpose() * ref_Laplacian_nor * ref_graph_signal_c2;
				Eigen::MatrixXd ref_smoothness3_nor = ref_graph_signal_c3.transpose() * ref_Laplacian_nor * ref_graph_signal_c3;

				Eigen::MatrixXd ref_smoothness(1, 3);
				ref_smoothness << ref_smoothness1_nor.cwiseAbs(), ref_smoothness2_nor.cwiseAbs(), ref_smoothness3_nor.cwiseAbs();

				Eigen::MatrixXd dis_smoothness1_nor = dis_graph_signal_c1.transpose() * dis_Laplacian_nor * dis_graph_signal_c1;
				Eigen::MatrixXd dis_smoothness2_nor = dis_graph_signal_c2.transpose() * dis_Laplacian_nor * dis_graph_signal_c2;
				Eigen::MatrixXd dis_smoothness3_nor = dis_graph_signal_c3.transpose() * dis_Laplacian_nor * dis_graph_signal_c3;

				Eigen::MatrixXd dis_smoothness(1, 3);
				dis_smoothness << dis_smoothness1_nor.cwiseAbs(), dis_smoothness2_nor.cwiseAbs(), dis_smoothness3_nor.cwiseAbs();
			
				int ref_center = 0, dis_center = 0;

				int ref_patch_mesh_patch_center_index_row = ref_patch_mesh.patch_center_index.rows();
				for (int j = 0; j < ref_patch_mesh_patch_center_index_row; ++j) {
					if (ref_patch_mesh.patch_center_index(j, 0) == 1)
						ref_center = j;
				}
					

				int dis_patch_mesh_patch_center_index_row = dis_patch_mesh.patch_center_index.rows();
				for (int j = 0; j < dis_patch_mesh_patch_center_index_row; ++j) {
					if (dis_patch_mesh.patch_center_index(j, 0) == 1)
						dis_center = j;
				}
					
	  			Eigen::MatrixXd ref_cot_l = cotmatrix(ref_patch_mesh.vertexInFace, ref_patch_mesh.face);
				Eigen::MatrixXd dis_cot_l = cotmatrix(dis_patch_mesh.vertexInFace, dis_patch_mesh.face);

				Eigen::MatrixXd ref_massmatrix = mass_cal(ref_patch_mesh);
				Eigen::MatrixXd dis_massmatrix = mass_cal(dis_patch_mesh);

				int ref_patch_mesh_vertexInFace_row = ref_patch_mesh.vertexInFace.rows();
				int ref_patch_mesh_vertexInFace_col = ref_patch_mesh.vertexInFace.cols();
				Eigen::MatrixXd ref_cur(1, ref_patch_mesh_vertexInFace_col);
				
				Eigen::MatrixXd diff_ref_cur(ref_patch_mesh_vertexInFace_row, ref_patch_mesh_vertexInFace_col);
				for (int i = 0; i < ref_patch_mesh_vertexInFace_row; ++i) {
					diff_ref_cur(i, 0) = ref_patch_mesh.vertexInFace(ref_center, 0) - ref_patch_mesh.vertexInFace(i, 0);
					diff_ref_cur(i, 1) = ref_patch_mesh.vertexInFace(ref_center, 1) - ref_patch_mesh.vertexInFace(i, 1);
					diff_ref_cur(i, 2) = ref_patch_mesh.vertexInFace(ref_center, 2) - ref_patch_mesh.vertexInFace(i, 2);
				}



				ref_cur = 1 / (2 * ref_massmatrix(ref_center, ref_center)) * ref_cot_l.row(ref_center) * diff_ref_cur;
			

				int dis_patch_mesh_vertexInFace_row = dis_patch_mesh.vertexInFace.rows();
				int dis_patch_mesh_vertexInFace_col = dis_patch_mesh.vertexInFace.cols();
				Eigen::MatrixXd dis_cur(1, dis_patch_mesh_vertexInFace_col);

				Eigen::MatrixXd diff_dis_cur(dis_patch_mesh_vertexInFace_row, dis_patch_mesh_vertexInFace_col);
				for (int i = 0; i < dis_patch_mesh_vertexInFace_row; ++i) {
					diff_dis_cur(i, 0) = dis_patch_mesh.vertexInFace(dis_center, 0) - dis_patch_mesh.vertexInFace(i, 0);
					diff_dis_cur(i, 1) = dis_patch_mesh.vertexInFace(dis_center, 1) - dis_patch_mesh.vertexInFace(i, 1);
					diff_dis_cur(i, 2) = dis_patch_mesh.vertexInFace(dis_center, 2) - dis_patch_mesh.vertexInFace(i, 2);
				}
				

				dis_cur = 1 / (2 * dis_massmatrix(dis_center, dis_center)) * dis_cot_l.row(dis_center) * diff_dis_cur;

				
				Eigen::MatrixXd ref_cur_value = ref_cur * ref_keypoint_normal.transpose();
				Eigen::MatrixXd dis_cur_value = dis_cur * dis_keypoint_normal.transpose();
			
				

				subfeature.ref_facelm = (ref_patch_mesh.l_mean);
				subfeature.dis_facelm = (dis_patch_mesh.l_mean);
				subfeature.ref_facevar = (ref_patch_mesh.l_var);
				subfeature.dis_facevar = (dis_patch_mesh.l_var);
				subfeature.ref_facec1m = (ref_patch_mesh.c1_mean);
				subfeature.dis_facec1m = (dis_patch_mesh.c1_mean);
				subfeature.ref_facec1var = (ref_patch_mesh.c1_var);
				subfeature.dis_facec1var = (dis_patch_mesh.c1_var);
				subfeature.ref_facec2m = (ref_patch_mesh.c2_mean);
				subfeature.dis_facec2m = (dis_patch_mesh.c2_mean);
				subfeature.ref_facec2var = (ref_patch_mesh.c2_var);
				subfeature.dis_facec2var = (dis_patch_mesh.c2_var);
				subfeature.ref_cur_value = ref_cur_value(0,0);
				subfeature.dis_cur_value = dis_cur_value(0,0);
			
				
				subfeature.ref_vertex_eff = ref_patch_mesh.pointInface_num;
				subfeature.dis_vertex_eff = dis_patch_mesh.pointInface_num;
			
	
				subfeature.ref_smoothness = ref_smoothness; 
				subfeature.dis_smoothness = dis_smoothness;
				feature[index_keypoint] = subfeature;
			}
			else {
				SubFeature subfeature;
				subfeature.flag = 0;
				feature[index_keypoint] = subfeature;
			}
		}
	}
	return feature;
}
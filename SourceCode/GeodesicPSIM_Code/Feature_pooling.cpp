#include "headfile.h"
Eigen::MatrixXd similarity(Eigen::MatrixXd a, Eigen::MatrixXd b) {
	double t = 1e-6;
	Eigen::MatrixXd sim(a.rows(), a.cols());
	Eigen::MatrixXd sim_0(a.rows(), a.cols());
	Eigen::MatrixXd sim_1(a.rows(), a.cols());
	for (int i = 0; i < a.rows(); ++i) {
		for (int j = 0; j < a.cols(); ++j) {
			sim_0(i, j) = fabs(2 * a(i, j) * b(i, j)) + t;
		}
	}
	for (int i = 0; i < a.rows(); ++i) {
		for (int j = 0; j < a.cols(); ++j) {
			sim_1(i, j) = a(i, j) * a(i, j) + b(i, j) * b(i, j) + t;
		}
	}

	for (int i = 0; i < a.rows(); ++i) {
		for (int j = 0; j < a.cols(); ++j) {
			sim(i, j) = sim_0(i, j) / sim_1(i, j);
		}
	}
	return sim;
}

double similarity2(double a, double b) {
	double t = 1e-6;
	double sim = (2 * a * b + t) / (a * a + b * b + t);
	return sim;
}

double mean(std::vector<double> arrays) {
	double sums = 0.0;
	double size = arrays.size();
	for (int i = 0; i < arrays.size(); ++i) {
		if (!std::isnan(arrays[i])) {
			sums += arrays[i];
		}
		else {
			size -= 1;
		}
	}
	return sums / size;
}


bool write_result(std::string result_file, double pcs, double dmc, double pca, double pcv, 
	double GeodesicPSIM, int num_patch_effective, int flag_1, int flag_2, int flag_3,std::string dir_reference, std::string dir_reference_png, std::string dir_distortion,
	std::string dir_distortion_pn, int sub_flag) {
	int length;
	std::ifstream filestr;
	filestr.open(result_file, std::ios::binary);
	filestr.seekg(0, std::ios::end);
	length = filestr.tellg();
	filestr.close();
	std::ofstream GeodesicPSIM_write(result_file, std::ios::app);

	if (GeodesicPSIM_write.is_open()) {

		if (length == -1)
		{
			GeodesicPSIM_write << "Reference mesh obj file" << ",";
			GeodesicPSIM_write << "Reference mesh texture map file" << ",";
			GeodesicPSIM_write << "Distorted mesh obj file" << ",";
			GeodesicPSIM_write << "Distorted mesh texture map file" << ",";
			GeodesicPSIM_write << "Mesh subdivision object" << ",";
			GeodesicPSIM_write << "Effective num of patch" << ",";
			GeodesicPSIM_write << "Num of flag 1" << ",";
			GeodesicPSIM_write << "Num of flag 2" << ",";
			GeodesicPSIM_write << "Num of flag 3" << ",";
			GeodesicPSIM_write << "Patch color smoothness" << ",";
			GeodesicPSIM_write << "Patch discrete mean curvature" << ",";
			GeodesicPSIM_write << "Patch pixel color average" << ",";
			GeodesicPSIM_write << "Patch pixel color variance" << ",";
			GeodesicPSIM_write << "GeodesicPSIM score" << ",";

		}
		std::string sub_obj;
		if (sub_flag == 1) {
			sub_obj = "Reference Mesh";
		}
		else if (sub_flag ==2) {
			sub_obj = "Distorted Mesh";
		}
		else {
			sub_obj = "No subdivision";
		}
		GeodesicPSIM_write << "\n";
		GeodesicPSIM_write << dir_reference << ",";
		GeodesicPSIM_write << dir_reference_png << ",";
		GeodesicPSIM_write << dir_distortion << ",";
		GeodesicPSIM_write << dir_distortion_pn << ",";
		GeodesicPSIM_write << sub_obj << ",";
		GeodesicPSIM_write << num_patch_effective << ",";
		GeodesicPSIM_write << flag_1 << ",";
		GeodesicPSIM_write << flag_2 << ",";
		GeodesicPSIM_write << flag_3 << ",";
		GeodesicPSIM_write << pcs << ",";
		GeodesicPSIM_write << dmc << ",";
		GeodesicPSIM_write << pca << ",";
		GeodesicPSIM_write << pcv << ",";
		GeodesicPSIM_write << GeodesicPSIM << ",";
		GeodesicPSIM_write.close();

		return true;
	}
	else
		std::cout << "Unable to open" << result_file << "for writing";
		GeodesicPSIM_write.close();
		return false;
}


double Feature_pooling(std::vector< SubFeature> feature, 
	std::string result_file, 
	std::string dir_reference, 
	std::string dir_reference_png, 
	std::string dir_distortion, 
	std::string dir_distortion_png, 
	int sub_flag, 
	int flag3_cal) {

	int size = feature.size();

	
	std::vector<double>  average_pcs(size), average_pcg(size);
	std::vector<double> average_dmc(size), average_face_lm(size), average_face_var(size), average_face_c1m(size), average_face_c1var(size), average_face_c2m(size), average_face_c2var(size);
	
	double pcs = 0.0;

	double dmc = 0.0;
	double face_lm = 0.0;
	double face_var = 0.0;
	double face_c1m = 0.0;
	double face_c1var = 0.0;
	double face_c2m = 0.0;
	double face_c2var = 0.0;
	int count = 0;
	int count_dmc = 0;


	double pcs_flag2 = 0.0;

	double dmc_flag2 = 0.0;
	double face_lm_flag2 = 0.0;
	double face_var_flag2 = 0.0;
	double face_c1m_flag2 = 0.0;
	double face_c1var_flag2 = 0.0;
	double face_c2m_flag2 = 0.0;
	double face_c2var_flag2 = 0.0;
	int count_flag2 = 0;
	int count_dmc_flag2 = 0;


	double pcs_flag3 = 0.0;

	double dmc_flag3 = 0.0;
	double face_lm_flag3 = 0.0;
	double face_var_flag3 = 0.0;
	double face_c1m_flag3 = 0.0;
	double face_c1var_flag3 = 0.0;
	double face_c2m_flag3 = 0.0;
	double face_c2var_flag3 = 0.0;
	int count_flag3 = 0;
	int count_dmc_flag3 = 0;
	
	
	for (int k = 0; k < feature.size(); ++k) 
	{
		SubFeature proposed = feature[k];
		if (proposed.flag == 0) {
			continue;
		}
		if (proposed.flag == 3 && flag3_cal == 0) {
			count_flag3++;
			continue;
		}

		Eigen::MatrixXi ref_n = proposed.ref_vertex_eff;
		Eigen::MatrixXi dis_n = proposed.dis_vertex_eff;

		Eigen::MatrixXd weighted_ref_n = ref_n.cast<double>() / ref_n.sum();
		Eigen::MatrixXd weighted_dis_n = dis_n.cast<double>() / dis_n.sum();

		Eigen::MatrixXd ref_smoothness = proposed.ref_smoothness;
		Eigen::MatrixXd dis_smoothness = proposed.dis_smoothness;








		int ref_num_edge = ref_n.rows();
		int dis_num_edge = dis_n.rows();

		Eigen::MatrixXd smoothness_diff = similarity(ref_smoothness / ref_n.sum(), dis_smoothness / dis_n.sum());

		average_pcs[k] = (6 * smoothness_diff(0, 0) + smoothness_diff(0, 1) + smoothness_diff(0, 2)) / 8;

		average_dmc[k] = similarity2(proposed.ref_cur_value, proposed.dis_cur_value);

		average_face_lm[k] = similarity2((weighted_ref_n.cwiseProduct(proposed.ref_facelm)).sum(), (weighted_dis_n.cwiseProduct(proposed.dis_facelm)).sum());
		average_face_var[k] = similarity2((weighted_ref_n.cwiseProduct(proposed.ref_facevar)).sum(), (weighted_dis_n.cwiseProduct(proposed.dis_facevar)).sum());

		average_face_c1m[k] = similarity2((weighted_ref_n.cwiseProduct(proposed.ref_facec1m)).sum(), (weighted_dis_n.cwiseProduct(proposed.dis_facec1m)).sum());
		average_face_c1var[k] = similarity2((weighted_ref_n.cwiseProduct(proposed.ref_facec1var)).sum(), (weighted_dis_n.cwiseProduct(proposed.dis_facec1var)).sum());

		average_face_c2m[k] = similarity2((weighted_ref_n.cwiseProduct(proposed.ref_facec2m)).sum(), (weighted_dis_n.cwiseProduct(proposed.dis_facec2m)).sum());
		average_face_c2var[k] = similarity2((weighted_ref_n.cwiseProduct(proposed.ref_facec2var)).sum(), (weighted_dis_n.cwiseProduct(proposed.dis_facec2var)).sum());
		

		if (proposed.flag == 1) {
			count++;
			pcs += average_pcs[k];


			if (!std::isnan(average_dmc[k])) {
				if (average_dmc[k] >= 0) {
					dmc += average_dmc[k];
				}
				else
				{
					double dis = fabs(proposed.ref_cur_value - proposed.dis_cur_value);
					double ref_cur = proposed.ref_cur_value + dis;
					double dis_cur = proposed.dis_cur_value + dis;
					average_dmc[k] = similarity2(ref_cur, dis_cur);
					dmc += average_dmc[k];
				}
				count_dmc += 1;
			}
			face_lm += average_face_lm[k];
			face_var += average_face_var[k];
			face_c1m += average_face_c1m[k];
			face_c1var += average_face_c1var[k];
			face_c2m += average_face_c2m[k];
			face_c2var += average_face_c2var[k];
		}
		else if (proposed.flag == 2){
			count_flag2++;
			pcs_flag2 += average_pcs[k];

			if (!std::isnan(average_dmc[k])) {
				if (average_dmc[k] >= 0) {
					dmc_flag2 += average_dmc[k];
				}
				else
				{
					double dis = fabs(proposed.ref_cur_value - proposed.dis_cur_value);
					double ref_cur = proposed.ref_cur_value + dis;
					double dis_cur = proposed.dis_cur_value + dis;
					average_dmc[k] = similarity2(ref_cur, dis_cur);
					dmc_flag2 += average_dmc[k];
				}
				count_dmc_flag2 += 1;
			}
			face_lm_flag2 += average_face_lm[k];
			face_var_flag2 += average_face_var[k];
			face_c1m_flag2 += average_face_c1m[k];
			face_c1var_flag2 += average_face_c1var[k];
			face_c2m_flag2 += average_face_c2m[k];
			face_c2var_flag2 += average_face_c2var[k];
			
		}
		else if (proposed.flag == 3) {
			count_flag3++;
			pcs_flag3 += average_pcs[k];

			if (!std::isnan(average_dmc[k])) {
				if (average_dmc[k] >= 0) {
					dmc_flag3 += average_dmc[k];
				}
				else
				{
					double dis = fabs(proposed.ref_cur_value - proposed.dis_cur_value);
					double ref_cur = proposed.ref_cur_value + dis;
					double dis_cur = proposed.dis_cur_value + dis;
					average_dmc[k] = similarity2(ref_cur, dis_cur);
					dmc_flag3 += average_dmc[k];
				}
				count_dmc_flag3 += 1;
			}
			face_lm_flag3 += average_face_lm[k];
			face_var_flag3 += average_face_var[k];
			face_c1m_flag3 += average_face_c1m[k];
			face_c1var_flag3 += average_face_c1var[k];
			face_c2m_flag3 += average_face_c2m[k];
			face_c2var_flag3 += average_face_c2var[k];
		}
	}

	if (count + count_flag2 + count_flag3 == 0 || count_dmc + count_dmc_flag2 + count_dmc_flag3 == 0) {
		std::cout << "Too few useful keypoints, the result is meaningless." << std::endl;
		std::cout << "Try larger number of keypoints." << std::endl;
	}

	double score_face_lm = 0;
	double score_face_var = 0;

	double score_face_c1m = 0;
	double score_face_c1var = 0;
	double score_face_c2m = 0;
	double score_face_c2var = 0;

	double pcv = 0;
	double pca = 0;
	double obj_score = 0;
	
	double flag1_flag = 0;
	if (count > 0 && count_dmc > 0) {
		flag1_flag = 1;
		pcs = pcs / count;
		dmc = dmc / count_dmc;
		score_face_lm = face_lm / count;
		score_face_var = face_var / count;
		score_face_c1m = face_c1m / count;
		score_face_c1var = face_c1var / count;
		score_face_c2m = face_c2m / count;
		score_face_c2var = face_c2var / count;

		pcv = (6 * score_face_var + score_face_c1var + score_face_c2var) / 8;
		pca = (6 * score_face_lm + score_face_c1m + score_face_c2m) / 8;
	}
	double ratio = 0;
	if (count_flag2 > 0 && count_dmc_flag2 > 0) {
		std::cout << "Count_flag2: " << count_flag2 << std::endl;
		ratio = double(count_flag2) / (count + count_flag2);
		std::cout << "Flag2 exist and the ratio is: " << ratio << std::endl;
		pcs_flag2 = pcs_flag2 / count_flag2;
		dmc_flag2 = dmc_flag2 / count_dmc_flag2;
		double score_face_lm_flag2 = face_lm_flag2 / count_flag2;
		double score_face_var_flag2 = face_var_flag2 / count_flag2;
		double score_face_c1m_flag2 = face_c1m_flag2 / count_flag2;
		double score_face_c1var_flag2 = face_c1var_flag2 / count_flag2;
		double score_face_c2m_flag2 = face_c2m_flag2 / count_flag2;
		double score_face_c2var_flag2 = face_c2var_flag2 / count_flag2;


		double pcv_flag2 = (6 * score_face_var_flag2 + score_face_c1var_flag2 + score_face_c2var_flag2) / 8;
		double pca_flag2 = (6 * score_face_lm_flag2 + score_face_c1m_flag2 + score_face_c2m_flag2) / 8;
		pcs = (pcs + ratio * pcs_flag2) / (flag1_flag + ratio);
		dmc = (dmc + ratio * dmc_flag2) / (flag1_flag + ratio);
		pcv = (pcv + ratio * pcv_flag2) / (flag1_flag + ratio);
		pca = (pca + ratio * pca_flag2) / (flag1_flag + ratio);
	}
	int effective_num_patch = count + count_flag2;
	if (effective_num_patch > 0) {
		obj_score = (pcs  + dmc + pcv + pca) / 4;
		write_result(result_file, pcs, dmc, pca, pcv, obj_score, effective_num_patch, count, count_flag2, count_flag3, dir_reference, dir_reference_png, dir_distortion, dir_distortion_png, sub_flag);
		return obj_score;
	}
	else {
		pcs = pcs_flag3 / count_flag3;
		dmc = dmc_flag3 / count_dmc_flag3;
		score_face_lm = face_lm_flag3 / count_flag3;
		score_face_var = face_var_flag3 / count_flag3;
		score_face_c1m = face_c1m_flag3 / count_flag3;
		score_face_c1var = face_c1var_flag3 / count_flag3;
		score_face_c2m = face_c2m_flag3 / count_flag3;
		score_face_c2var = face_c2var_flag3 / count_flag3;

		pcv = (6 * score_face_var + score_face_c1var + score_face_c2var) / 8;
		pca = (6 * score_face_lm + score_face_c1m + score_face_c2m) / 8;
		obj_score = (pcs + dmc + pcv + pca) / 4;
		write_result(result_file, pcs, dmc, pca, pcv, obj_score, count_flag3, count, count_flag2, count_flag3, dir_reference, dir_reference_png, dir_distortion, dir_distortion_png, sub_flag);
		return obj_score;
	}
}

#include "headfile.h"

double GeodesicPSIM(int N, std::string dir_reference, std::string dir_distortion, std::string dir_reference_png, std::string dir_distortion_png, std::string result_file) {

	Mesh ref_mesh, dis_mesh;
	Extra ref_extra, dis_extra;
	double start, end;

	std::cout << "Loding reference Mesh ..."<<std::endl;
	start = clock();
	read_obj(dir_reference, ref_mesh, ref_extra);
	end = clock();
	std::cout << "Loding reference Mesh Time: " << (end - start) / CLOCKS_PER_SEC << std::endl;

	std::cout << "Loding distorted Mesh ..." << std::endl;;
	start = clock();

	read_obj(dir_distortion, dis_mesh, dis_extra);

	end = clock();
	std::cout << "Loding distorted Mesh Time: " << (end - start) / CLOCKS_PER_SEC << std::endl;

	cv::Mat ref_pic = cv::imread(dir_reference_png, 1);
	cv::Mat dis_pic = cv::imread(dir_distortion_png, 1);

	Image3 ref_texture;
	Image3 dis_texture;

	Image3_read(ref_texture, ref_pic);
	Image3_read(dis_texture, dis_pic);

	std::vector<SubFeature> feature = Feature_extraction(N, ref_mesh, ref_extra, ref_texture, dis_mesh, dis_extra, dis_texture);

	std::cout << "Feature pooling ..." << std::endl;
	double score = Feature_pooling(feature, result_file, dir_reference, dir_reference_png, dir_distortion, dir_distortion_png);

	return score;
}
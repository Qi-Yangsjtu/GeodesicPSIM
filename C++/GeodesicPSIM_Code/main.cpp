#include "headfile.h"


int main(int argc, char** argv) {

	double start, end;
	int N = 500;
	std::string result_file = "result.csv";
	
	std::cout << "GeodesicPSIM C++ Implementation" << std::endl;
	std::cout << "Copyright: Tencent Media Lab" << std::endl;
	std::cout << "Author: Chino Yang (Qi Yang), chinoyang@tencent.com." << std::endl;
	std::cout << "https://multimedia.tencent.com/resources/GeodesicPSIM" << std::endl;
	std::cout << "********************************************************************" << std::endl;
	///*
	if (argc < 3) {
		std::cerr << "Usage: " <<std::endl << "GeodesicPSIM.exe ReferenceMesh(obj) ReferenceMeshTextureMap DistortedMesh(obj) DistortedMeshTextureMap" << std::endl;
		std::cerr << "(Options) (--N Number of keypoints) (Options) (--result_file CSV file to save results) "<<std::endl;
		return -1;
	}

	std::string dir_reference = argv[1];
	std::string dir_reference_png = argv[2];
	std::string dir_distortion = argv[3];
	std::string dir_distortion_png = argv[4];
	for (int i = 5; i < argc; ++i) {
		if (std::string(argv[i]) == "--N") {
			i++;
			N = std::stod(std::string(argv[i]));		
		}
		if (std::string(argv[i]) == "--result_file") {
			i++;
			result_file = (std::string(argv[i]));
		}

	}
	//*/
	std::cout << "The number of keypoint is set to : " << N << std::endl;
	if (N < 1) {
		std::cout << "The number of keypoint cannot be 0!" << std::endl;
		return -1;
	}
	std::cout << "Results are stored in: " << result_file << std::endl;
	start = clock();

	double score = GeodesicPSIM(N, dir_reference, dir_distortion, dir_reference_png, dir_distortion_png, result_file);

	end = clock();

	std::cout << "GeodesicPSIM score is: " << score << std::endl;

	std::cout << "Using Time: " << (end - start) / CLOCKS_PER_SEC << std::endl;
	//system("pause");
	return 0;
}


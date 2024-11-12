#include "headfile.h"

void Image3_Init(Image3 &image, int width, int height) {
	image.row = width;
	image.col = height;
	image.r.conservativeResize(width, height);
	image.g.conservativeResize(width, height);
	image.b.conservativeResize(width, height);
}

void Image3_read(Image3& image, cv::Mat pic) {

	Image3_Init(image, pic.rows, pic.cols);

	for (int i = 0; i < pic.rows; ++i) {
		for (int j = 0; j < pic.cols; ++j) {
			cv::Vec3b pixel = pic.at<cv::Vec3b>(i, j);
			image.r(i, j) = static_cast<float>(pixel[2]); 
			image.g(i, j) = static_cast<float>(pixel[1]); 
			image.b(i, j) = static_cast<float>(pixel[0]); 
		}
	}
}



void unique(std::vector<int>& unique_ordered, Eigen::MatrixXi input) {
	std::unordered_map<Eigen::MatrixXi, int, MatrixXiHash> myMap;
	int input_row = input.rows();

	for (int i = 0; i < input_row; ++i) {
		Eigen::MatrixXi this_row = input.row(i);

		if (myMap.find(this_row) == myMap.end()) {
			myMap[this_row] = i;
			unique_ordered.push_back(i);
		}
	}
}

void uniqued(std::vector<int>& unique_ordered, Eigen::MatrixXd input) {
	std::unordered_map<Eigen::MatrixXd, int, MatrixXdHash> myMap;
	int input_row = input.rows();

	for (int i = 0; i < input_row; ++i) {
		Eigen::MatrixXd this_row = input.row(i);

		if (myMap.find(this_row) == myMap.end()) {
			myMap[this_row] = i;
			unique_ordered.push_back(i);
		}
	}
}
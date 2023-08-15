#include "headfile.h"


Eigen::MatrixXd farthest_point_sampling(Clean_Mesh dis_mesh, int N) {

	int num_points = dis_mesh.vertexInFace_count;

	Eigen::MatrixXd keypoint(N, 3);

	int start_index = rand() % num_points;
	keypoint.row(0) = dis_mesh.vertexInFace.row(start_index);

	std::vector<double> distances(num_points);
	
	for (int i = 0; i < num_points; ++i) {
		distances[i] = (dis_mesh.vertexInFace.row(i) - keypoint.row(0)).norm();
	}

	for (int i = 1; i < N; ++i) {
		auto maxElement = std::max_element(distances.begin(), distances.end());
		int farthest_index = std::distance(distances.begin(), maxElement);

		keypoint.row(i) = dis_mesh.vertexInFace.row(farthest_index);

		for (int j = 0; j < num_points; ++j) {
			distances[j] = MIN((dis_mesh.vertexInFace.row(j) - keypoint.row(i)).norm(), distances[j]);
		}
	}
	return keypoint;
}
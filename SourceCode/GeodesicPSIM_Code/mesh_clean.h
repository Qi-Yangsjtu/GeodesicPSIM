#pragma once
#include "headfile.h"

struct Vector3dEqual {
	bool operator()(const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs) const {
		return lhs.isApprox(rhs);
	}
};

struct Vector3dHash {
	std::size_t operator()(const Eigen::Vector3d& vec) const {
		std::size_t hx = std::hash<double>()(vec.x());
		std::size_t hy = std::hash<double>()(vec.y()) << 1;
		std::size_t hz = std::hash<double>()(vec.z()) << 2;
		return hx ^ hy ^ hz;
	}
};


struct Vector2dEqual {
	bool operator()(const Eigen::Vector2d& lhs, const Eigen::Vector2d& rhs) const {
		return lhs.isApprox(rhs);
	}
};

struct Vector2dHash {
	std::size_t operator()(const Eigen::Vector2d& vec) const {
		std::size_t hx = std::hash<double>()(vec.x());
		std::size_t hy = std::hash<double>()(vec.y()) << 1;
		return hx ^ hy;
	}
};

std::vector<int> processIndexes(const std::vector<int>& index, const std::vector<int>& _int);
Clean_Mesh mesh_clean(Mesh mesh, Extra extra, Image3 texture);
void mesh_clean_process_3(Clean_Mesh& ret_mesh, Image3 texture);
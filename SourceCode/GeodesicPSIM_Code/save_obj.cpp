#include "headfile.h"
void Save_Obj(Clean_Mesh mesh, std::string dir, std::string mtl) {


	Eigen::MatrixXi face = mesh.face;
	Eigen::MatrixXd vertex = mesh.vertex;
	Eigen::MatrixXd texture_coor = mesh.extra.textures;
	Eigen::MatrixXi face_texture_coor = mesh.extra.face_texture;


	std::ofstream file(dir);

	file << "mtllib " << mtl << std::endl;

	for (int i = 0; i < vertex.rows(); i++) {
		file << "v " << vertex(i, 0) << " " << vertex(i, 1) << " " << vertex(i, 2) << std::endl;
	}


	for (int i = 0; i < texture_coor.rows(); i++) {
		file << "vt " << texture_coor(i, 0) << " " << texture_coor(i, 1) << std::endl;
	}


	for (int i = 0; i < face.rows(); i++) {
		file << "f ";
		file << face(i, 0) + 1 << "/" << face_texture_coor(i, 0) + 1 << " "
			<< face(i, 1) + 1 << "/" << face_texture_coor(i, 1) + 1 << " "
			<< face(i, 2) + 1 << "/" << face_texture_coor(i, 2) + 1;
		file << std::endl;
	}

	file.close();

}
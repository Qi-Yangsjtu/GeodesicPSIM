#include <string>
#include <fstream>
#include <sstream>
#include "headfile.h"

bool readfilename(std::string filename, Vector_Mesh& mesh, Extra& extra)
{
	std::ifstream file;
	char* buf = new char[4 * 1024 * 1024 + 1];
	file.rdbuf()->pubsetbuf(buf, 4 * 1024 * 1024 + 1);
	file.open(filename, std::ios::in);
	if (!file.good()) {
		std::cout << "File " << filename << " does not exist!" << std::endl;
		return false;
	}

	if (!file)
	{
		std::cerr << "Error: can't open file " << filename << std::endl;
		delete[] buf;
		return false;
	}
	std::string line;
	double v_index = 0;
	double vt_index = 0;
	double vn_index = 0;
	double f_index = 0;
	while (getline(file, line))
	{
		if (line.substr(0, 2) == "v ") // 
		{
			std::istringstream ss(line.substr(2));
			std::vector< double > this_vertice;
			std::string word;
			while (ss >> word)
			{
				this_vertice.push_back(std::stof(word));
			}

			mesh.vertices.push_back(this_vertice);

		}

		else if (line.substr(0, 3) == "vt ") // 
		{
			std::istringstream ss(line.substr(2));
			std::vector< double > this_texture;
			std::string word;
			while (ss >> word)
			{
				this_texture.push_back(std::stof(word));
			}
			
			mesh.textures.push_back(this_texture);

		}

		else if (line.substr(0, 3) == "vn ") // 
		{
			std::istringstream ss(line.substr(2));
			std::vector< double > this_normal;
			std::string word;
			while (ss >> word)
			{
				this_normal.push_back(std::stof(word));
			}

			mesh.normals.push_back(this_normal);

		}

		else if (line.substr(0, 2) == "f ") // 
		{
			std::istringstream ss(line.substr(2));
			std::vector< int > this_face;
			std::string word;
			std::string num1, num2, num3;
			while (ss >> word)
			{
				size_t pos = word.find('/');
				num1 = word.substr(0, pos);
				num2 = word.substr(pos + 1);

				size_t pos_2 = num2.find('/');

				if (pos_2 <= num2.length()) {
					num3 = num2.substr(pos_2 + 1);
					num2 = num2.substr(0, pos_2);
					this_face.push_back(std::stoi(num1) - 1);
					this_face.push_back(std::stoi(num2) - 1);
					this_face.push_back(std::stoi(num3) - 1);
				}
				else {
					this_face.push_back(std::stoi(num1) - 1);
					this_face.push_back(std::stoi(num2) - 1);
				}
			}

			mesh.faces.push_back(this_face);

		}

		else if (line[0] == '#') 
		{

		}
		else
		{

		}
	}
	file.close();
	delete[] buf;
	return true;
}


void read_obj(std::string filename, Mesh& mesh, Extra& extra) {


	Vector_Mesh vector_mesh;
	std::cout << "File name:" << filename << std::endl;

	readfilename(filename, vector_mesh, extra);



	int vertices_row_num = vector_mesh.vertices.size();

	int vertices_col_num = vector_mesh.vertices[0].size();

	int faces_row_num = vector_mesh.faces.size();

	int textures_row_num = vector_mesh.textures.size();

	int normals_row_num = vector_mesh.normals.size();


	mesh.vertices.conservativeResize(vertices_row_num, 3);


	for (int i = 0; i < vertices_row_num; ++i) {
		mesh.vertices.row(i) << vector_mesh.vertices[i][0], vector_mesh.vertices[i][1], vector_mesh.vertices[i][2];
	}

	if (vertices_col_num > 3) {
		extra.vertex_color.conservativeResize(vertices_row_num, 3);
		for (int i = 0; i < vertices_row_num; ++i) {
			extra.vertex_color.row(i) << vector_mesh.vertices[i][3], vector_mesh.vertices[i][4], vector_mesh.vertices[i][5];
		}
	}


	mesh.faces.conservativeResize(faces_row_num, 3);
	extra.textures.conservativeResize(textures_row_num, 2);
	extra.normals.conservativeResize(normals_row_num, 3);


	if (textures_row_num > 0) {
		for (int i = 0; i < textures_row_num; ++i) {
			extra.textures.row(i) << vector_mesh.textures[i][0], vector_mesh.textures[i][1];
		}
	}

	if (normals_row_num > 0) {
		for (int i = 0; i < normals_row_num; ++i) {
			extra.normals.row(i) << vector_mesh.normals[i][0], vector_mesh.normals[i][1], vector_mesh.normals[i][2];
		}
	}


	extra.face_texture.conservativeResize(faces_row_num, 3);
	extra.face_normal.conservativeResize(faces_row_num, 3);

	if (textures_row_num && normals_row_num && vector_mesh.faces[0].size() == 9) {
		for (int i = 0; i < faces_row_num; ++i) {
			mesh.faces.row(i) << vector_mesh.faces[i][0], vector_mesh.faces[i][3], vector_mesh.faces[i][6];
			extra.face_texture.row(i) << vector_mesh.faces[i][1], vector_mesh.faces[i][4], vector_mesh.faces[i][7];
			extra.face_normal.row(i) << vector_mesh.faces[i][2], vector_mesh.faces[i][5], vector_mesh.faces[i][8];
		}
	}

	else if (textures_row_num && vector_mesh.faces[0].size() == 6) {

		for (int i = 0; i < faces_row_num; ++i) {
			mesh.faces.row(i) << vector_mesh.faces[i][0], vector_mesh.faces[i][2], vector_mesh.faces[i][4];
			extra.face_texture.row(i) << vector_mesh.faces[i][1], vector_mesh.faces[i][3], vector_mesh.faces[i][5];
		}
	}

	else if (!textures_row_num && normals_row_num && vector_mesh.faces[0].size() == 6) {

		for (int i = 0; i < faces_row_num; ++i) {
			mesh.faces.row(i) << vector_mesh.faces[i][0], vector_mesh.faces[i][2], vector_mesh.faces[i][4];
			extra.face_normal.row(i) << vector_mesh.faces[i][1], vector_mesh.faces[i][3], vector_mesh.faces[i][5];
		}
	}
	else {
		std::cout << "Corrupted mesh! Please check your obj file.";
		std::exit(0);
	}
}



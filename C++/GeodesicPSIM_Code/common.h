#pragma once
#include "headfile.h"

struct Vector_Mesh {
	std::vector < std::vector < double > > vertices;
	std::vector < std::vector < int > > faces;
	std::vector < std::vector < double > > textures;
	std::vector < std::vector < double > > normals;
};

struct Mesh
{
	Eigen::MatrixXd vertices;
	Eigen::MatrixXi faces;
};

struct Extra
{
	Eigen::MatrixXd vertex_color;
	Eigen::MatrixXd textures;
	Eigen::MatrixXd normals;
	Eigen::MatrixXi face_texture;
	Eigen::MatrixXi face_normal;

};

struct Image3
{
	int row;
	int col;
	Eigen::MatrixXi r;
	Eigen::MatrixXi g;
	Eigen::MatrixXi b;
};


struct Clean_Mesh {
	Eigen::MatrixXi face;
	Eigen::MatrixXd vertex;

	Extra extra;
	Image3 texturemap;

	int face_count;
	int vertex_count;

	Eigen::MatrixXi vertexcolor_rgb;
	Eigen::MatrixXd vertexInFace;

	int vertexInFace_count;

	Eigen::MatrixXi texture_coor;

	Eigen::MatrixXd vertexcolor;
};


struct Patch_Mesh
{
	Eigen::MatrixXd vertexInFace;
	int vertexInFace_count;
	Eigen::MatrixXd vertexcolor;
	Eigen::MatrixXi texture_coor;
	Eigen::MatrixXd patch_center_index;
	Eigen::MatrixXi face;
	int face_count;
	Eigen::MatrixXd face_size;
	Eigen::MatrixXd normal_weight;
	Eigen::MatrixXd face_size_rescale;
	Eigen::MatrixXd faceNormal;
	Eigen::MatrixXd l_mean;
	Eigen::MatrixXd l_var;
	Eigen::MatrixXd c1_mean;
	Eigen::MatrixXd c1_var;
	Eigen::MatrixXd c2_mean;
	Eigen::MatrixXd c2_var;
	Eigen::MatrixXd face_center;
	std::vector<Eigen::MatrixXi> pointInface_cell;
	Eigen::MatrixXi pointInface_num;
};

struct Feature
{
	double ref_vertex_eff;
	double dis_vertex_eff;
	double ref_smoothness;
	double dis_smoothness;
	double ref_cur_value;
	double dis_cur_value;
	double ref_facelm;
	double dis_facelm;
	double ref_facevar;
	double dis_facevar;

	double ref_facec1m;
	double dis_facec1m;
	double ref_facec1var;
	double dis_facec1var;
	double ref_facec2m;
	double dis_facec2m;
	double ref_facec2var;
	double dis_facec2var;
};

struct Mesh2Graph {
	Eigen::MatrixXd W;
	Eigen::MatrixXi W_flage;
	Eigen::MatrixXd D;
	Eigen::MatrixXi D_flage;
	Eigen::MatrixXd W_nor;
	Eigen::MatrixXd dis_w;
};

struct SubFeature {
	int flag;
	Eigen::MatrixXd ref_facelm;
	Eigen::MatrixXd dis_facelm;
	Eigen::MatrixXd ref_facevar;
	Eigen::MatrixXd dis_facevar;
	Eigen::MatrixXd ref_facec1m;
	Eigen::MatrixXd dis_facec1m;
	Eigen::MatrixXd ref_facec1var;
	Eigen::MatrixXd dis_facec1var;
	Eigen::MatrixXd ref_facec2m;
	Eigen::MatrixXd dis_facec2m;
	Eigen::MatrixXd ref_facec2var;
	Eigen::MatrixXd dis_facec2var;

	double ref_cur_value;
	double dis_cur_value;

	Eigen::MatrixXd ref_face_size;
	Eigen::MatrixXd dis_face_size;
	Eigen::MatrixXd ref_face_size_re;
	Eigen::MatrixXd dis_face_size_re;

	Eigen::MatrixXi ref_vertex_eff;
	Eigen::MatrixXi dis_vertex_eff;

	Eigen::MatrixXd ref_cur_vec;
	Eigen::MatrixXd dis_cur_vec;

	Eigen::MatrixXd ref_facenor;
	Eigen::MatrixXd dis_facenor;

	Eigen::MatrixXd ref_GTV;
	Eigen::MatrixXd dis_GTV;

	Eigen::MatrixXd ref_smoothness;
	Eigen::MatrixXd dis_smoothness;
};

struct Point {
	double x, y;
};

struct MatrixXdHash {
	std::size_t operator()(const Eigen::MatrixXd& matrix) const {
		std::size_t seed = 0;
		for (int i = 0; i < matrix.rows(); ++i) {
			for (int j = 0; j < matrix.cols(); ++j) {
				seed ^= std::hash<int>()(matrix(i, j)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
			}
		}
		return seed;
	}
};

struct MatrixXiHash {
	std::size_t operator()(const Eigen::MatrixXi& matrix) const {
		std::size_t seed = 0;
		for (int i = 0; i < matrix.rows(); ++i) {
			for (int j = 0; j < matrix.cols(); ++j) {
				seed ^= std::hash<int>()(matrix(i, j)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
			}
		}
		return seed;
	}
};

void Image3_Init(Image3 &image, int width, int height);
void Image3_read(Image3& image, cv::Mat pic);

void unique(std::vector<int>& unique_ordered, Eigen::MatrixXi input);
void uniqued(std::vector<int>& unique_ordered, Eigen::MatrixXd input);
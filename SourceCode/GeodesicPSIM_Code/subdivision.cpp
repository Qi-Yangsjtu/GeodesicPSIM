#include "headfile.h"


void subdivision(Clean_Mesh &A, Mesh &B, Extra &B_extra, int iter) {
	
	std::cout << "Mesh subdivision: " << std::endl;

	Eigen::MatrixXd A_vertices = A.vertexInFace;
	Eigen::MatrixXi A_faces = A.face;
	Eigen::MatrixXd A_uv = A.extra.textures;
	Eigen::MatrixXi A_faceuv = A.extra.face_texture;


	std::vector<Eigen::Vector3d> B_vertices;
	std::vector < Eigen::Vector3i > B_faces;
	std::vector < Eigen::Vector2d > B_uv;
	std::vector < Eigen::Vector3i > B_faceuv;
	//for (int i = 0; i < 1; i++) {
	for (int time = 0; time < iter; time++) {
		if (time == 0) {
			for (int i = 0; i < A.face_count; i++) {
				Eigen::Vector3i this_face = A_faces.row(i);
				Eigen::Vector3i this_face_uvindex = A_faceuv.row(i);
				Eigen::Vector3d p1 = A_vertices.row(this_face[0]);
				Eigen::Vector3d p2 = A_vertices.row(this_face[1]);
				Eigen::Vector3d p3 = A_vertices.row(this_face[2]);
				Eigen::Vector2d uv1 = A_uv.row(this_face_uvindex[0]);
				Eigen::Vector2d uv2 = A_uv.row(this_face_uvindex[1]);
				Eigen::Vector2d uv3 = A_uv.row(this_face_uvindex[2]);

				//std::cout << p1 << std::endl;
				Eigen::Vector3d mid_p1p2 = (p1 + p2) / 2;
				Eigen::Vector3d mid_p2p3 = (p2 + p3) / 2;
				Eigen::Vector3d mid_p3p1 = (p3 + p1) / 2;

				Eigen::Vector2d mid_uv1uv2 = (uv1 + uv2) / 2;
				Eigen::Vector2d mid_uv2uv3 = (uv2 + uv3) / 2;
				Eigen::Vector2d mid_uv3uv1 = (uv3 + uv1) / 2;

				int index = B_vertices.size();
				//std::cout << index << std::endl;
				int p1_index = index;
				int p2_index = index + 1;
				int p3_index = index + 2;
				int mid_p1p2_index = index + 3;
				int mid_p2p3_index = index + 4;
				int mid_p3p1_index = index + 5;

				B_vertices.push_back(p1);
				B_vertices.push_back(p2);
				B_vertices.push_back(p3);
				B_vertices.push_back(mid_p1p2);
				B_vertices.push_back(mid_p2p3);
				B_vertices.push_back(mid_p3p1);

				B_uv.push_back(uv1);
				B_uv.push_back(uv2);
				B_uv.push_back(uv3);
				B_uv.push_back(mid_uv1uv2);
				B_uv.push_back(mid_uv2uv3);
				B_uv.push_back(mid_uv3uv1);

				Eigen::Vector3i face1, face2, face3, face4;
				face1 << p1_index, mid_p1p2_index, mid_p3p1_index;
				face2 << mid_p1p2_index, p2_index, mid_p2p3_index;
				face3 << mid_p1p2_index, mid_p2p3_index, mid_p3p1_index; //Ë³Ðò
				face4 << mid_p3p1_index, mid_p2p3_index, p3_index;

				B_faces.push_back(face1);
				B_faces.push_back(face2);
				B_faces.push_back(face3);
				B_faces.push_back(face4);

				B_faceuv.push_back(face1);
				B_faceuv.push_back(face2);
				B_faceuv.push_back(face3);
				B_faceuv.push_back(face4);

			}

		}
		else {
			std::vector<Eigen::Vector3d> C_vertices;
			std::vector < Eigen::Vector3i > C_faces;
			std::vector < Eigen::Vector2d > C_uv;
			std::vector < Eigen::Vector3i > C_faceuv;
			C_vertices.assign(B_vertices.begin(), B_vertices.end());
			C_faces.assign(B_faces.begin(), B_faces.end());
			C_uv.assign(B_uv.begin(), B_uv.end());
			C_faceuv.assign(B_faceuv.begin(), B_faceuv.end());
			B_vertices.clear();
			B_faces.clear();
			B_faceuv.clear();
			B_uv.clear();
			for (int i = 0; i < C_faces.size(); i++) {
				Eigen::Vector3i this_face = C_faces[i];
				Eigen::Vector3i this_face_uvindex = C_faceuv[i];
				Eigen::Vector3d p1 = C_vertices[this_face[0]];
				Eigen::Vector3d p2 = C_vertices[this_face[1]];
				Eigen::Vector3d p3 = C_vertices[this_face[2]];
				Eigen::Vector2d uv1 = C_uv[this_face_uvindex[0]];
				Eigen::Vector2d uv2 = C_uv[this_face_uvindex[1]];
				Eigen::Vector2d uv3 = C_uv[this_face_uvindex[2]];

				//std::cout << p1 << std::endl;
				Eigen::Vector3d mid_p1p2 = (p1 + p2) / 2;
				Eigen::Vector3d mid_p2p3 = (p2 + p3) / 2;
				Eigen::Vector3d mid_p3p1 = (p3 + p1) / 2;

				Eigen::Vector2d mid_uv1uv2 = (uv1 + uv2) / 2;
				Eigen::Vector2d mid_uv2uv3 = (uv2 + uv3) / 2;
				Eigen::Vector2d mid_uv3uv1 = (uv3 + uv1) / 2;

				int index = B_vertices.size();
				//std::cout << index << std::endl;
				int p1_index = index;
				int p2_index = index + 1;
				int p3_index = index + 2;
				int mid_p1p2_index = index + 3;
				int mid_p2p3_index = index + 4;
				int mid_p3p1_index = index + 5;

				B_vertices.push_back(p1);
				B_vertices.push_back(p2);
				B_vertices.push_back(p3);
				B_vertices.push_back(mid_p1p2);
				B_vertices.push_back(mid_p2p3);
				B_vertices.push_back(mid_p3p1);

				B_uv.push_back(uv1);
				B_uv.push_back(uv2);
				B_uv.push_back(uv3);
				B_uv.push_back(mid_uv1uv2);
				B_uv.push_back(mid_uv2uv3);
				B_uv.push_back(mid_uv3uv1);

				Eigen::Vector3i face1, face2, face3, face4;
				face1 << p1_index, mid_p1p2_index, mid_p3p1_index;
				face2 << mid_p1p2_index, p2_index, mid_p2p3_index;
				face3 << mid_p1p2_index, mid_p2p3_index, mid_p3p1_index; //Ë³Ðò
				face4 << mid_p3p1_index, mid_p2p3_index, p3_index;

				B_faces.push_back(face1);
				B_faces.push_back(face2);
				B_faces.push_back(face3);
				B_faces.push_back(face4);

				B_faceuv.push_back(face1);
				B_faceuv.push_back(face2);
				B_faceuv.push_back(face3);
				B_faceuv.push_back(face4);
			
			}
		
		}

		B.vertices.conservativeResize(B_vertices.size(), 3);
		B.faces.conservativeResize(B_faces.size(), 3);
		B_extra.textures.conservativeResize(B_uv.size(), 2);
		B_extra.face_texture.conservativeResize(B_faceuv.size(), 3);

		for (int j = 0; j < B_vertices.size(); j++) {
			if (j < B_faces.size()) {
				B.faces.row(j) = B_faces[j];
				B_extra.face_texture.row(j) = B_faceuv[j];
			}
			B.vertices.row(j) = B_vertices[j];
			B_extra.textures.row(j) = B_uv[j];

		}
	}
}


void Targetsubdivision(Clean_Mesh& A, Mesh& B, Extra& B_extra, int TargetNUM) {

	std::cout << "Target Mesh subdivision: " << std::endl;

	Eigen::MatrixXd A_vertices = A.vertexInFace;
	Eigen::MatrixXi A_faces = A.face;
	Eigen::MatrixXd A_uv = A.extra.textures;
	Eigen::MatrixXi A_faceuv = A.extra.face_texture;


	std::vector<Eigen::Vector3d> B_vertices;
	std::vector < Eigen::Vector3i > B_faces;
	std::vector < Eigen::Vector2d > B_uv;
	std::vector < Eigen::Vector3i > B_faceuv;

	int taregt_face_num = TargetNUM;
	int act_face_num = A_faces.rows();
	int num_face_to_sub = 0;
	std::vector<double> face_size;
	std::vector<double> face_size_sort;
	if (taregt_face_num < act_face_num) {
		std::cerr << "Target face number should larger than actual face number!" << std::endl;
		std::abort();
	}
	else {
		num_face_to_sub = ceil((taregt_face_num - act_face_num) / 3.0);

		// calculate face size, subdivide the first num_face_to_sub faces;
		
		for (int i = 0; i < A_faces.rows(); i++) {
			Eigen::Vector3d vec_a = A_vertices.row(A_faces(i, 0)) - A_vertices.row(A_faces(i, 1));
			Eigen::Vector3d vec_b = A_vertices.row(A_faces(i, 2)) - A_vertices.row(A_faces(i, 1));
			Eigen::Vector3d cross_product = vec_a.cross(vec_b);
			face_size.push_back(cross_product.norm());
		}
		face_size_sort = face_size;
		std::sort(face_size_sort.begin(), face_size_sort.end());
		double threshould = face_size_sort[ act_face_num - num_face_to_sub];
		for (int j = 0; j < A.face_count; j++) {
			Eigen::Vector3i this_face = A_faces.row(j);
			Eigen::Vector3i this_face_uvindex = A_faceuv.row(j);
			Eigen::Vector3d p1 = A_vertices.row(this_face[0]);
			Eigen::Vector3d p2 = A_vertices.row(this_face[1]);
			Eigen::Vector3d p3 = A_vertices.row(this_face[2]);
			Eigen::Vector2d uv1 = A_uv.row(this_face_uvindex[0]);
			Eigen::Vector2d uv2 = A_uv.row(this_face_uvindex[1]);
			Eigen::Vector2d uv3 = A_uv.row(this_face_uvindex[2]);
			if (face_size[j] < threshould) {
				int index = B_vertices.size();
				//std::cout << index << std::endl;
				int p1_index = index;
				int p2_index = index + 1;
				int p3_index = index + 2;
				B_vertices.push_back(p1);
				B_vertices.push_back(p2);
				B_vertices.push_back(p3);
				B_uv.push_back(uv1);
				B_uv.push_back(uv2);
				B_uv.push_back(uv3);
				Eigen::Vector3i face1;
				face1 << p1_index, p2_index, p3_index;
				B_faces.push_back(face1);
				B_faceuv.push_back(face1);
			}
			else{
				// subdivision this face
				//std::cout << p1 << std::endl;
				Eigen::Vector3d mid_p1p2 = (p1 + p2) / 2;
				Eigen::Vector3d mid_p2p3 = (p2 + p3) / 2;
				Eigen::Vector3d mid_p3p1 = (p3 + p1) / 2;

				Eigen::Vector2d mid_uv1uv2 = (uv1 + uv2) / 2;
				Eigen::Vector2d mid_uv2uv3 = (uv2 + uv3) / 2;
				Eigen::Vector2d mid_uv3uv1 = (uv3 + uv1) / 2;

				int index = B_vertices.size();
				//std::cout << index << std::endl;
				int p1_index = index;
				int p2_index = index + 1;
				int p3_index = index + 2;
				int mid_p1p2_index = index + 3;
				int mid_p2p3_index = index + 4;
				int mid_p3p1_index = index + 5;

				B_vertices.push_back(p1);
				B_vertices.push_back(p2);
				B_vertices.push_back(p3);
				B_vertices.push_back(mid_p1p2);
				B_vertices.push_back(mid_p2p3);
				B_vertices.push_back(mid_p3p1);

				B_uv.push_back(uv1);
				B_uv.push_back(uv2);
				B_uv.push_back(uv3);
				B_uv.push_back(mid_uv1uv2);
				B_uv.push_back(mid_uv2uv3);
				B_uv.push_back(mid_uv3uv1);

				Eigen::Vector3i face1, face2, face3, face4;
				face1 << p1_index, mid_p1p2_index, mid_p3p1_index;
				face2 << mid_p1p2_index, p2_index, mid_p2p3_index;
				face3 << mid_p1p2_index, mid_p2p3_index, mid_p3p1_index; //Ë³Ðò
				face4 << mid_p3p1_index, mid_p2p3_index, p3_index;

				B_faces.push_back(face1);
				B_faces.push_back(face2);
				B_faces.push_back(face3);
				B_faces.push_back(face4);

				B_faceuv.push_back(face1);
				B_faceuv.push_back(face2);
				B_faceuv.push_back(face3);
				B_faceuv.push_back(face4);
			}
		
		}

		B.vertices.conservativeResize(B_vertices.size(), 3);
		B.faces.conservativeResize(B_faces.size(), 3);
		B_extra.textures.conservativeResize(B_uv.size(), 2);
		B_extra.face_texture.conservativeResize(B_faceuv.size(), 3);

		for (int j = 0; j < B_vertices.size(); j++) {
			if (j < B_faces.size()) {
				B.faces.row(j) = B_faces[j];
				B_extra.face_texture.row(j) = B_faceuv[j];
			}
			B.vertices.row(j) = B_vertices[j];
			B_extra.textures.row(j) = B_uv[j];

		}
	}
}
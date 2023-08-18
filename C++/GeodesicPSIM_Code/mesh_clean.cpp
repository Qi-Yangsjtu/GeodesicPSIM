#include "headfile.h"
#include<numeric>
#include<algorithm>
#include<unordered_set>
#include<unordered_map>


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

bool compare(const std::pair<int, int>& a, const std::pair<int, int>& b) {
	if (a.first != b.first)
		return a.first < b.first;
	return a.second < b.second;
}

std::pair<std::vector<int>, std::vector<int>> removeDuplicates(const std::vector<int>& a) {
	std::unordered_set<int> uniqueElements;
	std::vector<int> b;
	std::vector<int> c;

	for (int i = 0; i < a.size(); ++i) {
		if (uniqueElements.insert(a[i]).second) {
			b.push_back(a[i]);
			c.push_back(i);
		}
	}

	return std::make_pair(b, c);
}

std::vector<int> processIndexes(const std::vector<int>& index, const std::vector<int>& _int) {
	std::unordered_set<int> intSet(_int.begin(), _int.end());
	std::vector<int> _b(index.size(), 0);
	int cnt = 0;

	for (int i = 0; i < index.size(); ++i) {
		if (intSet.find(index[i]) != intSet.end()) {
			_b[i] = cnt;
			cnt += 1;
		}
	}

	return _b;
}


Clean_Mesh mesh_clean(Mesh mesh, Extra extra, Image3 texture) {

	Clean_Mesh ret_mesh;

	ret_mesh.face = mesh.faces;
	ret_mesh.vertex = mesh.vertices;

	extra.textures = (extra.textures.array() > 1).select(extra.textures.array() - 1, extra.textures);
	extra.textures = (extra.textures.array() < 0).select(extra.textures.array() + 1, extra.textures);

	ret_mesh.extra = extra;
	ret_mesh.texturemap = texture;

	Eigen::MatrixXi faces = mesh.faces;
	Eigen::MatrixXi face_texture = ret_mesh.extra.face_texture;

	Eigen::MatrixXd uv = ret_mesh.extra.textures;

	Eigen::MatrixXi face_vertex_index_reshape = faces;
	face_vertex_index_reshape.resize(1, faces.size());

	Eigen::MatrixXi face_vertexuv_index_reshape = face_texture;
	face_vertexuv_index_reshape.resize(1, face_texture.size());

	std::vector<int> face_vertex_index(face_vertex_index_reshape.size());

	std::iota(face_vertex_index.begin(), face_vertex_index.end(), 0);

	std::sort(face_vertex_index.begin(), face_vertex_index.end(), [&](int a, int b) {
		return face_vertex_index_reshape(a) < face_vertex_index_reshape(b);
	});

	Eigen::MatrixXi face_vertex_ordered(1, face_vertex_index_reshape.cols());
	
	for (int i = 0; i < face_vertex_index_reshape.cols(); ++i) {
		face_vertex_ordered(0, i) = face_vertex_index_reshape(face_vertex_index[i]);
	}

	int face_vertex_index_reshape_col = face_vertex_index_reshape.cols();
	Eigen::MatrixXi face_vertexuv_ordered(1, face_vertex_index_reshape_col);


	for (int i = 0; i < face_vertex_index_reshape_col; ++i) {
		face_vertexuv_ordered(0, i) = face_vertexuv_index_reshape(face_vertex_index[i]);
	}

	std::vector<int> unique_face_vertex_ordered;
	std::vector<int> old2new;
	int now_value = face_vertex_ordered(0,0);
	unique_face_vertex_ordered.push_back(now_value);
	old2new.push_back(0);

	for (int i = 0; i < face_vertex_ordered.cols(); ++i) {
		if (face_vertex_ordered(0,i) != now_value) {
			now_value = face_vertex_ordered(0, i);
			unique_face_vertex_ordered.push_back(now_value);
			old2new.push_back(i);
		}
	}
	Eigen::MatrixXi face_vertexuv_ordered2(1, old2new.size());

	for (int i = 0; i < old2new.size(); ++i) {
		face_vertexuv_ordered2(0, i) = face_vertexuv_ordered(old2new[i]);
	}

	Eigen::MatrixXd uv_ordered(face_vertexuv_ordered2.cols(), uv.cols());

	for (int i = 0; i < face_vertexuv_ordered2.cols(); ++i) {
		uv_ordered.row(i) = uv.row(face_vertexuv_ordered2(i));
	}

	Eigen::MatrixXd u2x = (1 - uv_ordered.col(1).array()) * texture.row;
	Eigen::MatrixXd u2y = uv_ordered.col(0).array() * texture.col;
	Eigen::MatrixXi x_coor = normalized(u2x, texture.row);
	Eigen::MatrixXi y_coor = normalized(u2y, texture.col);
	Eigen::MatrixXi r = texture.r;
	Eigen::MatrixXi g = texture.g;
	Eigen::MatrixXi b = texture.b;

	r.transposeInPlace(); g.transposeInPlace(); b.transposeInPlace();

	Eigen::MatrixXi ref_color_index = (x_coor.array() - 1) * texture.col + y_coor.array() - 1;
	r.resize(1, r.rows() * r.cols());
	g.resize(1, g.rows() * g.cols());
	b.resize(1, b.rows() * b.cols());

	Eigen::MatrixXi vertex_r(1, ref_color_index.rows()), vertex_g(1, ref_color_index.rows()), vertex_b(1, ref_color_index.rows());

	for (int i = 0; i < ref_color_index.size(); ++i) {
		vertex_r(0, i) = r(0, ref_color_index(i, 0));
		vertex_g(0, i) = g(0, ref_color_index(i, 0));
		vertex_b(0, i) = b(0, ref_color_index(i, 0));
	}

	Eigen::MatrixXi ref_vertexcolor(vertex_r.cols(), vertex_r.rows() * 3);
	ref_vertexcolor.block(0, 0, vertex_r.cols(), vertex_r.rows()) = vertex_r.transpose();
	ref_vertexcolor.block(0, vertex_r.rows() * 1, vertex_r.cols(), vertex_r.rows()) = vertex_g.transpose();
	ref_vertexcolor.block(0, vertex_r.rows() * 2, vertex_r.cols(), vertex_r.rows()) = vertex_b.transpose();

	Eigen::MatrixXi vertexcolor(mesh.vertices.rows(), mesh.vertices.cols());

	for (int i = 0; i < unique_face_vertex_ordered.size(); ++i) {
		vertexcolor.row(unique_face_vertex_ordered[i]) = ref_vertexcolor.row(i);
	}

	Eigen::MatrixXd vertices = mesh.vertices;

	std::vector<int> old2new2;
	std::vector<int> new2old;
	old2new2.reserve(vertices.rows());
	new2old.reserve(vertices.rows());

	int cnt = 0;
	std::unordered_map<Eigen::Vector3d, int, Vector3dHash, Vector3dEqual> myMap(vertices.rows()); 
	for (int i = 0; i < vertices.rows(); ++i) {
		Eigen::Vector3d row = vertices.row(i);
		auto pair = myMap.emplace(row, cnt);
		if (pair.second) { // new element
			old2new2.push_back(i);
			new2old.push_back(cnt);
			cnt += 1;
		}
		else {
			new2old.push_back(pair.first->second); //element already exist
		}
	}

	old2new2.shrink_to_fit();

	Eigen::MatrixXd vertices_new2(old2new2.size(), 3);

	for (int i = 0; i < old2new2.size(); ++i) {
		vertices_new2.row(i) = vertices.row(old2new2[i]);
	}

	ret_mesh.vertexcolor_rgb.conservativeResize(old2new2.size(), 3);

	for (int i = 0; i < ret_mesh.vertexcolor_rgb.rows(); ++i) {
		ret_mesh.vertexcolor_rgb.row(i) = vertexcolor.row(i);
	}

	if (old2new2.size() < vertices.rows()) {

		Eigen::MatrixXi corr_vertices(vertices.rows(), 2);
		for (int i = 0; i < vertices.rows(); ++i) {
			corr_vertices.row(i) << i, new2old[i];
		}

		for (int i = 0; i < faces.rows(); ++i) {
			faces(i, 0) = corr_vertices(faces(i, 0), 1);
			faces(i, 1) = corr_vertices(faces(i, 1), 1);
			faces(i, 2) = corr_vertices(faces(i, 2), 1);
		}

		std::vector<int> bad_face_index;
		for (int i = 0; i < faces.rows(); ++i) {
			if ( (faces(i, 0) == faces(i, 1)) || (faces(i, 0) == faces(i, 2)) || (faces(i, 1) == faces(i, 2)) ) {
				bad_face_index.push_back(i);
			}
		}

		int num_rows_to_remove = bad_face_index.size();
		Eigen::MatrixXi modified_faces(faces.rows() - num_rows_to_remove, faces.cols());
		Eigen::MatrixXi modified_face_texture(face_texture.rows() - num_rows_to_remove, face_texture.cols());

		int row_offset = 0;

		for (int i = 0; i < faces.rows(); i++) {
			if (std::find(bad_face_index.begin(), bad_face_index.end(), i) == bad_face_index.end()) {
				modified_faces.block(row_offset, 0, 1, faces.cols()) = faces.row(i);
				row_offset++;
			}
		}

		row_offset = 0;

		for (int i = 0; i < face_texture.rows(); i++) {
			if (std::find(bad_face_index.begin(), bad_face_index.end(), i) == bad_face_index.end()) {
				modified_face_texture.block(row_offset, 0, 1, face_texture.cols()) = face_texture.row(i);
				row_offset++;
			}
		}

		std::vector<int> old2new5;
		unique(old2new5, modified_faces);
	
		Eigen::MatrixXi modified_modified_faces(old2new5.size(), 3);
		for (int i = 0; i < modified_modified_faces.rows(); ++i) {
			modified_modified_faces.row(i) = modified_faces.row(old2new5[i]);
		}

		Eigen::MatrixXi modified_modified_face_texture(old2new5.size(), 3);
		for (int i = 0; i < modified_modified_faces.rows(); ++i) {
			modified_modified_face_texture.row(i) = modified_face_texture.row(old2new5[i]);
		}

		ret_mesh.vertex = vertices_new2;
		ret_mesh.face = modified_modified_faces;
		ret_mesh.face_count = modified_modified_faces.rows();
		ret_mesh.vertex_count = old2new2.size();
		ret_mesh.extra.face_texture = modified_modified_face_texture;
	}
	else {
		ret_mesh.face = faces;
		ret_mesh.face_count = ret_mesh.face.rows();
		ret_mesh.vertex_count = ret_mesh.vertex.rows();
	}

	std::vector<std::pair<int, int>> face_vertex_index_reshape_vector;
	for (int i = 0; i < ret_mesh.face.cols(); ++i) {
		for (int j = 0; j < ret_mesh.face.rows(); ++j) {
			face_vertex_index_reshape_vector.push_back(std::make_pair(ret_mesh.face(j, i), i * ret_mesh.face.rows() + j));
		}
	}

	std::sort(face_vertex_index_reshape_vector.begin(), face_vertex_index_reshape_vector.end(), compare);

	Eigen::MatrixXi face_vertexuv_ordered3 = ret_mesh.extra.face_texture;
	face_vertexuv_ordered3.resize(1, ret_mesh.extra.face_texture.rows() * ret_mesh.extra.face_texture.cols());
	
	Eigen::MatrixXi face_vertexuv_ordered4(1, ret_mesh.extra.face_texture.rows() * ret_mesh.extra.face_texture.cols());
	for (int i = 0; i < face_vertex_index_reshape_vector.size(); ++i) {
		face_vertexuv_ordered4(0, i) = face_vertexuv_ordered3(0, face_vertex_index_reshape_vector[i].second);
	}

	std::vector<int> face_vertex_ordered_vector;
	for (int i = 0; i < face_vertex_index_reshape_vector.size(); ++i) {
		face_vertex_ordered_vector.push_back(face_vertex_index_reshape_vector[i].first);
	}

	std::vector< int > old2new3;
	int old_value = face_vertex_ordered_vector[0];
	std::vector< int > _int;

	std::pair<std::vector<int>, std::vector<int>> result = removeDuplicates(face_vertex_ordered_vector);
	_int = result.first;
	old2new3 = result.second;

	Eigen::MatrixXi face_vertexuv_ordered5(1, old2new3.size());
	for (int i = 0; i < old2new3.size(); ++i) {
		face_vertexuv_ordered5(0, i) = face_vertexuv_ordered4(0, old2new3[i]);
	}

	Eigen::MatrixXd uv_ordered2(face_vertexuv_ordered5.cols(), 2);
	for (int i = 0; i < uv_ordered2.rows(); ++i) {
		uv_ordered2.row(i) = uv.row(face_vertexuv_ordered5(0, i));
	}


	ret_mesh.vertexInFace.resize(_int.size(), ret_mesh.vertex.cols());

	for (int i = 0; i < _int.size(); ++i) {
		ret_mesh.vertexInFace.row(i) = ret_mesh.vertex.row(_int[i]);
	}

	Eigen::MatrixXi vertexcolor_rgb_new = ret_mesh.vertexcolor_rgb;
	ret_mesh.vertexcolor_rgb.resize(_int.size(), ret_mesh.vertex.cols());

	for (int i = 0; i < _int.size(); ++i) {
		ret_mesh.vertexcolor_rgb.row(i) = vertexcolor_rgb_new.row(_int[i]);
	}

	ret_mesh.vertexInFace_count = ret_mesh.vertexInFace.rows();

	ret_mesh.extra.textures = uv_ordered2;

	std::vector< int > _b;
	std::vector<int> index_i(ret_mesh.vertex.rows());
	std::iota(index_i.begin(),index_i.end(), 0);
	_b = processIndexes(index_i, _int);

	Eigen::MatrixXi fix_face = ret_mesh.face;
	for (int i = 0; i < fix_face.rows(); ++i) {
		fix_face(i, 0) = _b[fix_face(i, 0)];
		fix_face(i, 1) = _b[fix_face(i, 1)];
		fix_face(i, 2) = _b[fix_face(i, 2)];
	}

	ret_mesh.face = fix_face;

	Eigen::MatrixXd YUV_matrix(3, 3);
	YUV_matrix << 0.299, 0.587, 0.114,
		-0.1678, -0.3313, 0.5,
		0.5, -0.4187, -0.0813;
	YUV_matrix.transposeInPlace();
	Eigen::MatrixXd yuv(ret_mesh.vertexcolor_rgb.rows(), 3);
	yuv = ret_mesh.vertexcolor_rgb.cast<double>() * YUV_matrix;
	for (int i = 0; i < yuv.rows(); ++i) {
		yuv(i, 1) += 128; yuv(i, 2) += 128;
	}
	ret_mesh.vertexcolor = yuv;

	Eigen::MatrixXd u2xx = (1 - ret_mesh.extra.textures.col(1).array()) * texture.row;
	Eigen::MatrixXd v2yy = ret_mesh.extra.textures.col(0).array() * texture.col;

	Eigen::MatrixXi x_coor2 = normalized(u2xx, texture.row);
	Eigen::MatrixXi y_coor2 = normalized(v2yy, texture.col);


	Eigen::MatrixXi x_y_coor2(x_coor2.rows(), x_coor2.cols() + y_coor2.cols());
	x_y_coor2.block(0, 0, x_coor2.rows(), x_coor2.cols()) = x_coor2;
	x_y_coor2.block(0, x_coor2.cols(), y_coor2.rows(), y_coor2.cols()) = y_coor2;

	ret_mesh.texture_coor = x_y_coor2;

	return ret_mesh;
}

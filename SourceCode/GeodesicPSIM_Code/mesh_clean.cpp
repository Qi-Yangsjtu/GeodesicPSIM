#include "headfile.h"


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

void mesh_clean_process_1(Mesh mesh, Clean_Mesh& ret_mesh, Extra extra, Image3 texture) {

	ret_mesh.face = mesh.faces;
	ret_mesh.vertex = mesh.vertices;

	extra.textures = (extra.textures.array() > 1).select(extra.textures.array() - 1, extra.textures);
	extra.textures = (extra.textures.array() < 0).select(extra.textures.array() + 1, extra.textures);

	ret_mesh.extra = extra;
	ret_mesh.texturemap = texture;

	// sort vertex and UV based on face texture

	Eigen::MatrixXi faces = mesh.faces;
	Eigen::MatrixXi face_texture = ret_mesh.extra.face_texture;

	Eigen::MatrixXd uv = ret_mesh.extra.textures;

	// rearrange face

	Eigen::MatrixXi face_vertex_index_reshape = faces;
	face_vertex_index_reshape.resize(1, faces.size());

	// rearrange face texture

	Eigen::MatrixXi face_vertexuv_index_reshape = face_texture;
	face_vertexuv_index_reshape.resize(1, face_texture.size());

	// sort index 

	std::vector<int> face_vertex_index(face_vertex_index_reshape.size());

	std::iota(face_vertex_index.begin(), face_vertex_index.end(), 0);

	std::sort(face_vertex_index.begin(), face_vertex_index.end(), [&](int a, int b) {
		return face_vertex_index_reshape(a) < face_vertex_index_reshape(b);
		});

	Eigen::MatrixXi face_vertex_ordered(1, face_vertex_index_reshape.cols());

	for (int i = 0; i < face_vertex_index_reshape.cols(); ++i) {
		face_vertex_ordered(0, i) = face_vertex_index_reshape(face_vertex_index[i]);
	}

	std::vector<int> face_vertexuv_index(face_vertexuv_index_reshape.size());

	std::iota(face_vertexuv_index.begin(), face_vertexuv_index.end(), 0);

	std::sort(face_vertexuv_index.begin(), face_vertexuv_index.end(), [&](int a, int b) {
		return face_vertexuv_index_reshape(a) < face_vertexuv_index_reshape(b);
		});

	Eigen::MatrixXi face_vertexuv_ordered(1, face_vertexuv_index_reshape.cols());

	for (int i = 0; i < face_vertexuv_index_reshape.cols(); ++i) {
		face_vertexuv_ordered(0, i) = face_vertexuv_index_reshape(face_vertexuv_index[i]);
	}

	

	std::vector<int> int_vertex;
	int now_value = face_vertex_ordered(0, 0);
	int_vertex.push_back(now_value);

	for (int i = 0; i < face_vertex_ordered.cols(); ++i) {
		if (face_vertex_ordered(0, i) != now_value) {
			now_value = face_vertex_ordered(0, i);
			int_vertex.push_back(now_value);
		}
	}

	// remove unreferenced vertex

	Eigen::MatrixXd vertex_ordered(int_vertex.size(), ret_mesh.vertex.cols());
	for (int i = 0; i < vertex_ordered.rows(); ++i) {
		vertex_ordered.row(i) = ret_mesh.vertex.row(int_vertex[i]);
	}

	// updated vertex index in face

	if (vertex_ordered.rows() < ret_mesh.vertex.rows()) {

		std::vector< int > _b;
		std::vector<int> index_i(ret_mesh.vertex.rows());
		std::iota(index_i.begin(), index_i.end(), 0);
		_b = processIndexes(index_i, int_vertex);

		Eigen::MatrixXi fix_face = faces;
		for (int i = 0; i < fix_face.rows(); ++i) {
			faces(i, 0) = _b[fix_face(i, 0)];
			faces(i, 1) = _b[fix_face(i, 1)];
			faces(i, 2) = _b[fix_face(i, 2)];
		}

	}

	// remove unreferenced UV

	std::vector<int> int_uv;
	now_value = face_vertexuv_ordered(0, 0);
	int_uv.push_back(now_value);

	for (int i = 0; i < face_vertexuv_ordered.cols(); ++i) {
		if (face_vertexuv_ordered(0, i) != now_value) {
			now_value = face_vertexuv_ordered(0, i);
			int_uv.push_back(now_value);
		}
	}

	Eigen::MatrixXd uv_ordered(int_uv.size(), uv.cols());
	for (int i = 0; i < uv_ordered.rows(); ++i) {
		uv_ordered.row(i) = uv.row(int_uv[i]);
	}

	if (uv_ordered.rows() < uv.rows()) {

		std::vector< int > _b;
		std::vector<int> index_i(uv.rows());
		std::iota(index_i.begin(), index_i.end(), 0);
		_b = processIndexes(index_i, int_uv);

		Eigen::MatrixXi fix_face_texture = face_texture;
		for (int i = 0; i < fix_face_texture.rows(); ++i) {
			face_texture(i, 0) = _b[fix_face_texture(i, 0)];
			face_texture(i, 1) = _b[fix_face_texture(i, 1)];
			face_texture(i, 2) = _b[fix_face_texture(i, 2)];
		}

	}

	// remove duplicated vertex

	std::vector<int> old2new;
	std::vector<int> new2old_vertex_clean;
	old2new.reserve(vertex_ordered.rows());
	new2old_vertex_clean.reserve(vertex_ordered.rows());

	int cnt = 0;
	std::unordered_map<Eigen::Vector3d, int, Vector3dHash, Vector3dEqual> myMap(vertex_ordered.rows());
	for (int i = 0; i < vertex_ordered.rows(); ++i) {
		Eigen::Vector3d row = vertex_ordered.row(i);
		auto pair = myMap.emplace(row, cnt);
		if (pair.second) { // new element
			old2new.push_back(i);
			new2old_vertex_clean.push_back(cnt);
			cnt += 1;
		}
		else {
			new2old_vertex_clean.push_back(pair.first->second); //element already exist
		}
	}

	old2new.shrink_to_fit();

	Eigen::MatrixXd vertices_cleaned(old2new.size(), 3);

	for (int i = 0; i < old2new.size(); ++i) {
		vertices_cleaned.row(i) = vertex_ordered.row(old2new[i]);
	}

	if (vertices_cleaned.rows() < vertex_ordered.rows()) {

		Eigen::MatrixXi corr_vertices(vertex_ordered.rows(), 2);
		for (int i = 0; i < vertex_ordered.rows(); ++i) {
			corr_vertices.row(i) << i, new2old_vertex_clean[i];
		}

		for (int i = 0; i < faces.rows(); ++i) {
			faces(i, 0) = corr_vertices(faces(i, 0), 1);
			faces(i, 1) = corr_vertices(faces(i, 1), 1);
			faces(i, 2) = corr_vertices(faces(i, 2), 1);
		}
	}

	// remove duplicated UV

	std::vector<int> old2new2;
	std::vector<int> new2old_uv_clean;
	old2new2.reserve(uv_ordered.rows());
	new2old_uv_clean.reserve(uv_ordered.rows());

	cnt = 0;
	std::unordered_map<Eigen::Vector2d, int, Vector2dHash, Vector2dEqual> myMap2(uv_ordered.rows());
	for (int i = 0; i < uv_ordered.rows(); ++i) {
		Eigen::Vector2d row = uv_ordered.row(i);
		auto pair = myMap2.emplace(row, cnt);
		if (pair.second) { // new element
			old2new2.push_back(i);
			new2old_uv_clean.push_back(cnt);
			cnt += 1;
		}
		else {
			new2old_uv_clean.push_back(pair.first->second); //element already exist
		}
	}

	old2new2.shrink_to_fit();

	Eigen::MatrixXd uv_cleaned(old2new2.size(), uv_ordered.cols());

	for (int i = 0; i < old2new2.size(); ++i) {
		uv_cleaned.row(i) = uv_ordered.row(old2new2[i]);
	}

	if (uv_cleaned.rows() < uv_ordered.rows()) {

		Eigen::MatrixXi corr_uv(uv_ordered.rows(), 2);
		for (int i = 0; i < uv_ordered.rows(); ++i) {
			corr_uv.row(i) << i, new2old_uv_clean[i];
		}

		for (int i = 0; i < faces.rows(); ++i) {
			face_texture(i, 0) = corr_uv(face_texture(i, 0), 1);
			face_texture(i, 1) = corr_uv(face_texture(i, 1), 1);
			face_texture(i, 2) = corr_uv(face_texture(i, 2), 1);
		}

	}

	// remove null faces which turn to edge / points: [1 1 2; 1 1 1] --> []
	// remove duplicated faces with diffferent order [1 2 3; 2 3 1] --> [1 2 3]
	// remove faces with three points in a line

	std::vector<int> bad_face_index;
	Eigen::VectorXd v1(3), v2(3), v3(3);
	for (int i = 0; i < faces.rows(); ++i) {
		if ((faces(i, 0) == faces(i, 1)) || (faces(i, 0) == faces(i, 2)) || (faces(i, 1) == faces(i, 2))) {
			bad_face_index.push_back(i);
			continue;
		}
		// calculate face size to remove face size = 0: three points in a line
		v1 = vertices_cleaned.row(faces(i, 0));
		v2 = vertices_cleaned.row(faces(i, 1));
		v3 = vertices_cleaned.row(faces(i, 2));

		Eigen::Vector3d vec_a = v1 - v2;
		Eigen::Vector3d vec_b = v3 - v2;

		Eigen::Vector3d cross_product = vec_a.cross(vec_b);
		float face_size = cross_product.norm();
		if (face_size == 0) {
			bad_face_index.push_back(i);
			continue;
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

	std::vector<int> old2new3;
	unique(old2new3, modified_faces);

	Eigen::MatrixXi modified_modified_faces(old2new3.size(), 3);
	for (int i = 0; i < modified_modified_faces.rows(); ++i) {
		modified_modified_faces.row(i) = modified_faces.row(old2new3[i]);
	}

	Eigen::MatrixXi modified_modified_face_texture(old2new3.size(), 3);
	for (int i = 0; i < modified_modified_faces.rows(); ++i) {
		modified_modified_face_texture.row(i) = modified_face_texture.row(old2new3[i]);
	}

	// updated mesh information

	faces = modified_modified_faces;
	face_texture = modified_modified_face_texture;

	ret_mesh.face = faces;
	ret_mesh.vertex = vertices_cleaned;

	ret_mesh.extra.textures = uv_cleaned;
	ret_mesh.extra.face_texture = face_texture;
}

void mesh_clean_process_2(Clean_Mesh& ret_mesh) {

	// check unreferenced vertices again after remove bad face

	Eigen::MatrixXi faces = ret_mesh.face;

	Eigen::MatrixXd vertex = ret_mesh.vertex;

	Eigen::MatrixXi face_texture = ret_mesh.extra.face_texture;

	Eigen::MatrixXd uv = ret_mesh.extra.textures;

	Eigen::MatrixXi face_vertex_index_reshape = faces;
	face_vertex_index_reshape.resize(1, faces.size());

	Eigen::MatrixXi face_vertexuv_index_reshape = face_texture;
	face_vertexuv_index_reshape.resize(1, face_texture.size());

	// sort index

	std::vector<int> face_vertex_index(face_vertex_index_reshape.size());

	std::iota(face_vertex_index.begin(), face_vertex_index.end(), 0);

	std::sort(face_vertex_index.begin(), face_vertex_index.end(), [&](int a, int b) {
		return face_vertex_index_reshape(a) < face_vertex_index_reshape(b);
		});

	Eigen::MatrixXi face_vertex_ordered(1, face_vertex_index_reshape.cols());

	for (int i = 0; i < face_vertex_index_reshape.cols(); ++i) {
		face_vertex_ordered(0, i) = face_vertex_index_reshape(face_vertex_index[i]);
	}

	std::vector<int> face_vertexuv_index(face_vertexuv_index_reshape.size());

	std::iota(face_vertexuv_index.begin(), face_vertexuv_index.end(), 0);

	std::sort(face_vertexuv_index.begin(), face_vertexuv_index.end(), [&](int a, int b) {
		return face_vertexuv_index_reshape(a) < face_vertexuv_index_reshape(b);
		});

	Eigen::MatrixXi face_vertexuv_ordered(1, face_vertexuv_index_reshape.cols());

	for (int i = 0; i < face_vertexuv_index_reshape.cols(); ++i) {
		face_vertexuv_ordered(0, i) = face_vertexuv_index_reshape(face_vertexuv_index[i]);
	}

	std::vector<int> int_vertex;
	int now_value = face_vertex_ordered(0, 0);
	int_vertex.push_back(now_value);

	for (int i = 0; i < face_vertex_ordered.cols(); ++i) {
		if (face_vertex_ordered(0, i) != now_value) {
			now_value = face_vertex_ordered(0, i);
			int_vertex.push_back(now_value);
		}
	}

	// remove unreferenced vertex

	Eigen::MatrixXd vertex_ordered(int_vertex.size(), vertex.cols());
	for (int i = 0; i < vertex_ordered.rows(); ++i) {
		vertex_ordered.row(i) = vertex.row(int_vertex[i]);
	}

	// updated face index

	if (vertex_ordered.rows() < vertex.rows()) {

		std::vector< int > _b;
		std::vector<int> index_i(vertex.rows());
		std::iota(index_i.begin(), index_i.end(), 0);
		_b = processIndexes(index_i, int_vertex);

		Eigen::MatrixXi fix_face = faces;
		for (int i = 0; i < fix_face.rows(); ++i) {
			faces(i, 0) = _b[fix_face(i, 0)];
			faces(i, 1) = _b[fix_face(i, 1)];
			faces(i, 2) = _b[fix_face(i, 2)];
		}

	}

	// remove unreferenced UV

	std::vector<int> int_uv;
	now_value = face_vertexuv_ordered(0, 0);
	int_uv.push_back(now_value);

	for (int i = 0; i < face_vertexuv_ordered.cols(); ++i) {
		if (face_vertexuv_ordered(0, i) != now_value) {
			now_value = face_vertexuv_ordered(0, i);
			int_uv.push_back(now_value);
		}
	}

	Eigen::MatrixXd uv_ordered(int_uv.size(), uv.cols());
	for (int i = 0; i < uv_ordered.rows(); ++i) {
		uv_ordered.row(i) = uv.row(int_uv[i]);
	}

	if (uv_ordered.rows() < uv.rows()) {

		std::vector< int > _b;
		std::vector<int> index_i(uv.rows());
		std::iota(index_i.begin(), index_i.end(), 0);
		_b = processIndexes(index_i, int_uv);

		Eigen::MatrixXi fix_face_texture = face_texture;
		for (int i = 0; i < fix_face_texture.rows(); ++i) {
			face_texture(i, 0) = _b[fix_face_texture(i, 0)];
			face_texture(i, 1) = _b[fix_face_texture(i, 1)];
			face_texture(i, 2) = _b[fix_face_texture(i, 2)];
		}

	}

	// clean finished

	ret_mesh.face = faces;
	ret_mesh.vertex = vertex_ordered;
	ret_mesh.vertexInFace = vertex_ordered;
	ret_mesh.extra.face_texture = face_texture;
	ret_mesh.extra.textures = uv_ordered;
	ret_mesh.vertexInFace_count = vertex_ordered.rows();
	ret_mesh.vertex_count = vertex_ordered.rows();
	ret_mesh.face_count = faces.rows();
}

void mesh_clean_process_3(Clean_Mesh& ret_mesh, Image3 texture) {

	// calculate vertex color

	Eigen::MatrixXi faces = ret_mesh.face;
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
	int now_value = face_vertex_ordered(0, 0);
	unique_face_vertex_ordered.push_back(now_value);
	old2new.push_back(0);

	for (int i = 0; i < face_vertex_ordered.cols(); ++i) {
		if (face_vertex_ordered(0, i) != now_value) {
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

	// calculate index of vertex in texture map: from UV to XY

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

	// RGB to YUV color space

	Eigen::MatrixXd YUV_matrix(3, 3);
	YUV_matrix << 0.299, 0.587, 0.114,
		-0.1678, -0.3313, 0.5,
		0.5, -0.4187, -0.0813;
	YUV_matrix.transposeInPlace();
	Eigen::MatrixXd yuv(ret_mesh.vertexcolor_rgb.rows(), 3);
	yuv = ref_vertexcolor.cast<double>() * YUV_matrix;
	for (int i = 0; i < yuv.rows(); ++i) {
		yuv(i, 1) += 128; yuv(i, 2) += 128;
	}

	Eigen::MatrixXd u2xx = (1 - uv_ordered.col(1).array()) * texture.row;
	Eigen::MatrixXd v2yy = uv_ordered.col(0).array() * texture.col;

	Eigen::MatrixXi x_coor2 = normalized(u2xx, texture.row);
	Eigen::MatrixXi y_coor2 = normalized(v2yy, texture.col);

	Eigen::MatrixXi x_y_coor2(x_coor2.rows(), x_coor2.cols() + y_coor2.cols());
	x_y_coor2.block(0, 0, x_coor2.rows(), x_coor2.cols()) = x_coor2;
	x_y_coor2.block(0, x_coor2.cols(), y_coor2.rows(), y_coor2.cols()) = y_coor2;

	ret_mesh.vertexcolor_rgb = ref_vertexcolor;
	ret_mesh.vertexcolor = yuv;
	ret_mesh.texture_coor = x_y_coor2;
	ret_mesh.face_count = faces.rows();
	ret_mesh.vertex_count = ret_mesh.vertex.rows();
	ret_mesh.vertexInFace_count = ret_mesh.vertex.rows();

}

Clean_Mesh mesh_clean(Mesh mesh, Extra extra, Image3 texture) {

	Clean_Mesh ret_mesh, mesh_noflat;
	mesh_clean_process_1(mesh, ret_mesh, extra, texture);
	mesh_clean_process_2(ret_mesh);
	mesh_clean_process_3(ret_mesh, texture);
	return ret_mesh;

}

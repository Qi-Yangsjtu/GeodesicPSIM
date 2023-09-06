function mesh_sample = mesh_clean(ref_face, ref_vertex, ref_extra, ref_texture)

ref_mesh.face = ref_face;
ref_mesh.vertex = ref_vertex;
ref_extra.texture(ref_extra.texture>1) = ref_extra.texture(ref_extra.texture>1)-1;
ref_extra.texture(ref_extra.texture<0) = ref_extra.texture(ref_extra.texture<0)+1;
ref_mesh.extra = ref_extra;
ref_mesh.texturemap = ref_texture;

%% rearray the texture coordinates
faces = ref_mesh.face;
face_texture = ref_mesh.extra.face_texture;
uv = ref_mesh.extra.texture;
face_vertex_index_reshape = reshape(faces, 1,  size(faces,1)*size(faces,2));
face_vertexuv_index_reshape = reshape(face_texture,1, size(face_texture,1)*size(face_texture,2));


[face_vertex_ordered, face_vertex_index] = sort(face_vertex_index_reshape);
face_vertexuv_ordered = face_vertexuv_index_reshape(face_vertex_index);
[int, old2new,~] = unique(face_vertex_ordered);
face_vertexuv_ordered = face_vertexuv_ordered(old2new);


uv_ordered = uv(face_vertexuv_ordered,:);

%% calculate index of vertex in texture map: from UV to XY

ref_u2x = (1-uv_ordered(:,2))*size(ref_texture,1);
ref_v2y = (uv_ordered(:,1))*size(ref_texture,2);

ref_x_coor = normalized(ref_u2x,size(ref_texture,1));
ref_y_coor = normalized(ref_v2y,size(ref_texture,2));

ref_r = ref_texture(:,:,1);
ref_g = ref_texture(:,:,2);
ref_b = ref_texture(:,:,3);
ref_texturemap_resolution = size(ref_r);
ref_r = reshape(ref_r',1,ref_texturemap_resolution(1,1)*ref_texturemap_resolution(1,2));
ref_g = reshape(ref_g',1,ref_texturemap_resolution(1,1)*ref_texturemap_resolution(1,2));
ref_b = reshape(ref_b',1,ref_texturemap_resolution(1,1)*ref_texturemap_resolution(1,2));
ref_color_index = (ref_x_coor-1)*ref_texturemap_resolution(1,2)+ref_y_coor;
ref_vertex_r = ref_r(1,ref_color_index);
ref_vertex_g = ref_g(1,ref_color_index);
ref_vertex_b = ref_b(1,ref_color_index);
ref_vertexcolor = [ref_vertex_r',ref_vertex_g',ref_vertex_b'];

vertexcolor = zeros(size(ref_vertex,1),3);
vertexcolor(int,:) = ref_vertexcolor;


%% mesh cleaning
vertices = ref_mesh.vertex;
[vertices_new, index, new2old] = unique(vertices, 'stable', 'rows');

if size(vertices_new,1) < size(vertices,1)
%% removing the duplicated vertices
    ref_mesh.vertexcolor_rgb = uint8(vertexcolor);
%% update face index

    corr_vertices = [1:size(vertices, 1); 1:size(vertices, 1)]';
    corr_vertices(:,2) = new2old(corr_vertices(:,1));
    
    faces(:,1) = corr_vertices(faces(:,1),2);
    faces(:,2) = corr_vertices(faces(:,2),2);
    faces(:,3) = corr_vertices(faces(:,3),2);

    %% remove null faces which turn to edge / points: [1 1 2; 1 1 1] --> []
    bad_face_index = find(faces(:,1)==faces(:,2) | faces(:,1)==faces(:,3) | faces(:,2)==faces(:,3));
    faces(bad_face_index, :) = [];
    face_texture(bad_face_index, :) = [];
    
    %% remove duplicated faces with diffferent order [1 2 3; 2 3 1] --> [1 2 3]
    [~, old2new, ~] = unique(sort(faces, 2), 'stable', 'rows');
    faces = faces(old2new,:);
    face_texture = face_texture(old2new,:);
    
    
    ref_mesh.vertex = vertices_new;
    ref_mesh.face = faces;
    ref_mesh.face_count = size(faces,1);
    ref_mesh.vertex_count = size(vertices_new,1);
    ref_mesh.extra.face_texture = face_texture;
else
    ref_mesh.face_count = size(ref_mesh.face,1);
    ref_mesh.vertex_count = size(ref_mesh.vertex,1);
    ref_mesh.vertexcolor_rgb = vertexcolor;
end


    %% update texture coordinates according to face_texture
    face_vertex_index_reshape = reshape(faces, 1,  size(faces,1)*size(faces,2));
    face_vertexuv_index_reshape = reshape(face_texture,1, size(face_texture,1)*size(face_texture,2));

    [face_vertex_ordered, face_vertex_index] = sort(face_vertex_index_reshape);
    face_vertexuv_ordered = face_vertexuv_index_reshape(face_vertex_index);
    [int, old2new,~] = unique(face_vertex_ordered);
    face_vertexuv_ordered = face_vertexuv_ordered(old2new);
    uv_ordered = uv(face_vertexuv_ordered,:);
    %% remove unreferenced vertices
    ref_mesh.vertexInFace = ref_mesh.vertex(int,:);
    ref_mesh.vertexcolor_rgb = ref_mesh.vertexcolor_rgb(int,:);
    ref_mesh.vertexInFace_count = size(ref_mesh.vertexInFace,1);
    ref_mesh.extra.texture = uv_ordered;

    
    face = ref_mesh.face;
    index = 1:1:size(ref_mesh.vertex,1);
    index = index';
    [a,b] = ismember(index, int','rows');
    new_face(:,1) = b(face(:,1),:);
    new_face(:,2) = b(face(:,2),:);
    new_face(:,3) = b(face(:,3),:);
    ref_mesh.face = new_face;


YUV_matrix = [0.299, 0.587,0.114;-0.1678,-0.3313,0.5;0.5,-0.4187,-0.0813]';
%% RGB to YUV color space
yuv = single(ref_mesh.vertexcolor_rgb)*YUV_matrix;
yuv(:,2) = yuv(:,2)+128;
yuv(:,3) = yuv(:,3)+128;
ref_mesh.vertexcolor = yuv;

ref_u2xx = (1-ref_mesh.extra.texture(:,2))*size(ref_texture,1);
ref_v2yy = (ref_mesh.extra.texture(:,1))*size(ref_texture,2);

ref_x_coor2 = normalized(ref_u2xx,size(ref_texture,1));
ref_y_coor2 = normalized(ref_v2yy,size(ref_texture,1));
ref_mesh.texture_coor = [ref_x_coor2, ref_y_coor2];

mesh_sample = ref_mesh;
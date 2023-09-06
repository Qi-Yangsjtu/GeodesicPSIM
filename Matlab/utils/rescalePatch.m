function dis_patch_mesh = rescalePatch(dis_patch_mesh, scale, dis_texture)

%% update the vertex coordinates
old_dis_vertex = dis_patch_mesh.vertexInFace;
vec_from_neighbor_to_center = old_dis_vertex - old_dis_vertex(dis_patch_mesh.patch_center_index,:);

new_dis_vertex_shift = vec_from_neighbor_to_center./scale;

new_dis_vertex = new_dis_vertex_shift + old_dis_vertex(dis_patch_mesh.patch_center_index,:);

%% update the uv coordinates

old_dis_uv = dis_patch_mesh.texture_coor;

uv_from_neighbor_to_center = old_dis_uv - old_dis_uv(dis_patch_mesh.patch_center_index,:);
new_dis_uv_shift = uv_from_neighbor_to_center./scale;

new_dis_uv = floor(new_dis_uv_shift) + old_dis_uv(dis_patch_mesh.patch_center_index,:);

dis_patch_mesh.vertexInFace = new_dis_vertex;
dis_patch_mesh.texture_coor = new_dis_uv;

%% update the vertex color based on the new uv
dis_r = dis_texture(:,:,1);
dis_g = dis_texture(:,:,2);
dis_b = dis_texture(:,:,3);
dis_texturemap_resolution = size(dis_r);
dis_r = reshape(dis_r',1,dis_texturemap_resolution(1,1)*dis_texturemap_resolution(1,2));
dis_g = reshape(dis_g',1,dis_texturemap_resolution(1,1)*dis_texturemap_resolution(1,2));
dis_b = reshape(dis_b',1,dis_texturemap_resolution(1,1)*dis_texturemap_resolution(1,2));
dis_x_coor = new_dis_uv(:,1);
dis_y_coor = new_dis_uv(:,2);
dis_color_index = (dis_x_coor-1)*dis_texturemap_resolution(1,2)+dis_y_coor;
dis_vertex_r = dis_r(1,dis_color_index);
dis_vertex_g = dis_g(1,dis_color_index);
dis_vertex_b = dis_b(1,dis_color_index);
dis_vertexcolor = [dis_vertex_r',dis_vertex_g',dis_vertex_b'];
YUV_matrix = [0.299, 0.587,0.114;-0.1678,-0.3313,0.5;0.5,-0.4187,-0.0813]';
yuv = single(dis_vertexcolor)*YUV_matrix;
yuv(:,2) = yuv(:,2)+128;
yuv(:,3) = yuv(:,3)+128;
dis_patch_mesh.vertexcolor = yuv;

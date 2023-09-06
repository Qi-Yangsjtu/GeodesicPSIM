function ref_patch_mesh_updated = face_texutre_cal(ref_patch_mesh, texturemap)
face  = ref_patch_mesh.face;

face_re = reshape(face, 1, size(face,1)*size(face,2)); 
face_x_coor = ref_patch_mesh.texture_coor(face_re',1);
face_y_coor = ref_patch_mesh.texture_coor(face_re',2);
face_x_coor_re = reshape(face_x_coor,size(face,1),size(face,2));
face_y_coor_re = reshape(face_y_coor,size(face,1),size(face,2));
YUV_matrix = [0.299, 0.587,0.114;-0.1678,-0.3313,0.5;0.5,-0.4187,-0.0813]';

% select the square bounding box of triangle faces
per_face_x_max = max(face_x_coor_re,[],2);
per_face_y_max = max(face_y_coor_re,[],2);

per_face_x_min = min(face_x_coor_re,[],2);
per_face_y_min = min(face_y_coor_re,[],2);

l_mean = zeros(size(face,1),1);
l_var = zeros(size(face,1),1);

c1_mean = zeros(size(face,1),1);
c1_var = zeros(size(face,1),1);

c2_mean = zeros(size(face,1),1);
c2_var = zeros(size(face,1),1);

map_r = texturemap(:,:,1);
map_g = texturemap(:,:,2);
map_b = texturemap(:,:,3);
texturemap_resolution = size(map_r);
allpixel = texturemap_resolution(1,1)*texturemap_resolution(1,2);

map_r = reshape(map_r',1,texturemap_resolution(1,1)*texturemap_resolution(1,2));
map_g = reshape(map_g',1,texturemap_resolution(1,1)*texturemap_resolution(1,2));
map_b = reshape(map_b',1,texturemap_resolution(1,1)*texturemap_resolution(1,2));
pointInface_cell = cell(size(face,1),1);
pointInface_num = zeros(size(face,1),1);
for i = 1:size(face,1)
    coarse_area_x = per_face_x_min(i,1):1:per_face_x_max(i,1);
    coarse_area_x = coarse_area_x';
    coarse_area_y = per_face_y_min(i,1):1:per_face_y_max(i,1);
    coarse_area_y = coarse_area_y';
    [m,n] = meshgrid(coarse_area_x, coarse_area_y');
    [a, b] = deal(reshape(m,[],1), reshape(n,[],1));

    x_vertex = face_x_coor_re(i,:);
    y_vertex = face_y_coor_re(i,:);

   [in, on] = inpolygon(a, b, x_vertex, y_vertex);
   index = logical(in+on);
    res = [a,b];
    pointInface = res(index,:);  % obtain the x-y index of pixel in this face
    pointInface_cell{i,1} = pointInface;
    pointInface_num(i,1) = size(pointInface,1);
    color_index = (pointInface(:,1)-1)*texturemap_resolution(1,2)+pointInface(:,2);
    color_index(color_index>allpixel) = allpixel;
    color_index(color_index<1) = 1;
    vertex_r = map_r(1,color_index);
    vertex_g = map_g(1,color_index);
    vertex_b = map_b(1,color_index);
    color_map = [vertex_r',vertex_g',vertex_b'];
    yuv_map_tri = single(color_map)*YUV_matrix;
    yuv_map_tri(:,2) = yuv_map_tri(:,2)+128;
    yuv_map_tri(:,3) = yuv_map_tri(:,3)+128;
    
    l_mean(i,1) = mean(yuv_map_tri(:,1),1);
    l_var(i,1) = var(yuv_map_tri(:,1));    
    c1_mean(i,1) = mean(yuv_map_tri(:,2),1);
    c1_var(i,1) = var(yuv_map_tri(:,2)); 
    c2_mean(i,1) = mean(yuv_map_tri(:,3),1);
    c2_var(i,1) = var(yuv_map_tri(:,3)); 
end
%% calculate tirangle norm
ref_patch_mesh.faceNormal = COMPUTE_mesh_normals(ref_patch_mesh);
ref_patch_mesh_updated = ref_patch_mesh;
ref_patch_mesh_updated.l_mean = l_mean;
ref_patch_mesh_updated.l_var = l_var;

ref_patch_mesh_updated.c1_mean = c1_mean;
ref_patch_mesh_updated.c1_var = c1_var;

ref_patch_mesh_updated.c2_mean = c2_mean;
ref_patch_mesh_updated.c2_var = c2_var;

ref_patch_mesh_updated.pointInface_num = pointInface_num;
function result = Feature_extraction(N, ref_face, ref_vertex, ref_extra,ref_texture,dis_face, dis_vertex, dis_extra, dis_texture)
%% Mesh cleaning
disp('Cleaning reference mesh...')
ref_mesh = mesh_clean(ref_face, ref_vertex, ref_extra,ref_texture);


disp('Cleaning distorted mesh...')
dis_mesh = mesh_clean(dis_face, dis_vertex, dis_extra, dis_texture);

%% keypoint selection:

rng(70);
vertex_num = dis_mesh.vertexInFace_count;
if vertex_num < N
    N = vertex_num;
end

%% random sampling
% key_index = randperm(vertex_num, N);
% keypoint = dis_mesh.vertexInFace(key_index,:);

%% FPS sampling
disp('FPS sampling...')
candidate_keypoint = dis_mesh.vertexInFace;
index_keypoint = fps_euclidean(candidate_keypoint,N,1);
keypoint = candidate_keypoint(index_keypoint,:);


feature= cell(size(keypoint,1),1);
keynum = size(keypoint,1);

%% Cropping threshold 
x_limit = max(ref_mesh.vertexInFace(:,1)) - min(ref_mesh.vertexInFace(:,1));
y_limit = max(ref_mesh.vertexInFace(:,2)) - min(ref_mesh.vertexInFace(:,2));
z_limit = max(ref_mesh.vertexInFace(:,3)) - min(ref_mesh.vertexInFace(:,3));
max_bbox = max(max(x_limit,y_limit),z_limit);
PatchRedius = 0.0005*max_bbox;
threshold = PatchRedius;
disp('Extracting features...')
for index_keypoint = 1:1:keynum

    %% 1-hop geodesic path construction
    ref_patch_mesh = creat_local_patch(ref_mesh, keypoint(index_keypoint,:));
    dis_patch_mesh = creat_local_patch(dis_mesh, keypoint(index_keypoint,:));
    patch_num_ref = size(ref_patch_mesh.vertexInFace,1);
    patch_num_dis = size(dis_patch_mesh.vertexInFace,1);
    patch_num_flag_ref = patch_num_ref>3;
    patch_num_flag_dis = patch_num_dis>3;

    if patch_num_flag_dis&&patch_num_flag_ref
                            
                ref_Mesh_graph_ini = Mesh_to_graph(ref_patch_mesh);
                dis_Mesh_graph_ini = Mesh_to_graph(dis_patch_mesh);
                %% Patch cropping
                [ref_patch_mesh, dis_patch_mesh] = patch_cropping(ref_patch_mesh, dis_patch_mesh, ref_Mesh_graph_ini, dis_Mesh_graph_ini, ref_texture, dis_texture, threshold);

                dis_Mesh_graph = Mesh_to_graph(dis_patch_mesh);
                ref_Mesh_graph = Mesh_to_graph(ref_patch_mesh);
                %% Patch pixel color mean and variance
                ref_patch_mesh = face_texutre_cal(ref_patch_mesh,ref_mesh.texturemap);
                dis_patch_mesh = face_texutre_cal(dis_patch_mesh,dis_mesh.texturemap);

                ref_face_normal = ref_patch_mesh.faceNormal;
                dis_face_normal = dis_patch_mesh.faceNormal;
                ref_keypoint_normal = sum(ref_patch_mesh.normal_weight.*ref_face_normal,1);
                dis_keypoint_normal = sum(dis_patch_mesh.normal_weight.*dis_face_normal,1); 


                %% Patch color smoothness calculation
                ref_graph_signal_c1 = double(ref_patch_mesh.vertexcolor(:,1));
                ref_graph_signal_c2 = double(ref_patch_mesh.vertexcolor(:,2));
                ref_graph_signal_c3 = double(ref_patch_mesh.vertexcolor(:,3));        
                ref_Laplacian = ref_Mesh_graph.D - ref_Mesh_graph.W;
                ref_Laplacian_nor = ref_Mesh_graph.D^(-1/2)*ref_Laplacian*ref_Mesh_graph.D^(-1/2); 
                ref_smoothness1_nor = ref_graph_signal_c1'*ref_Laplacian_nor*(ref_graph_signal_c1);
                ref_smoothness2_nor = ref_graph_signal_c2'*ref_Laplacian_nor*(ref_graph_signal_c2);
                ref_smoothness3_nor = ref_graph_signal_c3'*ref_Laplacian_nor*(ref_graph_signal_c3);
                ref_smoothness = [ref_smoothness1_nor,ref_smoothness2_nor, ref_smoothness3_nor];

                dis_graph_signal_c1 = double(dis_patch_mesh.vertexcolor(:,1));   
                dis_graph_signal_c2 = double(dis_patch_mesh.vertexcolor(:,2));
                dis_graph_signal_c3 = double(dis_patch_mesh.vertexcolor(:,3));
                dis_Laplacian = dis_Mesh_graph.D - dis_Mesh_graph.W;
                dis_Laplacian_nor = dis_Mesh_graph.D^(-1/2)*dis_Laplacian*dis_Mesh_graph.D^(-1/2);
                dis_smoothness1_nor = dis_graph_signal_c1'*dis_Laplacian_nor*(dis_graph_signal_c1);
                dis_smoothness2_nor = dis_graph_signal_c2'*dis_Laplacian_nor*(dis_graph_signal_c2);
                dis_smoothness3_nor = dis_graph_signal_c3'*dis_Laplacian_nor*(dis_graph_signal_c3);
                dis_smoothness = [dis_smoothness1_nor,dis_smoothness2_nor, dis_smoothness3_nor];

                

                %% Patch discrete mean curvature
                ref_center = find(ref_patch_mesh.patch_center_index==1);
                dis_center = find(dis_patch_mesh.patch_center_index==1);
                ref_cot_l = full(cotmatrix(ref_patch_mesh.vertexInFace, ref_patch_mesh.face));
                dis_cot_l = full(cotmatrix(dis_patch_mesh.vertexInFace, dis_patch_mesh.face));
                ref_massmatrix = mass_cal(ref_patch_mesh);
                dis_massmatrix = mass_cal(dis_patch_mesh);

            
                ref_cur = 1/(2*ref_massmatrix(ref_center, ref_center))*(ref_cot_l(ref_center,:)*(ref_patch_mesh.vertexInFace(ref_center,:)-ref_patch_mesh.vertexInFace));
                dis_cur = 1/(2*dis_massmatrix(dis_center, dis_center))*(dis_cot_l(dis_center,:)*(dis_patch_mesh.vertexInFace(dis_center,:)-dis_patch_mesh.vertexInFace));
                
                ref_cur_value = ref_cur*ref_keypoint_normal';
                dis_cur_value = dis_cur*dis_keypoint_normal';


                %% Feature saving
                % Features for patch pixel color mean and variance
                subfeature.ref_facelm = (ref_patch_mesh.l_mean);
                subfeature.dis_facelm = (dis_patch_mesh.l_mean);
                subfeature.ref_facevar = (ref_patch_mesh.l_var);
                subfeature.dis_facevar = (dis_patch_mesh.l_var);
                subfeature.ref_facec1m = (ref_patch_mesh.c1_mean);
                subfeature.dis_facec1m = (dis_patch_mesh.c1_mean);
                subfeature.ref_facec1var = (ref_patch_mesh.c1_var);
                subfeature.dis_facec1var = (dis_patch_mesh.c1_var);
                subfeature.ref_facec2m = (ref_patch_mesh.c2_mean);
                subfeature.dis_facec2m = (dis_patch_mesh.c2_mean);
                subfeature.ref_facec2var = (ref_patch_mesh.c2_var);
                subfeature.dis_facec2var = (dis_patch_mesh.c2_var);
                subfeature.ref_vertex_eff = ref_patch_mesh.pointInface_num;
                subfeature.dis_vertex_eff = dis_patch_mesh.pointInface_num;

                % Features for patch discrete mean curvature
                subfeature.ref_cur_value = ref_cur_value;
                subfeature.dis_cur_value = dis_cur_value;

                % Features for patch color smoothness
                subfeature.ref_smoothness = ref_smoothness;
                subfeature.dis_smoothness = dis_smoothness;

                feature{index_keypoint,1} = subfeature;

        else
            subfeature = [];
            feature{index_keypoint,1} = subfeature;
    end
end

result = feature;







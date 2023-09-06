function [ref_patch_mesh_updated, dis_patch_mesh_updated] = patch_cropping(ref_patch_mesh, dis_patch_mesh, ref_Mesh_graph, dis_Mesh_graph, ref_texture, dis_texture, threshold)


%% Face normal
ref_vec_a = ref_patch_mesh.vertexInFace(ref_patch_mesh.face(:,1),:) - ref_patch_mesh.vertexInFace(ref_patch_mesh.face(:,2),:);
ref_vec_b = ref_patch_mesh.vertexInFace(ref_patch_mesh.face(:,3),:) - ref_patch_mesh.vertexInFace(ref_patch_mesh.face(:,2),:);
ref_tri_area = vecnorm(cross(ref_vec_a, ref_vec_b),2,2);

                
dis_vec_a = dis_patch_mesh.vertexInFace(dis_patch_mesh.face(:,1),:) - dis_patch_mesh.vertexInFace(dis_patch_mesh.face(:,2),:);
dis_vec_b = dis_patch_mesh.vertexInFace(dis_patch_mesh.face(:,3),:) - dis_patch_mesh.vertexInFace(dis_patch_mesh.face(:,2),:);
dis_tri_area = vecnorm(cross(dis_vec_a, dis_vec_b),2,2);

ref_normal_weight = ref_tri_area./(sum(ref_tri_area));
dis_normal_weight = dis_tri_area./(sum(dis_tri_area));

ref_patch_mesh.face_size = ref_tri_area;
dis_patch_mesh.face_size = dis_tri_area;
ref_patch_mesh.normal_weight = ref_normal_weight;
dis_patch_mesh.normal_weight = dis_normal_weight;

%% patch cropping step 1

ref_center_to_neighbor_distance = ref_Mesh_graph.dis_w(ref_patch_mesh.patch_center_index,:);
dis_center_to_neighbor_distance = dis_Mesh_graph.dis_w(dis_patch_mesh.patch_center_index,:);

ref_ave_distance = mean(ref_center_to_neighbor_distance);
dis_ave_distance = mean(dis_center_to_neighbor_distance);

scale = dis_ave_distance/ref_ave_distance;

if scale >1
    dis_patch_mesh = rescalePatch(dis_patch_mesh, scale, dis_texture);
end



%% patch cropping step 2
if ref_ave_distance>threshold
    scale1 = ref_ave_distance / threshold;
    ref_patch_mesh = rescalePatch(ref_patch_mesh, scale1, ref_texture);
    dis_patch_mesh = rescalePatch(dis_patch_mesh, scale1, dis_texture);
end
    

ref_patch_mesh_updated = ref_patch_mesh;
dis_patch_mesh_updated = dis_patch_mesh;


                
function ref_patch_mesh = creat_local_patch(ref_mesh, keypoint1)
k=1;
[coarse_index_ref, ~] = knnsearch(ref_mesh.vertexInFace, keypoint1, "K",k,"Distance","euclidean");



% search 1-hop neighbors 
index_keypoint1 = coarse_index_ref;
patch_center = ref_mesh.vertexInFace(index_keypoint1,:); 
faces = ref_mesh.face;

face_selected = faces(:,1) == index_keypoint1 | faces(:,2) == index_keypoint1 | faces(:,3) ==index_keypoint1;
face_selected = faces(face_selected,:);
vertex_index = unique(face_selected(:));


ref_patch_mesh.vertexInFace = ref_mesh.vertexInFace(vertex_index,:);
ref_patch_mesh.vertexInFace_count = size(ref_patch_mesh.vertexInFace,1);
ref_patch_mesh.vertexcolor = ref_mesh.vertexcolor(vertex_index,:);
ref_patch_mesh.texture_coor = ref_mesh.texture_coor(vertex_index,:);
keypoint_index_in_patch = ismember(ref_patch_mesh.vertexInFace, patch_center,'rows');
ref_patch_mesh.patch_center_index = keypoint_index_in_patch;
face_patch = face_selected;

[~, Rowindex] = ismember(ref_patch_mesh.vertexInFace, ref_mesh.vertexInFace,'rows');    
rearray_face_patch = reshape(face_patch, 1, size(face_patch,1)*size(face_patch,2))';
[~,new_face_index] = ismember(rearray_face_patch,Rowindex,'rows');
face_patch_re = reshape(new_face_index,size(face_patch,1),size(face_patch,2));
ref_patch_mesh.face = face_patch_re;
ref_patch_mesh.face_count = size(face_patch_re,1);


function Meshgraph = Mesh_to_graph(ref_patch_mesh)

dis_W = zeros(ref_patch_mesh.vertexInFace_count, ref_patch_mesh.vertexInFace_count);
W = zeros(ref_patch_mesh.vertexInFace_count, ref_patch_mesh.vertexInFace_count);

face_matrix = ref_patch_mesh.face;
edge1 = face_matrix(:,[1,2]);
edge2 = face_matrix(:,[1,3]);
edge3 = face_matrix(:,[2,3]);
edge = unique([edge1; edge2; edge3],"rows");
coord_point_1 = ref_patch_mesh.vertexInFace(edge(:,1),:);
coord_point_2 = ref_patch_mesh.vertexInFace(edge(:,2),:);

Euclidean_distance = sqrt((coord_point_1(:,1)-coord_point_2(:,1)).^2+ ...
    (coord_point_1(:,2)-coord_point_2(:,2)).^2+(coord_point_1(:,3)-coord_point_2(:,3)).^2);

radius = mean(Euclidean_distance);
distance = exp(-(Euclidean_distance)./(radius*2));
for i=1:1:length(distance)
    W(edge(i,1),edge(i,2)) = distance(i,:);
    W(edge(i,2),edge(i,1)) = distance(i,:);
    dis_W(edge(i,1),edge(i,2)) = Euclidean_distance(i,:);
    dis_W(edge(i,2),edge(i,1)) = Euclidean_distance(i,:);
end
W_flage = (W>0);
D = zeros(size(W,1),size(W,2));
for i =1:1:size(D,1)
    D(i,i) = sum(W(i,:));
end

D_flage = sum(W_flage,2);
%% adjacency matrix
Meshgraph.W = W;
Meshgraph.W_flage = W_flage;
%% degree matrix
Meshgraph.D = D;
Meshgraph.D_flage = D_flage;
Meshgraph.dis_w = dis_W;
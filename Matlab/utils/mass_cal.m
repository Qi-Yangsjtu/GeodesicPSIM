function massmatric = mass_cal(ref_patch_mesh)

face = ref_patch_mesh.face;

nvert = ref_patch_mesh.vertexInFace_count;
 
pointInface = ref_patch_mesh.vertexInFace;

face_point1 = pointInface(face(:,1),:);
face_point2 = pointInface(face(:,2),:);
face_point3 = pointInface(face(:,3),:);

edge1 = (sum((face_point2-face_point3).^2, 2)).^0.5;
edge2 = (sum((face_point1-face_point3).^2, 2)).^0.5;
edge3 = (sum((face_point1-face_point2).^2, 2)).^0.5;

edge = [edge1, edge2, edge3];

%% local averaging area
massmatric = full(massmatrix_intrinsic(edge, face, nvert, 'voronoi'));

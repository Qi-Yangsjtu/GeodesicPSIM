function results = Metric_feature(N, ref_obj_name,ref_texturemap_name,dis_obj_name,dis_texturemap_name)

disp('Loading reference mesh...')
[ref_face, ref_vertex, ref_extra] = read_obj(ref_obj_name);
ref_texture = imread(ref_texturemap_name);

disp('Loading distorted mesh...')
[dis_face, dis_vertex, dis_extra] = read_obj(dis_obj_name);
dis_texture = imread(dis_texturemap_name);

disp('Calculating GeodesicPSIM:')
results = Feature_extraction(N, ref_face, ref_vertex, ref_extra,ref_texture,dis_face, dis_vertex, dis_extra, dis_texture);
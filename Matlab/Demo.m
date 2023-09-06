addpath('./utils/');

% path of reference mesh
ref_mesh_obj_path = './meshes/Reference/creature_box_squid_C0-L5_deq_tri.obj';
ref_mesh_png_path = './meshes/Reference/creature_box_squid_C0-L5_deq_tri_0.png';


% path of distorted mesh
dis_mesh_obj_path = './meshes/Distortion/creature_box_squid_dec0.10_qp8_qt7_cqlevel_63.obj';
dis_mesh_png_path = './meshes/Distortion/creature_box_squid_dec0.10_qp8_qt7_cqlevel_63_0.png';

disp('GeodesicPSIM Matlab Implementation')
disp('Copyright: Chino Yang (Qi Yang), Tencent Media Lab, chinoyang@tencent.com.')
disp('*************************************************************************************')

% number of keypoint
% N = 5 for fast code test: 
% please set you own N: larger N, better performance, longer computation
% time
disp('Number of keypoint = 5 for fast testing!')
N=5;

% Feature extraction
Features = Metric_feature(N, ref_mesh_obj_path,ref_mesh_png_path,dis_mesh_obj_path,dis_mesh_png_path);

% Feature pooling
disp('Feature pooling...')
score = feature_pooling(Features);
GeodesicPSIM = score;

% Output result
disp('GeodesicPSIM result:')
disp(GeodesicPSIM);
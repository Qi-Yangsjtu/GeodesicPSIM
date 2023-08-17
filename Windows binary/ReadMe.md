# Usage:

A binary is provided:

•	GeodesicPSIM.exe, for Windows (to be launched with administrator rights).

The texture file can be a PNG or a JPEG image, and the mesh file should be an OBJ mesh.

A Python Script is provided to test mesh sample. An example of the command line is the following:

python GeodesicPSIMMetric.py \

--ref_obj_path        ./testSample/reference/

--ref_img_path        ./testSample/reference/ 

--dis_obj_path        ./testSample/distorted/ 

--dis_img_path       ./testSample/distorted/ 

--N                 500

--results_path        ./testSample/score/

--results_file_name    result_GeodesicPSIM.csv


with:  

--ref_obj_path: path to the reference mesh sequence.

--ref_img_path: path to the texture map of reference mesh sequence.

--dis_obj_path: path to the distorted mesh sequence.

--dis_img_path: path to the texture map of distorted mesh sequence.

--N: number of keypoint used to calculate GeodesicPSIM, default value is 500.

--results_path: path to save the results.

--results_file_name: name of the results file (with csv extension).  


Note: the number of reference (or distorted) .obj/ply files and .png/jpg files should be the same, and the number of reference and distorted files should be the same. 

This Python Script calculates the objective scores of the mesh sequence. 

Test:

Test data are provided in the testSample\ folder. There is a pair of mesh available for testing: wiz_boots_C0-L5_deq_tri.obj and wiz_boots_dec0.10_qp7_qt6_cqlevel_63.obj. By using the Python script, the following csv file is produced:

./testSample/score/result_GeodesicPSIM.csv

Patch color smoothness, patch discrete mean curvature, patch pixel color average, and patch pixel color variance are four features used to calculate the final objective scores, i.e., GeodesicPSIM score. For more technology details, refer to our publication:

Yang, Q., Jung, J., Xu, X., & Liu, S. (2023). GeodesicPSIM: Predicting the Quality of Static Mesh with Texture Map via Geodesic Patch Similarity. arXiv preprint arXiv:2308.04928.

Mesh file normalization before using this software:

To ensure obtain correct results, we suggest that you pay attention to the following principles when testing the quality of mesh with this software:

1.	The mesh files consist of three parts, including .obj, .png (or other image formats), and .mtl files. To test objective quality, .obj and .png files are required.
   
2.	For .obj file, please make sure that the raw data meets the formatting requirements of the obj file. For example: the vt (UV) information should be normalized to 0-1 according to the resolution of the texture map.

3.	Please make sure the mesh sample can be normally rendered via MeshLab by directly dragging the obj file into MeshLab before using the software.

4. The mesh must be triangle mesh.	




# GeodesicPSIM
[GeodesicPSIM: Predicting the Quality of Static Mesh with Texture Map via Geodesic Patch Similarity][link]

[link]:https://scholar.google.com.hk/citations?view_op=view_citation&hl=zh-CN&user=87nXDYAAAAAJ&citation_for_view=87nXDYAAAAAJ:MXK_kJrjxJIC

Contributor:

Chino Yang (Qi Yang)

Tencent Media Lab

chinoyang@tencent.com

## C++ implementation 

Dependencies: OpenCV OpenMp

Make sure OpenCV and OpenMp can work normally on your computer.

1. [OpenCV](https://opencv.org/releases/), version 4.8.0 is recommended.

   Add openCV to the environment path, For example:

   D:\opencv\build\x64\vc16\bin

   D:\opencv\build\bin

2. If you have installed VS, OpenMp is already supported. Otherwise, please check [tdm-gcc](https://jmeubank.github.io/tdm-gcc/).


### Windows10 installation

1. Unzip eigen-3.4.0
2. If you have installed Git Bash on your computer, please use the following commands:
   
   2.1 mkdir build
   
   2.2 cd build
   
   2.3 cmake ..

3. If you do not have Git Bash:
   
   3.1 Run cmake-gui.exe

   3.2 Choose 'Visual Studio 17 2022 Win64' as the compiler version

   3.3 Where is the source code = ".../GeodesicPSIM/GeodesicPSIM_Code/"

   3.4 Where to build the binaries = ".../GeodesicPSIM/build"

   3.5 Click "Configure"

   3.6 Click "Generate"

4. Open ".../GeodesicPSIM/build/GeodesicPSIM.sln" solution with MSVC 2022, select 'Release' mode, then select GeodesicPSIM as the starting project.

5. To run the software:

   GeodesicPSIM.exe ReferenceMesh(obj) ReferenceMeshTextureMap DistortedMesh(obj) DistortedMeshTextureMap --N < Number of keypoint, default is 500 > --result_file < file to save results, default is result.csv >

   A pair of meshes is provided with '/Windows binary/testSample.zip' for testing.
   
7. Check the results.

   Patch color smoothness, patch discrete mean curvature, patch pixel color average, and patch pixel color variance are four features used to calculate the final objective scores, i.e., GeodesicPSIM score.

#### If you cannot obtain the results with your own meshes, please check your mesh .obj file and compare them with the testing samples. 

1.	The mesh files consist of three parts, including .obj, .png (or other image formats), and .mtl files. To test objective quality, .obj and .png files are required.
   
2.	For .obj file, please make sure that the raw data meets the formatting requirements of the obj file. For example: the vt (UV) information should be normalized to 0-1 according to the resolution of the texture map.
  
3.	Please make sure the mesh sample can be normally rendered via MeshLab by directly dragging the obj file into MeshLab before using the software. Otherwise, it might cause some unexpected errors.

4. The mesh must be triangle mesh.



#### If you do not want to compile the source code by yourself, we also provide a binary in the Windows binary folder

A Python script, GeodesicPSIMMetric.py, and a ReaMe.md is provided to help you use the binary.

## Matlab implementation

1. Open Demo.m
  
2. Run the script for fast testing
   
3. If successful, set your own N (N=500 in our publication)

### Note: the results between C++ and Matlab implementations might have some minor differences

# GeodesicPSIM
GeodesicPSIM: Predicting the Quality of Static Mesh with Texture Map via Geodesic Patch Similarity

chinoyang@tencent.com

## C++ implementation 

Dependencies: OpenCV OpenMp

Make sure OpenCV and OpenMp can work normally on your computer.

### Windows10 installation

1. Unzip eigen-3.4.0
2. If you have installed Git Bash on your computer, please use the following commands:
   
   2.1 mkdir build
   
   2.2 cd build
   
   2.3 cmake ..

3. If you do not have Git Bash:
   
   3.1 Run cmake-gui.exe

   3.2 Choose 'Visual Studio 17 2022 Win64' as the compiler version

   3.3 Where is the source code = ".../GeodesicPSIM/"

   3.4 Where to build the binaries = ".../GeodesicPSIM/build"

   3.5 Click "Configure"

   3.6 Click "Generate"

4. Open ".../GeodesicPSIM/build/GeodesicPSIM.sln" solution with MSVC 2022, select 'Release' mode, then select Gps as the starting project.

### If you do not want to compile the source code by yourself, we also provide a binary in C++ binary folder

A python script is provided to help you use the binary.

## Matlab implementation

1. Open Demo.m
  
2. Run the script for fast testing
   
3. If success, set your own N (N=500 in our publication)

cmake_minimum_required(VERSION 3.0)

project (GeodesicPSIM)

aux_source_directory(./GeodesicPSIM_Code DIR_SRCS)

# Config Eigen

set(Eigen3_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/eigen-3.4.0/eigen-3.4.0")

include_directories(${Eigen3_INCLUDE_DIR})

# Config OpenCV

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(Gps ${DIR_SRCS})

target_link_libraries(Gps ${OpenCV_LIBS})

#Config OpenMP

find_package(OpenMP REQUIRED)

target_link_libraries(Gps OpenMP::OpenMP_CXX)
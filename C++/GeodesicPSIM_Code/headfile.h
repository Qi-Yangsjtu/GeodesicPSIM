#pragma once

#include <opencv2/opencv.hpp>
#include <iostream>
#include "vector"
#include "string"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SparseCore>
#include <algorithm>
#include <random>
#include <unordered_set>
#include <cmath>
#include <unordered_map>
#include "omp.h"
#include <time.h>

#include <fstream>
#include <sstream>


#include "common.h"

#include "read_obj.h"
#include "mesh_clean.h"
#include "normalized.h"
#include "create_local_patch.h"
#include "Mesh_to_graph.h"
#include "patch_cropping.h"
#include "face_texture_cal.h"
#include "cotmatrix.h"
#include "mass_cal.h"
#include "massmatrix_intrinsic.h"
#include "compute_mesh_normals.h"
#include "rescalePatch.h"
#include "Feature_pooling.h"
#include "farthest_point_sampling.h"
#include "isPointInsidePolygon.h"
#include "GeodesicPSIM.h"

#include "Feature_extraction.h"

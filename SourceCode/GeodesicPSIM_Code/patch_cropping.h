#pragma once
#include "patch_cropping.h"

void patch_cropping(Patch_Mesh& ref_patch_mesh, Patch_Mesh& dis_patch_mesh,
	Mesh2Graph ref_Mesh_graph_ini, Mesh2Graph dis_Mesh_graph_ini,
	Image3& ref_texture, Image3& dis_texture, double threshold, double &ref_patch_size);
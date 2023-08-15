#pragma once
#pragma once

#include <vector>
#include <map>
#include <set>
#include <string>
#include "Common.h"

extern bool readfilename(std::string filename, Vector_Mesh& mesh, Extra& extra);
extern void read_obj(std::string filename, Mesh& mesh, Extra& extra);
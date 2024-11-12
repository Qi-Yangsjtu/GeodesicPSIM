#pragma once

void rescalePatch(Patch_Mesh& dis_patch_mesh, double scale, Image3& dis_texture);
void uv2yuv(Eigen::MatrixXd& vertexcolor, Image3& dis_texture, Eigen::MatrixXd uv_group);
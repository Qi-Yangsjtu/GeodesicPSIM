#include "headfile.h"

Eigen::MatrixXd compute_mesh_normals(Patch_Mesh &inputmesh) {
	Eigen::MatrixXi faces = inputmesh.face;
	Eigen::MatrixXd vertex = inputmesh.vertexInFace;
	int facetCOUNT = faces.rows();
	
	Eigen::MatrixXd coordVERTICES_1(facetCOUNT, 3);
	Eigen::MatrixXd coordVERTICES_2(facetCOUNT, 3);
	Eigen::MatrixXd coordVERTICES_3(facetCOUNT, 3);

	Eigen::MatrixXd coordNORMALS(facetCOUNT, 3);

	for (int loopa = 0; loopa < facetCOUNT; loopa++) {
		coordVERTICES_1.row(loopa) = vertex.row((faces(loopa, 0)));
		coordVERTICES_2.row(loopa) = vertex.row((faces(loopa, 1)));
		coordVERTICES_3.row(loopa) = vertex.row((faces(loopa, 2)));
	}

	for (int loopFACE = 0; loopFACE < facetCOUNT; ++loopFACE)
	{

		Eigen::Vector3d cornerA;
		Eigen::Vector3d cornerB;
		Eigen::Vector3d cornerC;
		
		cornerA << coordVERTICES_1(loopFACE, 0), coordVERTICES_1(loopFACE, 1), coordVERTICES_1(loopFACE, 2);
		cornerB << coordVERTICES_2(loopFACE, 0), coordVERTICES_2(loopFACE, 1), coordVERTICES_2(loopFACE, 2);
		cornerC << coordVERTICES_3(loopFACE, 0), coordVERTICES_3(loopFACE, 1), coordVERTICES_3(loopFACE, 2);


		Eigen::Vector3d AB = cornerB - cornerA;
		Eigen::Vector3d AC = cornerC - cornerA;

		Eigen::Vector3d ABxAC = AB.cross(AC);
		ABxAC.normalize();
		coordNORMALS.row(loopFACE) = ABxAC;

	}

	return coordNORMALS;
}

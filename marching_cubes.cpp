#include "marching_cubes.h"

F32Array3::F32Array3(int nx, int ny, int nz) : nx(nx), ny(ny), nz(nz) {
	data.resize(nx * ny * nz);
}


TriangleMesh<Eigen::Vector3f> extractIsosurface(
	float value, ScalarField3 field,
	BoundingBox region, float resolution) {
	assert(resolution > 0);

	return TriangleMesh<Eigen::Vector3f>();
}

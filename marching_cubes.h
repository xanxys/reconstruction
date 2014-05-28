#pragma once

#include <vector>

#include <Eigen/Dense>

class F32Array3 {
public:
	F32Array3(int nx, int ny, int nz);
private:
	const int nx, ny, nz;
	std::vector<float> data;
};

template<typename Vertex>
class TriangleMesh {
public:
	std::vector<std::tuple<int, int, int>> triangles;
	std::vector<std::pair<Eigen::Vector3f, Vertex>> vertices;
};


using ScalarField3 = std::function<float(Eigen::Vector3f)>;
using BoundingBox = std::pair<Eigen::Vector3f, Eigen::Vector3f>;

// Create triangle mesh of isosurface.
// Returned mesh is hole-free and contains normals of each vertices.
// Normal points toward in a direction that value becomes smaller.
TriangleMesh<Eigen::Vector3f> extractIsosurface(
	float value, ScalarField3 field,
	BoundingBox region, float resolution = 1.0);

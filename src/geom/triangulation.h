#pragma once

#include <array>
#include <vector>

#include <Eigen/Dense>

namespace recon {

// Triangulate a CCW simple polygon (no self intersecting edges, no holes)
// into CCW triangles. N-vertex polygon always results in N-2 triangles.
//
// Useful course slide(not used):
// https://www.cs.ucsb.edu/~suri/cs235/Triangulation.pdf
std::vector<std::array<int, 3>> triangulatePolygon(
	const std::vector<Eigen::Vector2f>& points);

}  // namespace

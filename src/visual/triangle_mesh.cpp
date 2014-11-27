#include "triangle_mesh.h"

#include <boost/range/irange.hpp>

namespace recon {

TriangleMesh<std::nullptr_t> createBox(
		Eigen::Vector3f center,Eigen::Vector3f half_dx,
		Eigen::Vector3f half_dy, Eigen::Vector3f half_dz) {
	TriangleMesh<std::nullptr_t> box;
	for(int i : boost::irange(0, 8)) {
		const Eigen::Vector3f vertex_pos = center +
			((i & 0b001) ? 1 : -1) * half_dx +
			((i & 0b010) ? 1 : -1) * half_dy +
			((i & 0b100) ? 1 : -1) * half_dz;
		box.vertices.push_back(std::make_pair(vertex_pos, nullptr));
	}

	// Create inward-facing triangles. (CCW is positive direction)
	// Draw a cube with 000-111 to understand this.
	box.triangles = {
		// X-
		{{0, 4, 2}},
		{{6, 2, 4}},
		// X+
		{{5, 1, 7}},
		{{3, 7, 1}},
		// Y-
		{{0, 1, 4}},
		{{5, 4, 1}},
		// Y+
		{{6, 7, 2}},
		{{3, 2, 7}},
		// Z-
		{{0, 2, 1}},
		{{3, 1, 2}},
		// Z+
		{{6, 4, 7}},
		{{5, 7, 4}}
	};

	return box;
}

TriangleMesh<std::nullptr_t> createBox(
	Eigen::Vector3f center, float half_size) {
	return createBox(center,
		Eigen::Vector3f::UnitX() * half_size,
		Eigen::Vector3f::UnitY() * half_size,
		Eigen::Vector3f::UnitZ() * half_size);
}

}  // namespace

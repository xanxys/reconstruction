#pragma once

#include <boost/optional.hpp>
#include <Eigen/Dense>

#include <visual/raytracing.h>
#include <visual/triangle_mesh.h>

namespace visual {

class MeshIntersecter {
public:
	MeshIntersecter(const TriangleMesh<std::nullptr_t>& mesh);

	// Return (triangle index, uv) of the nearest triangle (if any).
	boost::optional<std::tuple<int, Eigen::Vector2f>> intersect(const Ray& ray);

	const TriangleMesh<std::nullptr_t>& getMesh() const;
private:
	TriangleMesh<std::nullptr_t> mesh;
};

}  // namespace

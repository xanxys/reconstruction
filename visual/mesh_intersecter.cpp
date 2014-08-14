#include "mesh_intersecter.h"

#include <boost/range/irange.hpp>

#include <visual/raytracing.h>

namespace visual {

MeshIntersecter::MeshIntersecter(const TriangleMesh<std::nullptr_t>& mesh) :
		mesh(mesh) {
}

boost::optional<std::tuple<int, Eigen::Vector2f>>
		MeshIntersecter::intersect(const Ray& ray) {
	Material m;
	for(int i : boost::irange(0, (int)mesh.triangles.size())) {
		const auto& tri = mesh.triangles[i];
		Triangle geom(
			mesh.vertices[std::get<0>(tri)].first,
			mesh.vertices[std::get<1>(tri)].first,
			mesh.vertices[std::get<2>(tri)].first,
			m);

		float t;
		Eigen::Vector3f normal;
		if(geom.intersect(ray, t, normal)) {
			ray.at(t);
			return boost::make_optional(std::make_tuple(i, Eigen::Vector2f(0, 0)));
		}
	}
	return boost::none;
}

const TriangleMesh<std::nullptr_t>& MeshIntersecter::getMesh() const {
	return mesh;
}

}  // namespace

#include "mesh_intersecter.h"

#include <boost/optional.hpp>
#include <boost/range/irange.hpp>

#include <visual/raytracing.h>

namespace visual {

MeshIntersecter::MeshIntersecter(const TriangleMesh<std::nullptr_t>& mesh) :
		mesh(mesh) {
}

boost::optional<std::tuple<int, Eigen::Vector2f, float>>
		MeshIntersecter::intersect(const Ray& ray) {
	boost::optional<std::tuple<int, Eigen::Vector2f, float>> nearest_hit;
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
		Eigen::Vector2f bary;
		if(geom.intersect(ray, t, normal, bary)) {
			if(!nearest_hit || t < std::get<2>(*nearest_hit)) {
				nearest_hit = std::make_tuple(i, bary, t);
			}
		}
	}
	return nearest_hit;
}

const TriangleMesh<std::nullptr_t>& MeshIntersecter::getMesh() const {
	return mesh;
}

}  // namespace

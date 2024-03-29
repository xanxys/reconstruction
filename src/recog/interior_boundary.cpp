#include "interior_boundary.h"

#include <boost/range/irange.hpp>

namespace recon {

InteriorBoundary::InteriorBoundary(
		const TexturedMesh& mesh,
		const std::vector<Eigen::Vector2f>& wall_polygon,
		const std::pair<float, float>& z_range) :
		wall_polygon(wall_polygon), mesh(mesh) {
	assert(wall_polygon.size() >= 3);
	const Eigen::Vector3f local_origin_in_world(
		0, 0, z_range.first);

	height = z_range.second - z_range.first;
	assert(height > 0);

	local_to_world = Eigen::Affine3f::Identity();
	local_to_world.translation() = local_origin_in_world;

	// Transform mesh to local coordinates.
	const Eigen::Affine3f world_to_local = local_to_world.inverse();
	for(auto& vert : this->mesh.mesh.vertices) {
		vert.first = world_to_local * vert.first;
	}
}

Eigen::Affine3f InteriorBoundary::getPose() const {
	return local_to_world;
}

TexturedMesh InteriorBoundary::getMesh() const {
	return mesh;
}

std::vector<OBB3f> InteriorBoundary::getCollision(float thickness) const {
	assert(thickness > 0);
	// Calculate 2D aabb.
	Eigen::Vector2f vmin_2d(1e10, 1e10);
	Eigen::Vector2f vmax_2d = -vmin_2d;
	for(const auto& pt : wall_polygon) {
		vmin_2d = vmin_2d.cwiseMin(pt);
		vmax_2d = vmax_2d.cwiseMax(pt);
	}
	// Extend by thickness to secure edges.
	vmin_2d -= Eigen::Vector2f(thickness, thickness);
	vmax_2d += Eigen::Vector2f(thickness, thickness);

	std::vector<OBB3f> collision;
	// wall
	//   <outside>
	//   __________   <- OBB     ^ anti_normal
	// __|_________|_            |
	//   i1        i0
	//   <inside>
	Eigen::Matrix2f ev_to_an;
	ev_to_an.col(0) = Eigen::Vector2f(0, -1);
	ev_to_an.col(1) = Eigen::Vector2f(1, 0);
	for(const int i0 : boost::irange(0, (int)wall_polygon.size())) {
		// Calculate 2D OBB.
		const int i1 = (i0 + 1) % wall_polygon.size();
		const Eigen::Vector2f edge_v = wall_polygon[i1] - wall_polygon[i0];
		const Eigen::Vector2f anti_normal = ev_to_an * edge_v.normalized();
		const Eigen::Vector2f normal = -anti_normal;
		const Eigen::Vector2f center_2d =
			(wall_polygon[i0] + wall_polygon[i1]) / 2 +
			anti_normal * (thickness * 0.5);
		// Create 3D OBB from 2D OBB.
		const Eigen::Vector3f center(
			center_2d.x(), center_2d.y(), height / 2);
		Eigen::Matrix3f axis;
		axis.col(0) = Eigen::Vector3f(edge_v.x(), edge_v.y(), 0);
		axis.col(1) = Eigen::Vector3f(
			normal.x() * thickness, normal.y() * thickness, 0);
		axis.col(2) = Eigen::Vector3f(0, 0, height);
		collision.emplace_back(center, axis);
	}
	// floor + ceiling
	std::vector<float> z_centers = {-thickness / 2, height + thickness / 2};
	for(const float z_center : z_centers) {
		const Eigen::Vector2f size_2d = vmax_2d - vmin_2d;
		const Eigen::Vector2f center_2d = (vmin_2d + vmax_2d) / 2;

		const Eigen::Vector3f center(center_2d.x(), center_2d.y(), z_center);
		Eigen::Matrix3f axis;
		axis.col(0) = Eigen::Vector3f(size_2d.x(), 0, 0);
		axis.col(1) = Eigen::Vector3f(0, size_2d.y(), 0);
		axis.col(2) = Eigen::Vector3f(0, 0, thickness);
		collision.emplace_back(center, axis);
	}
	return collision;
}

}  // namespace

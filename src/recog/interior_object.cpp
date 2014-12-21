#include "interior_object.h"

namespace recon {

InteriorObject::InteriorObject(
		const TexturedMesh& mesh, const std::vector<OBB3f>& collision) {
	assert(!collision.empty());
	// Calculate whole collision to estimate lowest Z and center.
	AABB3f whole_collision = collision.front().toAABB();
	for(const auto& coll_obb : collision) {
		whole_collision = whole_collision | coll_obb.toAABB();
	}
	const Eigen::Vector3f center =
		(whole_collision.getMin() + whole_collision.getMax()) / 2;
	const Eigen::Vector3f local_origin_in_world(
		center.x(), center.y(), whole_collision.getMin().z());

	local_to_world = Eigen::Affine3f::Identity();
	local_to_world.translation() = local_origin_in_world;

	// Transform mesh to local coordinates.
	assert(mesh.has_normal);
	const Eigen::Affine3f world_to_local = local_to_world.inverse();
	this->mesh = mesh;
	for(auto& vert : this->mesh.mesh_w_normal.vertices) {
		vert.first = world_to_local * vert.first;
	}
	for(const auto& coll_obb : collision) {
		this->collision.push_back(
			coll_obb.rigidlyTransformed(world_to_local));
	}
}

Eigen::Affine3f InteriorObject::getPose() const {
	return local_to_world;
}

TexturedMesh InteriorObject::getMesh() const {
	return mesh;
}

std::vector<OBB3f> InteriorObject::getCollision() const {
	return collision;
}

}  // namespace

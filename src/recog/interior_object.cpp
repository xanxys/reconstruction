#include "interior_object.h"

namespace recon {

InteriorObject::InteriorObject(
		const TexturedMesh& mesh, const std::vector<OBB3f>& collision) {
	assert(!collision.empty());

	local_to_world = Eigen::Affine3f::Identity();
	this->mesh = mesh;
	this->collision = collision;
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

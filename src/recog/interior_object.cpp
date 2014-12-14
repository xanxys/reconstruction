#include "interior_object.h"

namespace recon {

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

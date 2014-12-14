#pragma once

#include <vector>

#include <Eigen/Dense>

#include <geom/util.h>
#include <visual/textured_mesh.h>

namespace recon {

// An InteriorObject is an object that can move and collide
// with each other and exterior, generating sounds.
// This corresponds to NoisyActor in EqExperiment.
//
// The up axis in the coordinates is Z+, but Z=0 should be
// bottom surface of the object, and object are generally
// aligned to somewhat close to origin in XY-plane,
// following common practice in 3D modeling.
class InteriorObject {
public:
	// Given mesh and collision shape in world coordinates,
	// guess good pose from collisions.
	InteriorObject(
		const TexturedMesh& mesh, const std::vector<OBB3f>& collision);

	Eigen::Affine3f getPose() const;
	TexturedMesh getMesh() const;
	std::vector<OBB3f> getCollision() const;
private:
	Eigen::Affine3f local_to_world;
	TexturedMesh mesh;
	std::vector<OBB3f> collision;
};

}  // namespace

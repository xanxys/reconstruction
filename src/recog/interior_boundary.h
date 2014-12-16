#pragma once

#include <vector>

#include <Eigen/Dense>

#include <geom/util.h>
#include <visual/textured_mesh.h>

namespace recon {

// An InteriorBoundary is an object that have walls/floor/ceiling
// corresponding to inside of a room, represented as
// mesh and collision shape.
//
// The up axis in the coordinates is Z+, but Z=0 should be
// bottom surface of the object, and object are generally
// aligned to somewhat close to origin in XY-plane,
// following common practice in 3D modeling.
class InteriorBoundary {
public:
	// Given mesh and collision shape in world coordinates,
	// guess good pose from collisions.
	// wall_polygon must be CCW, and doesn't have any self-intersection
	// or strange things.
	InteriorBoundary(
		const TexturedMesh& mesh,
		const std::vector<Eigen::Vector2f>& wall_polygon,
		const std::pair<float, float>& z_range);

	Eigen::Affine3f getPose() const;
	TexturedMesh getMesh() const;

	// Construct collision boxes of given thickness.
	std::vector<OBB3f> getCollision(float thickness) const;
private:
	Eigen::Affine3f local_to_world;  // only contains Z translation
	std::vector<Eigen::Vector2f> wall_polygon;
	float height;
	TexturedMesh mesh;
};

}  // namespace

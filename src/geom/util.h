#pragma once

#include <Eigen/Dense>

namespace recon {

// Create 3-d orthogonal basis
// (x y z) from given z. (column vector = new axis)
// z must be a unit vector.
Eigen::Matrix3f createOrthogonalBasis(
	const Eigen::Vector3f& z);

}  // namespace

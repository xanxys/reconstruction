#pragma once

#include <Eigen/Dense>

namespace recon {

class AABB3f {
public:
	AABB3f(const Eigen::Vector3f& vmin, const Eigen::Vector3f& vmax);
	Eigen::Vector3f getMin() const;
	Eigen::Vector3f getMax() const;
	Eigen::Vector3f getSize() const;
	bool contains(const Eigen::Vector3f& query) const;
	float getVolume() const;

	AABB3f enlarged(float dsize) const;

	bool overlap(const AABB3f& other) const;
private:
	Eigen::Vector3f vmin;
	Eigen::Vector3f vmax;
};


// Create 3-d orthogonal basis
// (x y z) from given z. (column vector = new axis)
// z must be a unit vector.
Eigen::Matrix3f createOrthogonalBasis(
	const Eigen::Vector3f& z);

}  // namespace

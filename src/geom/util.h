#pragma once

#include <Eigen/Dense>

namespace recon {

class AABB3f {
public:
	AABB3f(const Eigen::Vector3f& vmin, const Eigen::Vector3f& vmax);
	Eigen::Vector3f getMin() const;
	Eigen::Vector3f getMax() const;
	Eigen::Vector3f getCenter() const;
	Eigen::Vector3f getSize() const;
	bool contains(const Eigen::Vector3f& query) const;
	float getVolume() const;

	AABB3f enlarged(float dsize) const;

	bool overlap(const AABB3f& other) const;
private:
	Eigen::Vector3f vmin;
	Eigen::Vector3f vmax;
};

AABB3f operator|(AABB3f lhs, const AABB3f& rhs);

class OBB3f {
public:
	OBB3f(const AABB3f& aabb);
	OBB3f(const AABB3f& aabb, const Eigen::Affine3f& pose);

	// axis: each column vector is X, Y, Z edge.
	OBB3f(const Eigen::Vector3f& center, const Eigen::Matrix3f& axis);

	// Return OBB that is transformed by a rigid transform.
	// if trans is rigid, result is undefined.
	// This transformation won't make OBB bigger.
	OBB3f rigidlyTransformed(const Eigen::Affine3f& trans) const;

	// Return AABB that contains this OBB.
	AABB3f toAABB() const;

	std::pair<Eigen::Vector3f, Eigen::Matrix3f> getCenterAndAxis() const;
private:
	OBB3f();  // used for construction
private:
	Eigen::Vector3f center;
	Eigen::Matrix3f axis;  // each col vector is X, Y, Z full edge vector.
};


// Create 3-d orthogonal basis
// (x y z) from given z. (column vector = new axis)
// z must be a unit vector.
Eigen::Matrix3f createOrthogonalBasis(
	const Eigen::Vector3f& z);

}  // namespace

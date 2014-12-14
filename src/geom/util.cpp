#include "util.h"

namespace recon {


AABB3f::AABB3f(const Eigen::Vector3f& vmin, const Eigen::Vector3f& vmax) :
	vmin(vmin), vmax(vmax) {
}

Eigen::Vector3f AABB3f::getMin() const {
	return vmin;
}

Eigen::Vector3f AABB3f::getMax() const {
	return vmax;
}

Eigen::Vector3f AABB3f::getSize() const {
	return vmax - vmin;
}

bool AABB3f::contains(const Eigen::Vector3f& query) const {
	return (vmin.array() <= query.array()).all() &&
		(query.array() <= vmax.array()).all();
}

float AABB3f::getVolume() const {
	return (vmax - vmin).prod();
}

AABB3f AABB3f::enlarged(float dsize) const {
	assert(dsize >= 0);
	const Eigen::Vector3f offset =
		Eigen::Vector3f(1, 1, 1) * (dsize * 0.5);
	return AABB3f(vmin - offset, vmax + offset);
}

bool AABB3f::overlap(const AABB3f& other) const {
	const auto n_vmin = vmin.cwiseMax(other.vmin);
	const auto n_vmax = vmax.cwiseMin(other.vmax);
	return (n_vmin.array() < n_vmax.array()).all();
}


OBB3f::OBB3f(const AABB3f& aabb) :
	vmin(aabb.getMin()), vmax(aabb.getMax()),
	local_to_world(Eigen::Matrix3f::Identity()) {
}


Eigen::Matrix3f createOrthogonalBasis(
		const Eigen::Vector3f& z) {
	// Choose an arbitrary unit vector
	// not too parallel to z, to increase stability.
	const auto seed = (std::abs(z.x()) > 0.5) ?
		Eigen::Vector3f(0, 1, 0) :
		Eigen::Vector3f(1, 0, 0);

	Eigen::Matrix3f basis;
	basis.col(0) = z.cross(seed).normalized();
	basis.col(1) = z.cross(basis.col(0));
	basis.col(2) = z;
	return basis;
}

}  // namespace

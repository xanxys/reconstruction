#include "util.h"

#include <boost/range/irange.hpp>

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

Eigen::Vector3f AABB3f::getCenter() const {
	return (vmin + vmax) / 2;
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

AABB3f operator|(AABB3f lhs, const AABB3f& rhs) {
	return AABB3f(
		lhs.getMin().cwiseMin(rhs.getMin()),
		lhs.getMax().cwiseMax(rhs.getMax()));
}


OBB3f::OBB3f() {
}

OBB3f::OBB3f(const AABB3f& aabb) :
		center((aabb.getMin() + aabb.getMax()) / 2),
		axis((aabb.getMax() - aabb.getMin()).asDiagonal()) {
}

OBB3f::OBB3f(const AABB3f& aabb, const Eigen::Affine3f& pose) {
	center = pose * aabb.getCenter();
	axis = pose.linear() * aabb.getSize().asDiagonal();
}

OBB3f::OBB3f(const Eigen::Vector3f& center, const Eigen::Matrix3f& axis) :
		center(center), axis(axis) {
	Eigen::Matrix3f naxis = axis;
	const Eigen::Vector3f size(
		axis.col(0).norm(), axis.col(1).norm(), axis.col(2).norm());
	naxis.col(0) /= size(0);
	naxis.col(1) /= size(1);
	naxis.col(2) /= size(2);
	assert(std::abs(naxis.determinant() - 1) < 1e-3);
}

AABB3f OBB3f::toAABB() const {
	Eigen::Vector3f n_vmin(1e10, 1e10, 1e10);
	Eigen::Vector3f n_vmax = -n_vmin;
	const Eigen::Matrix3f half_axis = axis / 2;
	for(const int i : boost::irange(0, 8)) {
		const auto v =
			(((i & 1) == 0) ? -1 : 1) * half_axis.col(0) +
			(((i & 2) == 0) ? -1 : 1) * half_axis.col(1) +
			(((i & 4) == 0) ? -1 : 1) * half_axis.col(2);
		n_vmin = n_vmin.cwiseMin(v);
		n_vmax = n_vmax.cwiseMax(v);
	}
	return AABB3f(center + n_vmin, center + n_vmax);
}

OBB3f OBB3f::rigidlyTransformed(const Eigen::Affine3f& trans) const {
	OBB3f obb;
	obb.center = trans * center;
	obb.axis = trans.linear() * axis;
	return obb;
}

std::pair<Eigen::Vector3f, Eigen::Matrix3f> OBB3f::getCenterAndAxis() const {
	return std::make_pair(center, axis);
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

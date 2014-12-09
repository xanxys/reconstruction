#include "util.h"

namespace recon {

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
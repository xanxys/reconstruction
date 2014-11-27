#include "voxel_traversal.h"

namespace recon {

Eigen::Vector3i indexToVector(VoxelIndex index) {
	return Eigen::Vector3i(
		std::get<0>(index),
		std::get<1>(index),
		std::get<2>(index));
}


VoxelTraversal::VoxelTraversal(
	float size, Eigen::Vector3f org, Eigen::Vector3f dir) :
	org(org / size), dir(dir) {

	index = Eigen::Vector3i(
		std::floor(org(0)),
		std::floor(org(1)),
		std::floor(org(2)));

	frac = org - index.cast<float>();
}

std::tuple<int, int, int> VoxelTraversal::next() {
	const auto key = std::make_tuple(index(0), index(1), index(2));

	const auto remaining = Eigen::Vector3f(
		(dir(0) < 0) ? -frac(0) : 1 - frac(0),
		(dir(1) < 0) ? -frac(1) : 1 - frac(1),
		(dir(2) < 0) ? -frac(2) : 1 - frac(2));

	const auto dt_xyz = remaining.cwiseQuotient(dir);

	// Select the direction with smallest dt. (tie-breaker: prefer X>Y>Z)
	if(dt_xyz(0) <= dt_xyz(1) && dt_xyz(0) <= dt_xyz(2)) {
		const int dix = (dir(0) < 0) ? -1 : 1;
		index(0) += dix;
		frac += dt_xyz(0) * dir;
		frac(0) -= dix;
	} else if(dt_xyz(1) <= dt_xyz(2)) {
		const int dix = (dir(1) < 0) ? -1 : 1;
		index(1) += dix;
		frac += dt_xyz(1) * dir;
		frac(1) -= dix;
	} else {
		const int dix = (dir(2) < 0) ? -1 : 1;
		index(2) += dix;
		frac += dt_xyz(2) * dir;
		frac(2) -= dix;
	}

	return key;
}

}  // namespace

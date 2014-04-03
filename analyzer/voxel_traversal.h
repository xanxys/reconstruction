#pragma once

#include <tuple>

#include <Eigen/Dense>

using VoxelIndex = std::tuple<int, int, int>;

Eigen::Vector3i indexToVector(VoxelIndex index);


class VoxelTraversal {
public:
	VoxelTraversal(float size, Eigen::Vector3f org, Eigen::Vector3f dir);
	VoxelIndex next();
private:
	const Eigen::Vector3f org;
	const Eigen::Vector3f dir;
	Eigen::Vector3i index;
	Eigen::Vector3f frac;
};

#pragma once

#include <tuple>

#include <Eigen/Dense>

class VoxelTraversal {
public:
	VoxelTraversal(float size, Eigen::Vector3f org, Eigen::Vector3f dir);
	std::tuple<int, int, int> next();
private:
	const Eigen::Vector3f org;
	const Eigen::Vector3f dir;
	Eigen::Vector3i index;
	Eigen::Vector3f frac;
};

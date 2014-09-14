#pragma once

#include <sstream>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "floor_belief.h"
#include "voxel_traversal.h"


template<typename T> std::set<T> set_difference(std::set<T>& a, std::set<T>& b) {
	std::set<T> result;
	for(const auto& v : a) {
		if(b.find(v) == b.end()) {
			result.insert(v);
		}
	}
	return result;
}


class OrientedBox {
public:
	OrientedBox(
		Eigen::Vector3f position,
		float ry,
		Eigen::Vector3f size,
		Eigen::Vector3f color,
		bool valid);

	Eigen::Vector3f getPosition() const;
	Eigen::Vector3f getSize() const;
	Eigen::Vector3f getColor() const;
	float getRotationY() const;
	bool getValid() const;
private:
	float ry;

	Eigen::Vector3f position;
	Eigen::Vector3f size;
	Eigen::Vector3f color;
	bool valid;
};


class WallBelief {
public:
	WallBelief(const WallBelief& that);
	WallBelief(const FloorBelief& floor, int index);
	static std::vector<std::shared_ptr<WallBelief>> expand(const FloorBelief& floor);
	static OrientedBox guessBoxForBlob(const FloorBelief& floor, std::vector<VoxelIndex> blob);

	std::vector<OrientedBox> getObjects() const;

	static std::vector<std::vector<VoxelIndex>> splitCC(std::vector<VoxelIndex> nodes);
public:
	mutable std::ostringstream log;

	FloorBelief floor;
	std::vector<OrientedBox> objects;
};

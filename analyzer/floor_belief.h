#pragma once

#include <sstream>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "manhattan_belief.h"

class FloorBelief {
public:
	FloorBelief(const FloorBelief& that);
	FloorBelief(const ManhattanBelief& manhattan, int index);
	static std::vector<std::shared_ptr<FloorBelief>> expand(const ManhattanBelief& manhattan);
public:
	// Put this before all other members to initialize first,
	// since logging is used in SceneAnalyzer's initializer's list.
	mutable std::ostringstream log;

	ManhattanBelief manhattan;

	int index;
};

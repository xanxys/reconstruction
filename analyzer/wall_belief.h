#pragma once

#include <sstream>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>

#include "floor_belief.h"


class WallBelief {
public:
	WallBelief(const WallBelief& that);
	WallBelief(const FloorBelief& floor, int index);
	static std::vector<std::shared_ptr<WallBelief>> expand(const FloorBelief& floor);
public:
	mutable std::ostringstream log;

	FloorBelief floor;
};

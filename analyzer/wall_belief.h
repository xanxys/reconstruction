#pragma once

#include <sstream>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>

#include "floor_belief.h"


template<typename T> std::set<T> set_difference(std::set<T>& a, std::set<T>& b) {
	std::set<T> result;
	for(const auto& v : a) {
		if(b.find(v) == b.end()) {
			result.insert(v);
		}
	}
	return result;
}

class WallBelief {
public:
	WallBelief(const WallBelief& that);
	WallBelief(const FloorBelief& floor, int index);
	static std::vector<std::shared_ptr<WallBelief>> expand(const FloorBelief& floor);

	static std::vector<std::vector<std::tuple<int, int, int>>> splitCC(
		std::vector<std::tuple<int, int, int>> nodes);

public:
	mutable std::ostringstream log;

	FloorBelief floor;


};

#include "analyzer.h"

#include <map>
#include <cmath>
#include <exception>
#include <random>

#include <boost/range/irange.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using ColorCloud = pcl::PointCloud<pcl::PointXYZRGBA>;
using RigidTrans3f = Eigen::Transform<float, 3, Eigen::AffineCompact>;

SceneAnalyzer::SceneAnalyzer(const ColorCloud::ConstPtr& raw_cloud) :
	frame(raw_cloud) {
}

std::shared_ptr<SceneBelief> SceneAnalyzer::getBestBelief() {
	auto manhattans = ManhattanBelief::expand(frame);
	auto floors = FloorBelief::expand(*manhattans[0]);
	auto walls = WallBelief::expand(*floors[0]);
	return std::make_shared<SceneBelief>(*walls[0]);
}

std::vector<std::shared_ptr<SceneBelief>> SceneAnalyzer::getAllBelief() {
	std::vector<std::shared_ptr<SceneBelief>> results;
	for(auto& manhattan : ManhattanBelief::expand(frame)) {
		for(auto& floor : FloorBelief::expand(*manhattan)) {
			for(auto& wall : WallBelief::expand(*floor)) {
				results.push_back(std::make_shared<SceneBelief>(*wall));
			}
		}
	}
	return results;
}

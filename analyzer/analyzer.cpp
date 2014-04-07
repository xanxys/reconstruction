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
	const auto results = getAllBelief();
	return *std::min_element(results.begin(), results.end(),
		[](const std::shared_ptr<SceneBelief>& a,
			const std::shared_ptr<SceneBelief> b) {
			return getScore(*a) < getScore(*b);
		});
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

float SceneAnalyzer::getScore(const SceneBelief& belief) {
	const cv::Mat target = belief.getRGBImage();
	const cv::Mat render = belief.renderRGBImage();

	cv::Mat delta;
	cv::absdiff(target, render, delta);

	const auto ds = cv::sum(delta);
	const float norm_l1 = ds[0] + ds[1] + ds[2];

	return norm_l1 / (ds.rows * ds.cols);
}

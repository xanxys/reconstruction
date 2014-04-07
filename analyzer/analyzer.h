#pragma once

#include <memory>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>

#include "scene_belief.h"

// Analyze a single RGB-D frame.
// Camera is always at the origin.
class SceneAnalyzer {
public:
	SceneAnalyzer(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud);
	std::shared_ptr<SceneBelief> getBestBelief();
	std::vector<std::shared_ptr<SceneBelief>> getAllBelief();

	// Average L1 norm
	// 0:complete match
	static float getScore(const SceneBelief& belief);
private:
	FrameBelief frame;
};

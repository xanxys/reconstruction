#pragma once

#include <memory>
#include <sstream>

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
private:
	FrameBelief frame;
};

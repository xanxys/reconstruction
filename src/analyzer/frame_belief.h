#pragma once

#include <sstream>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Intrinsic parameters of RGB camera.
class FrameBelief {
public:
	FrameBelief(const FrameBelief& that);
	FrameBelief(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud);

	cv::Mat extractImage();
	cv::Mat extractDepthImage();
public:
	// Put this before all other members to initialize first,
	// since logging is used in SceneAnalyzer's initializer's list.
	mutable std::ostringstream log;

	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud;
	Eigen::Vector3f camera_pos;
	Eigen::Vector2f camera_center;
	float camera_fl;
private:
	
};

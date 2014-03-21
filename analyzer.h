#pragma once

#include <Eigen/Dense>
#include <jsoncpp/json/json.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>


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

enum class VoxelState {
	OCCUPIED,
	EMPTY
};


class VoxelDescription {
public:
	VoxelDescription();
public:
	VoxelState state;
	Eigen::Vector3f average_image_color;
};

// Analyze a single RGB-D frame.
// Camera is always at the origin.
class SceneAnalyzer {
public:
	SceneAnalyzer(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud);

	// Return aligned cloud.
	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr getCloud();

	float getFloor();

	cv::Mat getRGBImage();
	std::map<std::tuple<int, int, int>, VoxelState> getVoxels();
	std::map<std::tuple<int, int, int>, VoxelDescription> getVoxelsDetailed();
	Json::Value getObjects();
protected:
	static pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr align(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud);
	static cv::Mat extractImageFromPointCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud);
protected:
	const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud;
private:
	const float voxel_size;
};

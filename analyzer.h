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

// Finite plane (quad) with texture.
// x: [-size/2,size/2]
// y: y_offset
// z: [-size/2,size/2]
class TexturedPlane {
public:
	TexturedPlane(float size, cv::Mat texture, float y_offset);
public:
	cv::Mat texture;
	float y_offset;
	const float size;
};


// Analyze a single RGB-D frame.
// Camera is always at the origin.
class SceneAnalyzer {
public:
	SceneAnalyzer(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud);

	std::vector<TexturedPlane> getPlanes();
	
	// Return aligned cloud.
	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr getCloud();

	float getFloor();

	cv::Mat getRGBImage();
	cv::Mat renderRGBImage();
	std::map<std::tuple<int, int, int>, VoxelState> getVoxels();
	std::map<std::tuple<int, int, int>, VoxelDescription> getVoxelsDetailed();

	// TODO: Define a new box class and use it.
	Json::Value getObjects();
protected:
	// Synthesize complete texture from RGB image and unreliable mask.
	static cv::Mat synthesizeTexture(const cv::Mat image, const cv::Mat mask);
	static cv::Mat growTexture(const cv::Mat core, int width, int height);

	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr align(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud);
	static cv::Mat extractImageFromPointCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud);
protected:
	const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud;
	Eigen::Matrix3f camera_loc_to_world;
private:
	const float voxel_size;
};

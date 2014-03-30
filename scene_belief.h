#pragma once

#include <sstream>

#include <Eigen/Dense>
// TODO: SceneBelief shouldn't depend on serialization format.
#include <jsoncpp/json/json.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>

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


// A coherent set of belief about the scene, which may or may not be
// visible. It's a node of search tree.
class SceneBelief {
public:
	SceneBelief(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud);

	std::string getLog();
	
	std::vector<TexturedPlane> getPlanes();
	
	// Return aligned cloud.
	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr getCloud();

	Eigen::Matrix3f getCameraLocalToWorld();

	float getFloor();

	cv::Mat getRGBImage();
	// return meter in f32
	cv::Mat getDepthImage();

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
	static cv::Mat extractDepthImageFromPointCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud);
protected:
	// Put this before all other members to initialize first,
	// since logging is used in SceneAnalyzer's initializer's list.
	std::ostringstream log;

	const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud;
	Eigen::Matrix3f camera_loc_to_world;
private:
	const float voxel_size;
};

#pragma once

#include <sstream>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>


enum class Direction {
	XP,
	XN,
	YP,
	YN,
	ZP,
	ZN
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
// It's an interface between known and unknown regions of the scene.
// (walls etc.), and NOT part of objects like box.
//
// x: [-size/2,size/2]
// y: y_offset
// z: [-size/2,size/2]
class TexturedPlane {
public:
	TexturedPlane(float size, cv::Mat texture, float y_offset);
public:
	Direction normal;
	Eigen::Vector3f center;
	cv::Mat texture;
	const float size;
};


class OrientedBox {
public:
	OrientedBox(
		Eigen::Vector3f position,
		float ry,
		Eigen::Vector3f size,
		Eigen::Vector3f color,
		bool valid);

	Eigen::Vector3f getPosition() const;
	Eigen::Vector3f getSize() const;
	Eigen::Vector3f getColor() const;
	float getRotationY() const;
	bool getValid() const;
private:
	float ry;

	Eigen::Vector3f position;
	Eigen::Vector3f size;
	Eigen::Vector3f color;
	bool valid;
};


// A coherent set of belief about the scene, which may or may not be
// visible. It's a node of search tree.
class SceneBelief {
public:
	SceneBelief(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud);
	SceneBelief(
		const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud,
		Eigen::Matrix3f camera_loc_to_world);


	// tree exploration
	std::vector<std::shared_ptr<SceneBelief>> expandByAlignment();

	// attribs
	std::string getLog();
	std::vector<TexturedPlane> getPlanes();
	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr getCloud();
	Eigen::Matrix3f getCameraLocalToWorld();
	float getFloor();

	cv::Mat getRGBImage();
	// return meter in f32
	cv::Mat getDepthImage();

	cv::Mat renderRGBImage();
	std::map<std::tuple<int, int, int>, VoxelState> getVoxels();
	std::map<std::tuple<int, int, int>, VoxelDescription> getVoxelsDetailed();

	std::vector<OrientedBox> getObjects();

	Eigen::Vector2f projectToRGBCameraScreen(Eigen::Vector3f pos_world);
protected:
	// Extract textured plane from a plane with Manhattan normal.
	// index, distance means the coordinate of dir.
	// (e.g. XN,XP -> index, distance is x coord of the plane)
	TexturedPlane extractPlane(int index, float coord, Direction dir);

	// Synthesize complete texture from RGB image and unreliable mask.
	static cv::Mat synthesizeTexture(const cv::Mat image, const cv::Mat mask);
	static cv::Mat growTexture(const cv::Mat core, int width, int height);

	std::shared_ptr<SceneBelief> align();

	static cv::Mat extractImageFromPointCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud);
	static cv::Mat extractDepthImageFromPointCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud);
protected:
	// Put this before all other members to initialize first,
	// since logging is used in SceneAnalyzer's initializer's list.
	std::ostringstream log;

	const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud;
	const Eigen::Matrix3f camera_loc_to_world;
	const Eigen::Matrix3f world_to_camera_loc;

	// belief about the RGB image.
	const Eigen::Vector3f camera_pos;
	const Eigen::Vector2f camera_center;
	const float camera_fl;
private:
	const float voxel_size;
};

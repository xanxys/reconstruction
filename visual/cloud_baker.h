#pragma once

#include <map>
#include <tuple>

#include <Eigen/Dense>
#include <jsoncpp/json/json.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <asset.pb.h>
#include <logging.h>
#include <visual/dense_voxel.h>
#include <visual/textured_mesh.h>
#include <visual/triangle_mesh.h>

namespace visual {

enum class VoxelState {
	OCCUPIED,
	EMPTY
};

enum class RoomVoxel {
	EMPTY,
	EXTERIOR,
	INTERIOR,
	UNKNOWN
};

const std::map<RoomVoxel, std::string> RoomVoxelString = {
	{RoomVoxel::EMPTY, "EMPTY"},
	{RoomVoxel::EXTERIOR, "EXTERIOR"},
	{RoomVoxel::INTERIOR, "INTERIOR"},
	{RoomVoxel::UNKNOWN, "UNKNOWN"}
};


class VoxelDescription {
public:
	VoxelDescription();
public:
	bool guess;
	VoxelState state;
	Eigen::Vector3f average_image_color;
};


class Voxelizer {
public:
	Voxelizer(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float voxel_size);
	std::map<std::tuple<int, int, int>, VoxelDescription> getVoxelsDetailed() const;
	std::map<std::tuple<int, int, int>, VoxelDescription> getVoxelsDetailedWithoutGuess() const;
private:
	float voxel_size;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
};


// CloudBaker takes a single point cloud and generates
// textured 3D mesh as asset.
class CloudBaker {
public:
	// Convert json-style XYZRGB point cloud.
	CloudBaker(const Json::Value& cloud);

	TexturedMesh generateRoomMesh();
private:
	void loadFromJson(const Json::Value& cloud);
	static Json::Value showVec3i(const Eigen::Vector3i& v);

	static void logVoxelCounts(const DenseVoxel<RoomVoxel>& dv);
	static void estimateUnknownCells(DenseVoxel<RoomVoxel>& dv);

	// Fill pixels with value = undefined with approximately nearest
	// colors. Does nothing if all pixel = undefined.
	// iteration = approx min distance of value propagation
	//
	// worst time: O(iteration * number of pixels)
	static void fillHoles(cv::Mat& image, cv::Vec3b undefined, int iteration);
	static pcl::PointCloud<pcl::PointXYZ>::Ptr decolor(const pcl::PointCloud<pcl::PointXYZRGB>& cloud);
private:
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
	TriangleMesh<Eigen::Vector2f> exterior_mesh;
};

}  // namespace

#pragma once

#include <map>
#include <tuple>

#include <boost/optional.hpp>
#include <Eigen/Dense>
#include <jsoncpp/json/json.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <logging.h>
#include <visual/textured_mesh.h>

namespace visual {

// A complete information to be loaded by EqExperiment/LoaderPlugin.
class SceneAssetBundle {
public:
	// Put bunch of files into specified directory (newly created).
	// Behavior is undefined when the directory already exists.
	// TODO: erase it & re-create. just-like a single file.
	// The directory might be nested.
	void serializeIntoDirectory(std::string dir_path) const;
private:
	// Serialize relative small data that nicely fits into a JSON
	// (e.g. <100MB).
	// Don't store image-like things or huge triangle mesh here.
	Json::Value serializeSmallData() const;
	TriangleMesh<Eigen::Vector3f> serializeDebugPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) const;
public:
	TexturedMesh exterior_mesh;
	std::vector<Eigen::Vector3f> point_lights;
	std::vector<TexturedMesh> interior_objects;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr debug_points_interior;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr debug_points_interior_distance;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr debug_points_merged;
};


// Immutable data that holds a single scan.
class SingleScan {
public:
	SingleScan(Json::Value& old_style);

	SingleScan(const std::string& path, float pre_rotation = 0);
public:
	// TODO: make this immutable.
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_w_normal;

	// Hack to make sparse ICP work.
	// TODO: guess this internally and remove this field,
	// or replace with magnetometer measurement.
	const float pre_rotation;

	cv::Mat er_rgb;
	cv::Mat er_depth;
	cv::Mat er_intensity;
};


// All scans succesfully aligned.
class AlignedScans {
public:
	// Create aligned scan from unaligned scans. (slow, takes a few minutes)
	AlignedScans(const std::vector<SingleScan>& scans);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getMergedPoints() const;
	std::vector<std::pair<SingleScan, Eigen::Affine3f>> getScansWithPose() const;
private:
	// Calculate a rough transform from source to target by heuristics.
	static Eigen::Affine3f prealign(const SingleScan& target, const SingleScan& source);

	// Mkae large near-horizontal planar segment (most likely ceiling) completely level
	// by slight rotation.
	void applyLeveling();
private:
	// [(original scan, local_to_world)]
	std::vector<std::pair<SingleScan, Eigen::Affine3f>> scans_with_pose;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged;
};


std::vector<Eigen::Vector3f> recognize_lights(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

// Try avoiding classes for this kind of complex, pure operation.
namespace scene_recognizer {

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr applyTransform(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, const Eigen::Affine3f& trans);

template<typename Scalar>
Eigen::Vector3f append(Eigen::Vector2f v, Scalar x) {
	return Eigen::Vector3f(v(0), v(1), x);
}

TexturedMesh bakeTexture(const AlignedScans& scans, const TriangleMesh<std::nullptr_t>& shape);

// Downsample using grid filter (leave one point per voxel).
template<typename Point>
typename pcl::PointCloud<Point>::Ptr downsample(typename pcl::PointCloud<Point>::Ptr cloud, float grid_size=0.05) {
	// voxel index -> ([pos], [normal])
	std::map<
		std::tuple<int, int, int>,
		std::tuple<std::vector<Eigen::Vector3f>, std::vector<Eigen::Vector3f>>> tiles;
	for(const auto& pt_xyz : cloud->points) {
		const Eigen::Vector3f pt = pt_xyz.getVector3fMap();
		const auto pt_i = (pt / grid_size).cast<int>();
		const auto tix = std::make_tuple(pt_i(0), pt_i(1), pt_i(2));
		std::get<0>(tiles[tix]).push_back(pt);
		std::get<1>(tiles[tix]).push_back(pt_xyz.getNormalVector3fMap());
	}
	typename pcl::PointCloud<Point>::Ptr cloud_new(new pcl::PointCloud<Point>);
	for(const auto& tile : tiles) {
		Point pt;
		pt.getVector3fMap() = std::accumulate(
			std::get<0>(tile.second).begin(), std::get<0>(tile.second).end(),
			Eigen::Vector3f(0, 0, 0)) / std::get<0>(tile.second).size();
		pt.getNormalVector3fMap() = std::accumulate(
			std::get<1>(tile.second).begin(), std::get<1>(tile.second).end(),
			Eigen::Vector3f(0, 0, 0)) / std::get<1>(tile.second).size();
		cloud_new->points.push_back(pt);
	}
	return cloud_new;
}

// Calculate distance between two point clouds by
// RGB and normal similarity.
float cloudDistance(
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr c1,
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr c2);

// Calculate ranges (both ends of box) on wall polygon by analyzing interior point cloud.
// Indices of polygon vertices are used to indicate position on the primeter.
std::vector<std::pair<int, int>> decomposeWallBoxes(
	pcl::PointCloud<pcl::PointXYZ>::Ptr interior_cloud,
	const std::vector<Eigen::Vector2f>& polygon);

// Create a geometry (box) for given ticks, if there's actually box.
boost::optional<TexturedMesh> createWallBox(
	const std::vector<Eigen::Vector2f>& polygon,
	std::pair<float, float> z_range,
	std::pair<int, int> ticks,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

// Takes several scans of a single room as input (in unordered way),
// and generates a SceneAsssetBundle.
SceneAssetBundle recognizeScene(const std::vector<SingleScan>& scans);

pcl::PointCloud<pcl::PointXYZ>::Ptr decolor(const pcl::PointCloud<pcl::PointXYZRGB>& cloud);
//pcl::PointCloud<pcl::PointXYZNormal>::Ptr decolor(const pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud);

}  // namespace

}  // namespace

#pragma once

#include <map>
#include <string>
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
// Also contains bunch of data for debugging purpose.
// TODO: might need option to turn throw away debug info, if data
// gets too large.
class SceneAssetBundle {
public:
	// Initialize empty data with output directory.
	// The output directory will be filled by files as data
	// become available.
	SceneAssetBundle(std::string dir_path);
	// open with existing incomplete asset bundle.
	SceneAssetBundle(std::string dir_path, int count_start);
	~SceneAssetBundle();

	void addDebugPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
	void addDebugPointCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);

	void addDebugPointCloud(std::string name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
	void addDebugPointCloud(std::string name, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);

	void addMesh(std::string name, const TriangleMesh<std::nullptr_t>& mesh);

	Json::Value loadJson(std::string name) const;
private:
	void recreateDirectory(std::string dir_path) const;

	// Put bunch of files into specified directory.
	// Directory must exist and be empty.
	// Behavior is undefined when the directory already exists.
	// The directory might be nested.
	void serializeIntoDirectory(std::string dir_path) const;

	// Serialize relative small data that nicely fits into a JSON
	// (e.g. <100MB).
	// Don't store image-like things or huge triangle mesh here.
	Json::Value serializeSmallData() const;
	TriangleMesh<Eigen::Vector3f> serializeDebugPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) const;
	TriangleMesh<std::tuple<Eigen::Vector3f, Eigen::Vector3f>> serializeDebugPoints(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud) const;
public:
	TexturedMesh exterior_mesh;
	std::vector<Eigen::Vector3f> point_lights;
	std::vector<TexturedMesh> interior_objects;
private:
	int debug_count;
	const std::string dir_path;
};

}  // namespace

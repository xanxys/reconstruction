#pragma once

#include <map>
#include <tuple>

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
public:
	TexturedMesh exterior_mesh;
	std::vector<Eigen::Vector3f> point_lights;
};


class SingleScan {
public:
	SingleScan(Json::Value& old_style);
	SingleScan(const std::string& path);
public:
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
};

std::vector<Eigen::Vector3f> recognize_lights(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

// Try avoiding classes for this kind of complex, pure operation.
namespace scene_recognizer {

// Takes several scans of a single room as input (in unordered way),
// and generates a SceneAsssetBundle.
SceneAssetBundle recognizeScene(const std::vector<SingleScan>& scans);

pcl::PointCloud<pcl::PointXYZ>::Ptr decolor(const pcl::PointCloud<pcl::PointXYZRGB>& cloud);

}  // namespace

}  // namespace
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
#include <visual/aligned_scan.h>
#include <visual/scene_asset_bundle.h>
#include <visual/textured_mesh.h>

namespace visual {

std::vector<Eigen::Vector3f> recognize_lights(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

// Try avoiding classes for this kind of complex, pure operation.
namespace scene_recognizer {

TexturedMesh bakeTexture(const AlignedScans& scans, const TriangleMesh<std::nullptr_t>& shape);

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
// and populate given SceneAsssetBundle.
void recognizeScene(SceneAssetBundle& bundle, const std::vector<SingleScan>& scans);

// Second pass of scene recognition, after ./extract_shape.py is applied.
// Create objects.
void recognizeScene2(SceneAssetBundle& bundle, const std::vector<SingleScan>& scans);

void splitObjects(
	SceneAssetBundle& bundle,
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_org,
	const AlignedScans& scans);

pcl::PointCloud<pcl::PointXYZ>::Ptr decolor(const pcl::PointCloud<pcl::PointXYZRGB>& cloud);
//pcl::PointCloud<pcl::PointXYZNormal>::Ptr decolor(const pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud);

}  // namespace

}  // namespace

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
namespace cloud_baker {

// Assign color to point cloud based on distance from mesh.
pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorPointsByDistance(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
	TriangleMesh<std::nullptr_t> shape,
	bool dont_color);

// A convenient version where UV coordinates are automatically assigned.
TexturedMesh bakePointsToMesh(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
	TriangleMesh<std::nullptr_t> shape);

// Project points to given mesh shape, and
// return mesh with interpolated texture.
TexturedMesh bakePointsToMesh(
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
	TriangleMesh<Eigen::Vector2f> shape);

// Fill pixels with value = undefined with approximately nearest
// colors. Does nothing if all pixel = undefined.
// iteration = approx min distance of value propagation
//
// worst time: O(iteration * number of pixels)
void fillHoles(cv::Mat& image, cv::Vec3b undefined, int iteration);

}  // namespace
}  // namespace

#pragma once

#include <map>
#include <tuple>

#include <Eigen/Dense>
#include <jsoncpp/json/json.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <logging.h>
#include <visual/cloud_conversion.h>
#include <visual/dense_voxel.h>
#include <visual/textured_mesh.h>
#include <visual/texture_conversion.h>
#include <visual/triangle_mesh.h>
#include <visual/mapping.h>

namespace visual {
namespace cloud_baker {

// Assign color to point cloud based on distance from mesh.
// Point type must have RGB and XYZ and Normal.
template<typename Point>
typename pcl::PointCloud<Point>::Ptr colorPointsByDistance(
		typename pcl::PointCloud<Point>::Ptr cloud,
		TriangleMesh<std::nullptr_t> shape,
		bool dont_color) {
	const auto mesh_uv = mapSecond(assignUV(shape));

	typename pcl::PointCloud<Point>::Ptr cloud_new(new pcl::PointCloud<Point>);
	for(auto point : cloud->points) {
		const Eigen::Vector3f pos = point.getVector3fMap();
		const auto dist_and_uv = nearestCoordinateWithNormal(mesh_uv, pos);
		const float dist = std::get<0>(dist_and_uv);
		const auto normal = std::get<2>(dist_and_uv);
		// inside == very far from walls
		// OR somewhat far from walls and have very different normals
		if(dist > 0.2 || (dist > 0.05 && normal.dot(point.getNormalVector3fMap()) < 0.5)) {
			if(!dont_color) {
				point.r = dist * 255;
				point.g = dist * 255;
				point.b = dist * 255;
			}
			cloud_new->points.push_back(point);
		}
	}
	return cloud_new;
}

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

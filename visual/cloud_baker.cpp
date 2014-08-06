#include "cloud_baker.h"

#include <array>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <set>

#include <boost/filesystem.hpp>
#include <boost/range/irange.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <analyzer/voxel_traversal.h>
#include <logging.h>
#include <range2.h>
#include <visual/cloud_conversion.h>
#include <visual/mapping.h>
#include <visual/marching_cubes.h>
#include <visual/shape_fitter.h>
#include <visual/texture_conversion.h>
#include <visual/voxel_conversion.h>

namespace visual {
namespace cloud_baker {

pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorPointsByDistance(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		TriangleMesh<std::nullptr_t> shape,
		bool dont_color) {
	const auto mesh_uv = mapSecond(assignUV(shape));

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_new(new pcl::PointCloud<pcl::PointXYZRGB>);
	for(auto point : cloud->points) {
		const Eigen::Vector3f pos = point.getVector3fMap();
		const auto dist_and_uv = nearestCoordinate(mesh_uv, pos);
		const float dist = dist_and_uv.first;
		if(dist > 0.2) {
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


TexturedMesh bakePointsToMesh(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		TriangleMesh<std::nullptr_t> shape) {
	return bakePointsToMesh(cloud, mapSecond(assignUV(shape)));
}

TexturedMesh bakePointsToMesh(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		TriangleMesh<Eigen::Vector2f> mesh_uv) {
	// Project points to the surface and draw circles onto texture.
	const int tex_size = 2048;
	const float thresh_distance = 0.5;
	cv::Mat diffuse(tex_size, tex_size, CV_8UC3);
	diffuse = cv::Scalar(0, 0, 0);
	for(const auto& point : cloud->points) {
		const Eigen::Vector3f pos = point.getVector3fMap();
		const auto dist_and_uv = nearestCoordinate(mesh_uv, pos);
		// Ignore points too far from exterior.
		if(dist_and_uv.first < thresh_distance) {
			const cv::Scalar color(point.b, point.g, point.r);
			cv::circle(
				diffuse, eigenToCV(swapY(dist_and_uv.second) * tex_size), 1,
				color, -1);
		}
	}
	fillHoles(diffuse, cv::Vec3b(0, 0, 0), 5);

	TexturedMesh tm;
	tm.diffuse = diffuse;
	tm.mesh = mesh_uv;
	return tm;
}

void fillHoles(cv::Mat& image, const cv::Vec3b undefined, int iteration) {
	const Eigen::Vector2i imageMin(0, 0);
	const Eigen::Vector2i imageMax(image.cols, image.rows);
	for(int step : boost::irange(0, iteration)) {
		bool propagation_happened = false;
		for(auto pos : range2(imageMin, imageMax)) {
			auto& current_pixel = image.at<cv::Vec3b>(pos.y(), pos.x());
			if(current_pixel != undefined) {
				continue;
			}
			// Copy defined neighbor color.
			// (Search range contains itself, but it's ok because it's undefined thus ignored)
			for(auto pos_search : range2(
				(pos - Eigen::Vector2i(1, 1)).cwiseMax(imageMin),
				(pos + Eigen::Vector2i(2, 2)).cwiseMin(imageMax))) {
				const auto candidate = image.at<cv::Vec3b>(pos_search.y(), pos_search.x());
				if(candidate != undefined) {
					current_pixel = candidate;
					propagation_happened = true;
					break;
				}
			}
		}
		// all pixels are filled, or image == undefined
		if(!propagation_happened) {
			break;
		}
	}
}


}  // namespace
}  // namespace

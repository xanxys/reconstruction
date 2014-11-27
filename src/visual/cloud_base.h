#pragma once

#include <map>
#include <tuple>

#include <boost/optional.hpp>
#include <Eigen/Dense>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <logging.h>

namespace recon {

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr applyTransform(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, const Eigen::Affine3f& trans);

template<typename Scalar>
Eigen::Vector3f append(Eigen::Vector2f v, Scalar x) {
	return Eigen::Vector3f(v(0), v(1), x);
}

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

template<typename Point>
typename pcl::PointCloud<Point>::Ptr merge(typename pcl::PointCloud<Point>::Ptr c1, typename pcl::PointCloud<Point>::Ptr c2) {
	typename pcl::PointCloud<Point>::Ptr cloud(new pcl::PointCloud<Point>());
	for(const auto& pt : c1->points) {
		cloud->points.push_back(pt);
	}
	for(const auto& pt : c2->points) {
		cloud->points.push_back(pt);
	}
	return cloud;
}

// Calculate distance between two point clouds by
// RGB and normal similarity.
float cloudDistance(
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr c1,
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr c2);

template<typename PointIn, typename PointOut>
typename pcl::PointCloud<PointOut>::Ptr cast(typename pcl::PointCloud<PointIn>::Ptr cloud) {
	typename pcl::PointCloud<PointOut>::Ptr cloud_out(new pcl::PointCloud<PointOut>);
	pcl::copyPointCloud(*cloud, *cloud_out);
	return cloud_out;
}

}  // namespace

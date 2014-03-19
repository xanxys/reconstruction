#include "analyzer.h"

#include <map>
#include <cmath>
#include <exception>
#include <iostream>
#include <sstream>
#include <random>

#include <boost/range/irange.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using ColorCloud = pcl::PointCloud<pcl::PointXYZRGBA>;
using RigidTrans3f = Eigen::Transform<float, 3, Eigen::AffineCompact>;

const double pi = 3.14159265359;

VoxelTraversal::VoxelTraversal(float size, Eigen::Vector3f org, Eigen::Vector3f dir) :
org(org / size), dir(dir) {
	index = Eigen::Vector3i(
		std::floor(org(0)),
		std::floor(org(1)),
		std::floor(org(2)));

	frac = org - index.cast<float>();
}

std::tuple<int, int, int> VoxelTraversal::next() {
	const auto key = std::make_tuple(index(0), index(1), index(2));

	const auto remaining = Eigen::Vector3f(
		(dir(0) < 0) ? -frac(0) : 1 - frac(0),
		(dir(1) < 0) ? -frac(1) : 1 - frac(1),
		(dir(2) < 0) ? -frac(2) : 1 - frac(2));

	const auto dt_xyz = remaining.cwiseQuotient(dir);

	// Select the direction with smallest dt. (tie-breaker: prefer X>Y>Z)
	if(dt_xyz(0) <= dt_xyz(1) && dt_xyz(0) <= dt_xyz(2)) {
		const int dix = (dir(0) < 0) ? -1 : 1;
		index(0) += dix;
		frac += dt_xyz(0) * dir;
		frac(0) -= dix;
	} else if(dt_xyz(1) <= dt_xyz(2)) {
		const int dix = (dir(1) < 0) ? -1 : 1;
		index(1) += dix;
		frac += dt_xyz(1) * dir;
		frac(1) -= dix;
	} else {
		const int dix = (dir(2) < 0) ? -1 : 1;
		index(2) += dix;
		frac += dt_xyz(2) * dir;
		frac(2) -= dix;
	}

	return key;
}


SceneAnalyzer::SceneAnalyzer(const ColorCloud::ConstPtr& cloud) :
	cloud(cloud), voxel_size(0.1) {
}

cv::Mat SceneAnalyzer::getRGBImage() {
	return extractImageFromPointCloud(cloud);
}

pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr SceneAnalyzer::getCloud() {
	return cloud;
}

std::map<std::tuple<int, int, int>, VoxelState> SceneAnalyzer::getVoxels() {
	const float size = voxel_size;

	// known to be filled
	std::map<std::tuple<int, int, int>, bool> voxels;
	for(const auto& pt : cloud->points) {
		if(!std::isfinite(pt.x)) {
			continue;
		}

		auto ix = pt.getVector3fMap() / size;
		auto key = std::make_tuple(
			static_cast<int>(std::floor(ix.x())),
			static_cast<int>(std::floor(ix.y())),
			static_cast<int>(std::floor(ix.z())));
		voxels[key] = true;
	}

	const auto& camera_origin = Eigen::Vector3f::Zero();

	std::map<std::tuple<int, int, int>, bool> voxels_empty;
	for(const auto& pair_filled : voxels) {
		// cast ray from camera
		const auto pos = Eigen::Vector3f(
			std::get<0>(pair_filled.first) + 0.5,
			std::get<1>(pair_filled.first) + 0.5,
			std::get<2>(pair_filled.first) + 0.5) * size;

		const auto dir = (pos - camera_origin).normalized();

		// traverse until hit.
		VoxelTraversal traversal(size, camera_origin, dir);
		for(int i : boost::irange(0, 100)) {
			const auto key = traversal.next();

			// Hit wall.
			if(voxels.find(key) != voxels.end()) {
				break;
			}

			voxels_empty[key] = true;
		}
	}

	std::map<std::tuple<int, int, int>, bool> voxels_unknown;
	for(int ix : boost::irange(-12, 12)) {
		for(int iy : boost::irange(-12, 12)) {
			for(int iz : boost::irange(10, 35)) {
				const auto key = std::make_tuple(ix, iy, iz);

				if(voxels.find(key) == voxels.end() && voxels_empty.find(key) == voxels_empty.end()) {
					voxels_unknown[key] = true;
				}
			}
		}
	}

	std::map<std::tuple<int, int, int>, VoxelState> voxel_merged;
	for(const auto& pair_filled : voxels) {
		voxel_merged[pair_filled.first] = VoxelState::OCCUPIED;
	}
	for(const auto& pair_empty : voxels_empty) {
		voxel_merged[pair_empty.first] = VoxelState::EMPTY;
	}
	return voxel_merged;
}

Json::Value SceneAnalyzer::getObjects() {
	const auto voxels = getVoxels();

	// find floor
	int iy_floor = 0;
	for(const auto& pair : voxels) {
		if(pair.second == VoxelState::OCCUPIED) {
			iy_floor = std::max(iy_floor, std::get<1>(pair.first));
		}
	}
	const float y_floor = (iy_floor + 1) * voxel_size;

	//
	Json::Value objects;

	std::mt19937 gen;
	for(int i : boost::irange(0, 1000)) {
		// Generate box params
		const float height = std::uniform_real_distribution<float>(0.05, 2)(gen);

		const Eigen::Vector3f box_center(
			std::uniform_real_distribution<float>(-1.25, 1.25)(gen),
			y_floor - height / 2,
			std::uniform_real_distribution<float>(0, 3)(gen));

		const float rot_y = std::uniform_real_distribution<float>(0, 2 * pi)(gen);

		const Eigen::Vector3f box_size(
			std::uniform_real_distribution<float>(0.05, 2)(gen),
			height,
			std::uniform_real_distribution<float>(0.05, 2)(gen));

		Json::Value object;
		object["px"] = box_center.x();
		object["py"] = box_center.y();
		object["pz"] = box_center.z();
		object["ry"] = rot_y;
		object["sx"] = box_size.x();
		object["sy"] = box_size.y();
		object["sz"] = box_size.z();

		bool collision = false;
		for(const auto& pair : voxels) {
			if(pair.second == VoxelState::EMPTY) {
				const auto voxel_center = Eigen::Vector3f(
					0.5 + std::get<0>(pair.first),
					0.5 + std::get<1>(pair.first),
					0.5 + std::get<2>(pair.first)) * voxel_size;

				const auto dp = Eigen::AngleAxisf(-rot_y, Eigen::Vector3f::UnitY()) * (voxel_center - box_center);

				if((dp.array() > (-box_size / 2).array()).all() &&
					(dp.array() < (box_size / 2).array()).all()) {
					collision = true;
				}
			}
		}

		object["valid"] = !collision;

		objects.append(object);
	}

	return objects;
}

cv::Mat SceneAnalyzer::extractImageFromPointCloud(
	const ColorCloud::ConstPtr& cloud) {
	if(cloud->points.size() != cloud->width * cloud->height) {
		throw std::runtime_error("Point cloud is not an image");
	}

	cv::Mat rgb(cloud->height, cloud->width, CV_8UC3);
	for(int y : boost::irange(0, (int)cloud->height)) {
		for(int x : boost::irange(0, (int)cloud->width)) {
			const pcl::PointXYZRGBA& pt = cloud->points[y * cloud->width + x];
			rgb.at<cv::Vec3b>(y, x) = cv::Vec3b(pt.b, pt.g, pt.r);
		}
	}
	return rgb;
}

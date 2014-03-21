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


VoxelDescription::VoxelDescription() : average_image_color(0, 0, 0) {

}


SceneAnalyzer::SceneAnalyzer(const ColorCloud::ConstPtr& raw_cloud) :
	cloud(align(raw_cloud)), voxel_size(0.1) {
	assert(cloud);
}

ColorCloud::ConstPtr SceneAnalyzer::align(const ColorCloud::ConstPtr& cloud) {
	// Detect the primary plane.
	pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01);

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);

	const auto normal = Eigen::Vector3f(
		coefficients->values[0],
		coefficients->values[1],
		coefficients->values[2]).normalized();

	const Eigen::Vector3f up(0, -1, 0);

	Eigen::Matrix3f rotation(Eigen::Matrix3f::Identity());
	if(normal.dot(up) >= std::cos(pi / 4)) {
		// the plane is likely to be floor
		std::cout << "Correcting w/ floor" << std::endl;
		const auto axis = up.cross(normal);

		rotation = Eigen::AngleAxisf(
			-std::asin(axis.norm()),
			axis.normalized());

		// TODO: adjust Y-rotation
	} else {
		// the plane is likely to be wall
		std::cout << "Correcting w/ wall" << std::endl;


	}

	// apply rotation.
	ColorCloud::Ptr cloud_aligned(new ColorCloud());
	for(auto pt : cloud->points) {
		pt.getVector3fMap() = rotation * pt.getVector3fMap();
		cloud_aligned->points.push_back(pt);
	}
	cloud_aligned->width = 640;
	cloud_aligned->height = 480;
	return cloud_aligned;
}

cv::Mat SceneAnalyzer::getRGBImage() {
	return extractImageFromPointCloud(cloud);
}

pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr SceneAnalyzer::getCloud() {
	assert(cloud);
	return cloud;
}

std::map<std::tuple<int, int, int>, VoxelState> SceneAnalyzer::getVoxels() {
	std::map<std::tuple<int, int, int>, VoxelState> voxels;
	for(const auto& pair : getVoxelsDetailed()) {
		voxels[pair.first] = pair.second.state;
	}
	return voxels;
}

std::map<std::tuple<int, int, int>, VoxelDescription> SceneAnalyzer::getVoxelsDetailed() {
	const float size = voxel_size;

	// known to be filled
	std::map<std::tuple<int, int, int>, bool> voxels;
	std::map<std::tuple<int, int, int>, Eigen::Vector3f> voxels_accum;
	std::map<std::tuple<int, int, int>, int> voxels_count;
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
		if(voxels_accum.find(key) == voxels_accum.end()) {
			voxels_accum[key] = Eigen::Vector3f(pt.r, pt.g, pt.b);
			voxels_count[key] = 1;
		} else {
			voxels_accum[key] += Eigen::Vector3f(pt.r, pt.g, pt.b);
			voxels_count[key] += 1;
		}
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

	std::map<std::tuple<int, int, int>, VoxelDescription> voxel_merged;
	for(const auto& pair_filled : voxels) {
		VoxelDescription desc;
		desc.state = VoxelState::OCCUPIED;
		desc.average_image_color =
		voxels_accum[pair_filled.first] / voxels_count[pair_filled.first];
		voxel_merged[pair_filled.first] = desc;
	}
	for(const auto& pair_empty : voxels_empty) {
		VoxelDescription desc;
		desc.state = VoxelState::EMPTY;
		voxel_merged[pair_empty.first] = desc;
	}
	return voxel_merged;
}

Json::Value SceneAnalyzer::getObjects() {
	Json::Value objects;

	const auto voxels = getVoxelsDetailed();

	// find floor
	int iy_floor = 0;
	for(const auto& pair : voxels) {
		if(pair.second.state == VoxelState::OCCUPIED) {
			iy_floor = std::max(iy_floor, std::get<1>(pair.first));
		}
	}
	const float y_floor = (iy_floor + 1) * voxel_size;

	// find walls by 3D -> 2D projection
	std::map<std::tuple<int, int>, int> projected;
	for(const auto& pair : voxels) {
		if(pair.second.state == VoxelState::OCCUPIED) {
			const auto key = std::make_tuple(
				std::get<0>(pair.first),
				std::get<2>(pair.first));

			projected[key] += 1;
		}
	}

	// find x limits
	int ix0 = 0;
	int ix1 = 0;
	for(const auto& pair : projected) {
		ix0 = std::min(ix0, std::get<0>(pair.first));
		ix1 = std::max(ix1, std::get<0>(pair.first));
	}

	{
		Json::Value object;
		object["px"] = (ix0 + 0.5) * voxel_size;
		object["py"] = 0;
		object["pz"] = 2;
		object["ry"] = 0;
		object["sx"] = 0.03;
		object["sy"] = 3;
		object["sz"] = 4;
		object["valid"] = true;
		object["r"] = 0;
		object["g"] = 255;
		object["b"] = 0;
		objects.append(object);
	}
	{
		Json::Value object;
		object["px"] = (ix1 + 0.5) * voxel_size;
		object["py"] = 0;
		object["pz"] = 2;
		object["ry"] = 0;
		object["sx"] = 0.03;
		object["sy"] = 3;
		object["sz"] = 4;
		object["valid"] = true;
		object["r"] = 0;
		object["g"] = 255;
		object["b"] = 0;
		objects.append(object);
	}

	//
	

	for(const auto& wall_tile : projected) {
		if(wall_tile.second < 5) {
			continue;
		}

		Json::Value object;
		object["px"] = (std::get<0>(wall_tile.first) + 0.5) * voxel_size;
		object["py"] = 0;
		object["pz"] = (std::get<1>(wall_tile.first) + 0.5) * voxel_size;
		object["ry"] = 0;
		object["sx"] = 0.03;
		object["sy"] = 3;
		object["sz"] = 0.03;
		object["valid"] = true;
		object["r"] = 0;
		object["g"] = 0;
		object["b"] = 255;

		objects.append(object);
	}

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

		bool collision_empty = false;
		bool collision_occupied = false;
		Eigen::Vector3f avg_color(0, 0, 0);
		int avg_count = 0;
		for(const auto& pair : voxels) {
			const auto voxel_center = Eigen::Vector3f(
				0.5 + std::get<0>(pair.first),
				0.5 + std::get<1>(pair.first),
				0.5 + std::get<2>(pair.first)) * voxel_size;

			const auto dp = Eigen::AngleAxisf(-rot_y, Eigen::Vector3f::UnitY()) * (voxel_center - box_center);

			const bool collision =
			(dp.array() > (-box_size / 2).array()).all() &&
			(dp.array() < (box_size / 2).array()).all();


			if(pair.second.state == VoxelState::EMPTY) {
				collision_empty |= collision;
			} else if(pair.second.state == VoxelState::OCCUPIED) {
				if(collision) {
					collision_occupied = true;
					avg_color += pair.second.average_image_color;
					avg_count += 1;
				}
			}
		}
		if(avg_count > 0) {
			avg_color /= avg_count;

			object["r"] = avg_color.x();
			object["g"] = avg_color.y();
			object["b"] = avg_color.z();
		}

		object["valid"] = !collision_empty & collision_occupied;

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

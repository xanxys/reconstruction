#include "manhattan_belief.h"

#include <map>
#include <cmath>
#include <exception>
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

#include "../renderer/renderer.h"
#include "voxel_traversal.h"

using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using ColorCloud = pcl::PointCloud<pcl::PointXYZRGBA>;
using RigidTrans3f = Eigen::Transform<float, 3, Eigen::AffineCompact>;

const double pi = 3.14159265359;

VoxelDescription::VoxelDescription() : average_image_color(0, 0, 0) {
}

ManhattanBelief::ManhattanBelief(const ManhattanBelief& that) :
	log(that.log.str()), frame(that.frame), cloud(that.cloud),
	camera_loc_to_world(that.camera_loc_to_world), world_to_camera_loc(that.world_to_camera_loc) {
}

std::vector<std::shared_ptr<ManhattanBelief>> ManhattanBelief::expand(const FrameBelief& frame) {
	std::vector<std::shared_ptr<ManhattanBelief>> results;
	results.push_back(align(frame));
	return results;
}

std::shared_ptr<ManhattanBelief> ManhattanBelief::align(const FrameBelief& frame) {
	// Detect the primary plane.
	pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.03);  // 0.03 is better than 0.01.

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	seg.setInputCloud(frame.cloud);
	seg.segment(*inliers, *coefficients);

	const auto normal = Eigen::Vector3f(
		coefficients->values[0],
		coefficients->values[1],
		coefficients->values[2]).normalized();

	const Eigen::Vector3f up(0, -1, 0);
	Eigen::Matrix3f rotation(Eigen::Matrix3f::Identity());
	if(normal.dot(up) >= std::cos(pi / 4)) {
		// the plane is likely to be floor
		frame.log << "Correcting w/ floor" << std::endl;

		const auto axis = up.cross(normal);

		rotation = Eigen::AngleAxisf(
			-std::asin(axis.norm()),
			axis.normalized());

		// Extract remaining cloud.
		ColorCloud::Ptr cloud_remaining(new ColorCloud());

		pcl::ExtractIndices<pcl::PointXYZRGBA> extract_remaining;
		extract_remaining.setIndices(inliers);
		extract_remaining.setNegative(true);

		extract_remaining.setInputCloud(frame.cloud);
		extract_remaining.filter(*cloud_remaining);

		// Adjust Y-rotation
		seg.setInputCloud(cloud_remaining);
		seg.segment(*inliers, *coefficients);

		auto new_normal = Eigen::Vector3f(
			coefficients->values[0],
			coefficients->values[1],
			coefficients->values[2]).normalized();
		if(new_normal.z() > 0) {
			new_normal *= -1;
		}

		const Eigen::Vector3f backward(0, 0, -1);
		frame.log << new_normal << std::endl;


		if(new_normal.dot(backward) >= std::cos(pi / 4)) {
			frame.log << "Correcting w/ wall further" << std::endl;

			// Make orthogonal basis
			Eigen::Vector3f backward_real = new_normal;
			Eigen::Vector3f up_real = normal;

			backward_real -= backward_real.dot(up_real) * up_real;
			backward_real.normalize();
			const auto right_real = up_real.cross(backward_real);

			rotation.row(0) = right_real;
			rotation.row(1) = -up_real;
			rotation.row(2) = -backward_real;
		}
	} else {
		// the plane is likely to be wall
		frame.log << "Correcting w/ wall" << std::endl;

		const Eigen::Vector3f forward(0, 0, 1);
		const auto axis = forward.cross(normal);

		rotation = Eigen::AngleAxisf(
			-std::asin(axis.norm()),
			axis.normalized());

		// TODO: adjust Z-rotation
	}

	return std::make_shared<ManhattanBelief>(frame, rotation);
}

ManhattanBelief::ManhattanBelief(
	const FrameBelief& frame, Eigen::Matrix3f camera_loc_to_world) :
	frame(frame),
	camera_loc_to_world(camera_loc_to_world),
	world_to_camera_loc(camera_loc_to_world.inverse()),
	voxel_size(0.1) {

	// apply rotation.
	ColorCloud::Ptr cloud_aligned(new ColorCloud());
	for(auto pt : frame.cloud->points) {
		pt.getVector3fMap() = camera_loc_to_world * pt.getVector3fMap();
		cloud_aligned->points.push_back(pt);
	}
	cloud_aligned->width = 640;
	cloud_aligned->height = 480;

	cloud = cloud_aligned;
}

Eigen::Vector2f ManhattanBelief::projectToRGBCameraScreen(Eigen::Vector3f pos_world) {
	const auto loc = world_to_camera_loc * (pos_world - frame.camera_pos);
	return (loc.head(2) / loc.z()) * frame.camera_fl + frame.camera_center;
}

std::map<std::tuple<int, int, int>, VoxelDescription> ManhattanBelief::getVoxelsDetailed() const {
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

#include "analyzer.h"

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

#include "renderer/renderer.h"

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


TexturedPlane::TexturedPlane(float size, cv::Mat texture, float y_offset) :
	size(size), texture(texture), y_offset(y_offset)  {
}


SceneAnalyzer::SceneAnalyzer(const ColorCloud::ConstPtr& raw_cloud) :
	cloud(align(raw_cloud)), voxel_size(0.1) {
	assert(cloud);
}

std::string SceneAnalyzer::getLog() {
	return log.str();
}

ColorCloud::ConstPtr SceneAnalyzer::align(const ColorCloud::ConstPtr& cloud) {
	// Detect the primary plane.
	pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.03);  // 0.03 is better than 0.01.

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
		log << "Correcting w/ floor" << std::endl;

		const auto axis = up.cross(normal);

		rotation = Eigen::AngleAxisf(
			-std::asin(axis.norm()),
			axis.normalized());

		// Extract remaining cloud.
		ColorCloud::Ptr cloud_remaining(new ColorCloud());

		pcl::ExtractIndices<pcl::PointXYZRGBA> extract_remaining;
		extract_remaining.setIndices(inliers);
		extract_remaining.setNegative(true);

		extract_remaining.setInputCloud(cloud);
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
//		log << new_normal << std::endl;


		if(new_normal.dot(backward) >= std::cos(pi / 4)) {
			log << "Correcting w/ wall further" << std::endl;

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
		log << "Correcting w/ wall" << std::endl;

		const Eigen::Vector3f forward(0, 0, 1);
		const auto axis = forward.cross(normal);

		rotation = Eigen::AngleAxisf(
			-std::asin(axis.norm()),
			axis.normalized());

		// TODO: adjust Z-rotation
	}

	camera_loc_to_world = rotation;

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

cv::Mat SceneAnalyzer::renderRGBImage() {
	Scene scene;
	Camera camera;
	camera.fov_h = 60.0 / 180 * pi;
	scene.camera = camera;

	for(const auto& plane : getPlanes()) {
		const Eigen::Vector3f trans(0, plane.y_offset, 0);

		Triangle tri;

		tri.pos[0] = Eigen::Vector3f(-plane.size / 2, 0, -plane.size / 2) + trans;
		tri.pos[2] = Eigen::Vector3f( plane.size / 2, 0, -plane.size / 2) + trans;
		tri.pos[1] = Eigen::Vector3f(-plane.size / 2, 0,  plane.size / 2) + trans;
		tri.uv[0] = Eigen::Vector2f(0, 0);
		tri.uv[1] = Eigen::Vector2f(1, 0);
		tri.uv[2] = Eigen::Vector2f(0, 1);
		tri.texture = plane.texture;
		scene.triangles.push_back(tri);

		/*
		tri.pos[0] = Eigen::Vector3f(-plane.size / 2, 0, -plane.size / 2) + trans;
		tri.pos[1] = Eigen::Vector3f( plane.size / 2, 0, -plane.size / 2) + trans;
		tri.pos[2] = Eigen::Vector3f(-plane.size / 2, 0,  plane.size / 2) + trans;
		tri.uv[0] = Eigen::Vector2f(0, 0);
		tri.uv[1] = Eigen::Vector2f(1, 0);
		tri.uv[2] = Eigen::Vector2f(0, 1);
		tri.texture = plane.texture;
		scene.triangles.push_back(tri);
		*/

		tri.pos[0] = Eigen::Vector3f( plane.size / 2, 0,  plane.size / 2) + trans;
		tri.pos[1] = Eigen::Vector3f(-plane.size / 2, 0,  plane.size / 2) + trans;
		tri.pos[2] = Eigen::Vector3f( plane.size / 2, 0, -plane.size / 2) + trans;
		tri.uv[0] = Eigen::Vector2f(1, 1);
		tri.uv[1] = Eigen::Vector2f(0, 1);
		tri.uv[2] = Eigen::Vector2f(1, 0);
		tri.texture = plane.texture;
		scene.triangles.push_back(tri);
	}

	return Renderer::getInstance().render(scene);
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

std::vector<TexturedPlane> SceneAnalyzer::getPlanes() {
	int iy_floor = 0;
	for(const auto& pair : getVoxelsDetailed()) {
		if(pair.second.state == VoxelState::OCCUPIED) {
			iy_floor = std::max(iy_floor, std::get<1>(pair.first));
		}
	}

	const float y_floor = voxel_size * (iy_floor + 1);

	const Eigen::Vector3f camera_pos(0, 0, 0);

	const Eigen::Vector2f center(320, 240);
	const float f = 585;

	const float tile_size = 10;
	const int tex_size = 1024;
	const Eigen::Matrix3f world_to_camera_loc = camera_loc_to_world.inverse();
	cv::Mat coords(tex_size, tex_size, CV_32FC2);
	for(int y : boost::irange(0, tex_size)) {
		for(int x : boost::irange(0, tex_size)) {
			const Eigen::Vector3f pos_world(
				tile_size * (x - tex_size * 0.5) / tex_size,
				y_floor,
				tile_size * (-y + tex_size * 0.5) / tex_size);

			const auto loc = world_to_camera_loc * (pos_world - camera_pos);
			const auto screen = (loc.head(2) / loc.z()) * f + center;

			coords.at<cv::Vec2f>(y, x) = cv::Vec2f(
				screen.x(), screen.y());
		}
	}

	cv::Mat mask(tex_size, tex_size, CV_8U);  // 0:invalid 255:valid
	mask = cv::Scalar(0);

	for(const auto& pair : getVoxelsDetailed()) {
		if(pair.second.state == VoxelState::OCCUPIED &&
			std::abs(std::get<1>(pair.first) - iy_floor) <= 1) {
			const auto voxel_center = Eigen::Vector3f(
				0.5 + std::get<0>(pair.first),
				0.5 + std::get<1>(pair.first),
				0.5 + std::get<2>(pair.first)) * voxel_size;

			const int cx = tex_size * (0.5 + voxel_center.x() / tile_size);
			const int cy = tex_size - tex_size * (0.5 + voxel_center.z() / tile_size);

			cv::rectangle(mask,
				cv::Point(cx - 5, cy - 5),
				cv::Point(cx + 5, cy + 5),
				cv::Scalar(255),
				CV_FILLED);
		}
	}

	cv::Mat texture(tex_size, tex_size, CV_8UC3);
	cv::remap(getRGBImage(), texture, coords, cv::Mat(), cv::INTER_LINEAR);

	// Near-border pixels will be "tainted" by black.
	cv::Mat texture_mask_org(480, 640, CV_8U);
	texture_mask_org = cv::Scalar(255);
	cv::Mat texture_mask(tex_size, tex_size, CV_8U);
	cv::remap(texture_mask_org, texture_mask, coords, cv::Mat(), cv::INTER_LINEAR);

	// Reject pixels affected by border color.
	for(int y : boost::irange(0, tex_size)) {
		for(int x : boost::irange(0, tex_size)) {
			if(texture_mask.at<uint8_t>(y, x) != 255) {
				mask.at<uint8_t>(y, x) = 0;
			}
		}
	}

	std::vector<TexturedPlane> planes;
	planes.emplace_back(tile_size, synthesizeTexture(texture, mask), y_floor);
	return planes;
}

cv::Mat SceneAnalyzer::synthesizeTexture(const cv::Mat image, const cv::Mat mask) {
	assert(image.type() == CV_8UC3);
	assert(mask.type() == CV_8U);
	assert(image.size() == mask.size());

	cv::Mat texture(image.rows, image.cols, CV_8UC3);

	// Visualizat image & mask w/o synthesis.
	if(false) {
		for(int y : boost::irange(0, image.rows)) {
			for(int x : boost::irange(0, image.cols)) {
				if(mask.at<uint8_t>(y, x) == 0) {
					texture.at<cv::Vec3b>(y, x) =
					(image.at<cv::Vec3b>(y, x) + cv::Vec3b(0, 0, 255)) / 2;
				}

			}
		}	
	}

	// Fill spaces with avg color.
	if(false) {
		cv::Vec3f accum(0, 0, 0);
		int count = 0;
		for(int y : boost::irange(0, image.rows)) {
			for(int x : boost::irange(0, image.cols)) {
				if(mask.at<uint8_t>(y, x)) {
					accum += image.at<cv::Vec3b>(y, x);
					count += 1;
				}
			}
		}

		const cv::Vec3b avg_color = accum / count;
		for(int y : boost::irange(0, image.rows)) {
			for(int x : boost::irange(0, image.cols)) {
				if(mask.at<uint8_t>(y, x) == 0) {
					texture.at<cv::Vec3b>(y, x) = avg_color;
					
				} else {
					texture.at<cv::Vec3b>(y, x) = image.at<cv::Vec3b>(y, x);
				}
			}
		}
	}

	if(true) {
		// Select core square image.
		std::mt19937 gen;

		cv::Mat core;
		bool found = false;
		for(int size : boost::irange(100, 10, -10)) {
			for(int i : boost::irange(0, 1000)) {
				const int x0 = std::uniform_int_distribution<int>(0, image.rows - size)(gen);
				const int y0 = std::uniform_int_distribution<int>(0, image.cols - size)(gen);

				bool ok = true;
				for(int iy : boost::irange(y0, y0 + size)) {
					for(int ix : boost::irange(x0, x0 + size)) {
						if(!mask.at<uint8_t>(iy, ix)) {
							ok = false;
							break;
						}
					}
				}

				if(ok) {
					core = image(cv::Range(y0, y0 + size), cv::Range(x0, x0 + size));
					found = true;
					break;
				}
			}

			if(found) {
				break;
			}
		}

		if(found) {
			// grow it.
			texture = growTexture(core, image.cols, image.rows);
		}
	}

	return texture;
}

// ref http://graphics.cs.cmu.edu/people/efros/research/NPS/alg.html
cv::Mat SceneAnalyzer::growTexture(const cv::Mat core, int width, int height) {
	assert(core.type() == CV_8UC3);
	const int window_size = 10;

	cv::Mat image(height, width, CV_8UC3);
	image = cv::Scalar(0, 0, 0);

	// Tiling
	for(int iy : boost::irange(0, (int)std::floor(height / core.rows))) {
		for(int ix : boost::irange(0, (int)std::floor(width / core.cols))) {
			core.copyTo(image(
				cv::Range(iy * core.rows, (iy + 1) * core.rows),
				cv::Range(ix * core.cols, (ix + 1) * core.cols)));
		}
	}

	return image;
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
	for(int i : boost::irange(0, 10000)) {
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

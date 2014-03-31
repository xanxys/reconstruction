#include "scene_belief.h"

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
#include "voxel_traversal.h"

using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using ColorCloud = pcl::PointCloud<pcl::PointXYZRGBA>;
using RigidTrans3f = Eigen::Transform<float, 3, Eigen::AffineCompact>;

const double pi = 3.14159265359;

VoxelDescription::VoxelDescription() : average_image_color(0, 0, 0) {
}


TexturedPlane::TexturedPlane(float size, cv::Mat texture, float y_offset) :
	size(size), texture(texture), normal(Direction::YN)  {
	if(normal == Direction::YN) {
		center = Eigen::Vector3f(0, y_offset, 0);
	} else {
		assert(false);
	}
}


OrientedBox::OrientedBox(
	Eigen::Vector3f position,
	float ry,
	Eigen::Vector3f size,
	Eigen::Vector3f color,
	bool valid) :
	position(position), ry(ry), size(size), color(color), valid(valid) {
}

Eigen::Vector3f OrientedBox::getPosition() const {
	return position;
}

Eigen::Vector3f OrientedBox::getSize() const {
	return size;
}

Eigen::Vector3f OrientedBox::getColor() const {
	return color;
}

bool OrientedBox::getValid() const {
	return valid;
}

float OrientedBox::getRotationY() const {
	return ry;
}


SceneBelief::SceneBelief(const ColorCloud::ConstPtr& raw_cloud) :
	SceneBelief(raw_cloud, Eigen::Matrix3f::Identity()) {
}

SceneBelief::SceneBelief(
		const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud,
		Eigen::Matrix3f camera_loc_to_world) :
	cloud(cloud),
	voxel_size(0.1),
	camera_loc_to_world(camera_loc_to_world),
	world_to_camera_loc(camera_loc_to_world.inverse()),
	camera_pos(0, 0, 0),
	camera_center(320, 240),
	camera_fl(585) {
}

std::vector<std::shared_ptr<SceneBelief>> SceneBelief::expandByAlignment() {
	std::vector<std::shared_ptr<SceneBelief>> results;
	results.push_back(align());
	return results;
}

std::string SceneBelief::getLog() {
	return log.str();
}

std::shared_ptr<SceneBelief> SceneBelief::align() {
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
		log << new_normal << std::endl;


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

	// apply rotation.
	ColorCloud::Ptr cloud_aligned(new ColorCloud());
	for(auto pt : cloud->points) {
		pt.getVector3fMap() = rotation * pt.getVector3fMap();
		cloud_aligned->points.push_back(pt);
	}
	cloud_aligned->width = 640;
	cloud_aligned->height = 480;

	return std::make_shared<SceneBelief>(
		cloud_aligned, rotation);
}

Eigen::Matrix3f SceneBelief::getCameraLocalToWorld() {
	return camera_loc_to_world;
}

cv::Mat SceneBelief::getRGBImage() {
	return extractImageFromPointCloud(cloud);
}

cv::Mat SceneBelief::getDepthImage() {
	return extractDepthImageFromPointCloud(cloud);
}

cv::Mat SceneBelief::renderRGBImage() {
	Scene scene(
		Camera(640, 480,
			0.994837674, Eigen::Transform<float, 3, Eigen::Affine>(camera_loc_to_world)));

	for(const auto& plane : getPlanes()) {
		const Eigen::Vector3f trans = plane.center;

		{
			Triangle tri;
			tri.pos[0] = Eigen::Vector3f(-plane.size / 2, 0, -plane.size / 2) + trans;
			tri.pos[1] = Eigen::Vector3f( plane.size / 2, 0, -plane.size / 2) + trans;
			tri.pos[2] = Eigen::Vector3f(-plane.size / 2, 0,  plane.size / 2) + trans;
			tri.uv[0] = Eigen::Vector2f(0, 0);
			tri.uv[1] = Eigen::Vector2f(1, 0);
			tri.uv[2] = Eigen::Vector2f(0, 1);
			tri.texture = plane.texture;
			scene.triangles.push_back(tri);
		}
		{
			Triangle tri;
			tri.pos[0] = Eigen::Vector3f( plane.size / 2, 0,  plane.size / 2) + trans;
			tri.pos[1] = Eigen::Vector3f(-plane.size / 2, 0,  plane.size / 2) + trans;
			tri.pos[2] = Eigen::Vector3f( plane.size / 2, 0, -plane.size / 2) + trans;
			tri.uv[0] = Eigen::Vector2f(1, 1);
			tri.uv[1] = Eigen::Vector2f(0, 1);
			tri.uv[2] = Eigen::Vector2f(1, 0);
			tri.texture = plane.texture;
			scene.triangles.push_back(tri);	
		}
	}

	return Renderer::getInstance().render(scene);
}

pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr SceneBelief::getCloud() {
	assert(cloud);
	return cloud;
}

std::map<std::tuple<int, int, int>, VoxelState> SceneBelief::getVoxels() {
	std::map<std::tuple<int, int, int>, VoxelState> voxels;
	for(const auto& pair : getVoxelsDetailed()) {
		voxels[pair.first] = pair.second.state;
	}
	return voxels;
}

std::map<std::tuple<int, int, int>, VoxelDescription> SceneBelief::getVoxelsDetailed() {
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

std::vector<TexturedPlane> SceneBelief::getPlanes() {
	int iy_floor = 0;
	int iz_wall = 0;
	for(const auto& pair : getVoxelsDetailed()) {
		if(pair.second.state == VoxelState::OCCUPIED) {
			iy_floor = std::max(iy_floor, std::get<1>(pair.first));
			iz_wall = std::max(iz_wall, std::get<2>(pair.first));
		}
	}

	const float y_floor = voxel_size * (iy_floor + 1);
	const float z_wall = voxel_size * iz_wall;

	std::vector<TexturedPlane> planes;
	planes.push_back(extractPlane(iy_floor, y_floor, Direction::YN));
	return planes;
}

TexturedPlane SceneBelief::extractPlane(int index, float coord, Direction dir) {
	assert(dir == Direction::YN);

	const float tile_size = 10;
	const int tex_size = 1024;
	
	cv::Mat coords(tex_size, tex_size, CV_32FC2);
	for(int y : boost::irange(0, tex_size)) {
		for(int x : boost::irange(0, tex_size)) {
			const Eigen::Vector3f pos_world(
				tile_size * (x - tex_size * 0.5) / tex_size,
				coord,
				tile_size * (-y + tex_size * 0.5) / tex_size);

			const auto screen = projectToRGBCameraScreen(pos_world);
			coords.at<cv::Vec2f>(y, x) = cv::Vec2f(
				screen.x(), screen.y());
		}
	}

	cv::Mat mask(tex_size, tex_size, CV_8U);  // 0:invalid 255:valid
	mask = cv::Scalar(0);

	for(const auto& pair : getVoxelsDetailed()) {
		if(pair.second.state == VoxelState::OCCUPIED &&
			std::abs(std::get<1>(pair.first) - index) <= 1) {
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
	return TexturedPlane(tile_size, synthesizeTexture(texture, mask), coord);
}

Eigen::Vector2f SceneBelief::projectToRGBCameraScreen(Eigen::Vector3f pos_world) {
	const auto loc = world_to_camera_loc * (pos_world - camera_pos);
	return (loc.head(2) / loc.z()) * camera_fl + camera_center;
}

cv::Mat SceneBelief::synthesizeTexture(const cv::Mat image, const cv::Mat mask) {
	assert(image.type() == CV_8UC3);
	assert(mask.type() == CV_8U);
	assert(image.size() == mask.size());

	cv::Mat texture(image.rows, image.cols, CV_8UC3);
	texture = cv::Scalar(0, 0, 0);

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
cv::Mat SceneBelief::growTexture(const cv::Mat core, int width, int height) {
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

std::vector<OrientedBox> SceneBelief::getObjects() {
	std::vector<OrientedBox> objects;

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

	objects.emplace_back(
		Eigen::Vector3f((ix0 + 0.5) * voxel_size, 0, 2),
		0,
		Eigen::Vector3f(0.03, 3, 4),
		Eigen::Vector3f(0, 255, 0),
		true);

	objects.emplace_back(
		Eigen::Vector3f((ix1 + 0.5) * voxel_size, 0, 2),
		0,
		Eigen::Vector3f(0.03, 3, 4),
		Eigen::Vector3f(0, 255, 0),
		true);

	//
	for(const auto& wall_tile : projected) {
		if(wall_tile.second < 5) {
			continue;
		}

		objects.emplace_back(
			Eigen::Vector3f(
				(std::get<0>(wall_tile.first) + 0.5) * voxel_size,
				0,
				(std::get<1>(wall_tile.first) + 0.5) * voxel_size),
			0,
			Eigen::Vector3f(0.03, 3, 0.03),
			Eigen::Vector3f(0, 0, 255),
			true);
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
		Eigen::Vector3f box_color(0, 0, 0);
		if(avg_count > 0) {
			box_color = avg_color / avg_count;
		}
		const bool valid = !collision_empty & collision_occupied;

		objects.emplace_back(
			box_center,
			rot_y,
			box_size,
			box_color,
			valid);
	}

	return objects;
}

cv::Mat SceneBelief::extractImageFromPointCloud(
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

cv::Mat SceneBelief::extractDepthImageFromPointCloud(
	const ColorCloud::ConstPtr& cloud) {
	if(cloud->points.size() != cloud->width * cloud->height) {
		throw std::runtime_error("Point cloud is not an image");
	}

	cv::Mat depth(cloud->height, cloud->width, CV_32F);
	for(int y : boost::irange(0, (int)cloud->height)) {
		for(int x : boost::irange(0, (int)cloud->width)) {
			const pcl::PointXYZRGBA& pt = cloud->points[y * cloud->width + x];

			depth.at<float>(y, x) = pt.z;
		}
	}
	return depth;
}

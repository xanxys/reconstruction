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

#include "../renderer/renderer.h"
#include "voxel_traversal.h"

using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using ColorCloud = pcl::PointCloud<pcl::PointXYZRGBA>;
using RigidTrans3f = Eigen::Transform<float, 3, Eigen::AffineCompact>;

const double pi = 3.14159265359;


TexturedPlane::TexturedPlane(float size, cv::Mat texture, float offset, Direction normal) :
	size(size), texture(texture), normal(normal) {
	switch(normal) {
		case Direction::XP:
		case Direction::XN:
			center = Eigen::Vector3f(offset, 0, 0);
			break;
		case Direction::YP:
		case Direction::YN:
			center = Eigen::Vector3f(0, offset, 0);
			break;
		case Direction::ZP:
		case Direction::ZN:
			center = Eigen::Vector3f(0, 0, offset);
			break;
	}
}

Eigen::Matrix3f TexturedPlane::getLocalToWorld() const {
	Eigen::Matrix3f tr;
	switch(normal) {
		case Direction::XP:
			tr.col(0) = Eigen::Vector3f::UnitY();
			tr.col(1) = Eigen::Vector3f::UnitZ();
			tr.col(2) = Eigen::Vector3f::UnitX();
			break;
		case Direction::YP:
			tr.col(0) = Eigen::Vector3f::UnitZ();
			tr.col(1) = Eigen::Vector3f::UnitX();
			tr.col(2) = Eigen::Vector3f::UnitY();
			break;
		case Direction::ZP:
			tr.col(0) = Eigen::Vector3f::UnitX();
			tr.col(1) = Eigen::Vector3f::UnitY();
			tr.col(2) = Eigen::Vector3f::UnitZ();
			break;
		case Direction::XN:
			tr.col(0) = Eigen::Vector3f::UnitZ();
			tr.col(1) = Eigen::Vector3f::UnitY();
			tr.col(2) = -Eigen::Vector3f::UnitX();
			break;
		case Direction::YN:
			tr.col(0) = Eigen::Vector3f::UnitX();
			tr.col(1) = Eigen::Vector3f::UnitZ();
			tr.col(2) = -Eigen::Vector3f::UnitY();
			break;
		case Direction::ZN:
			tr.col(0) = Eigen::Vector3f::UnitY();
			tr.col(1) = Eigen::Vector3f::UnitX();
			tr.col(2) = -Eigen::Vector3f::UnitZ();
			break;
	}
	return tr;
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


SceneBelief::SceneBelief(const WallBelief& w) :
	wall(w), floor(wall.floor), manhattan(floor.manhattan), frame(manhattan.frame) {
}

std::string SceneBelief::getLog() {
	return
		"== Frame\n" + frame.log.str() + "\n" +
		"== Manhattan\n" + manhattan.log.str() + "\n" +
		"== Floor\n" + floor.log.str() + "\n" +
		"== Wall\n" + wall.log.str() + "\n";
}

Eigen::Matrix3f SceneBelief::getCameraLocalToWorld() {
	return manhattan.camera_loc_to_world;
}

cv::Mat SceneBelief::getRGBImage() {
	return frame.extractImage();
}

cv::Mat SceneBelief::getDepthImage() {
	return frame.extractDepthImage();
}

cv::Mat SceneBelief::renderRGBImage() {
	Scene scene(
		Camera(640, 480,
			0.994837674, Eigen::Transform<float, 3, Eigen::Affine>(manhattan.camera_loc_to_world)));

	for(const auto& plane : getPlanes()) {
		const Eigen::Vector3f trans = plane.center;

		{
			Triangle tri;
			tri.pos[0] = plane.getLocalToWorld() * Eigen::Vector3f(-plane.size / 2, -plane.size / 2, 0) + trans;
			tri.pos[1] = plane.getLocalToWorld() * Eigen::Vector3f( plane.size / 2, -plane.size / 2, 0) + trans;
			tri.pos[2] = plane.getLocalToWorld() * Eigen::Vector3f(-plane.size / 2,  plane.size / 2, 0) + trans;
			tri.uv[0] = Eigen::Vector2f(0, 0);
			tri.uv[1] = Eigen::Vector2f(1, 0);
			tri.uv[2] = Eigen::Vector2f(0, 1);
			tri.texture = plane.texture;
			scene.triangles.push_back(tri);
		}
		{	Triangle tri;
			tri.pos[0] = plane.getLocalToWorld() * Eigen::Vector3f( plane.size / 2,  plane.size / 2, 0) + trans;
			tri.pos[1] = plane.getLocalToWorld() * Eigen::Vector3f(-plane.size / 2,  plane.size / 2, 0) + trans;
			tri.pos[2] = plane.getLocalToWorld() * Eigen::Vector3f( plane.size / 2, -plane.size / 2, 0) + trans;
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
	assert(manhattan.cloud);
	return manhattan.cloud;
}

std::map<std::tuple<int, int, int>, VoxelState> SceneBelief::getVoxels() {
	std::map<std::tuple<int, int, int>, VoxelState> voxels;
	for(const auto& pair : getVoxelsDetailed()) {
		voxels[pair.first] = pair.second.state;
	}
	return voxels;
}

std::map<std::tuple<int, int, int>, VoxelDescription> SceneBelief::getVoxelsDetailed() {
	return manhattan.getVoxelsDetailed();
}

std::vector<TexturedPlane> SceneBelief::getPlanes() {
	int iz_wall = 0;
	for(const auto& pair : getVoxelsDetailed()) {
		if(pair.second.state == VoxelState::OCCUPIED) {
			iz_wall = std::max(iz_wall, std::get<2>(pair.first));
		}
	}

	const float y_floor = manhattan.voxel_size * (floor.index + 0.5);
	const float z_wall = manhattan.voxel_size * iz_wall;

	std::vector<TexturedPlane> planes;
	planes.push_back(extractPlane(floor.index, y_floor, Direction::YN));
	// planes.push_back(extractPlane(iz_wall, z_wall, Direction::ZN));
	return planes;
}

TexturedPlane SceneBelief::extractPlane(int index, float coord, Direction dir) {
	const float tile_size = 10;
	const int tex_size = 1024;
	TexturedPlane plane(tile_size, cv::Mat(), coord, dir);
	
	cv::Mat coords(tex_size, tex_size, CV_32FC2);
	for(int y : boost::irange(0, tex_size)) {
		for(int x : boost::irange(0, tex_size)) {
			const Eigen::Vector3f pos_local(
				tile_size * (static_cast<float>(x) / tex_size - 0.5),
				tile_size * (static_cast<float>(tex_size - y) / tex_size - 0.5),
				0);
			const Eigen::Vector3f pos_world =
				plane.getLocalToWorld() * pos_local + plane.center;
			const auto screen = manhattan.projectToRGBCameraScreen(pos_world);
			coords.at<cv::Vec2f>(y, x) = cv::Vec2f(screen.x(), screen.y());
		}
	}

	cv::Mat mask(tex_size, tex_size, CV_8U);  // 0:invalid 255:valid
	mask = cv::Scalar(0);

	for(const auto& pair : getVoxelsDetailed()) {
		const auto voxel_center = Eigen::Vector3f(
			0.5 + std::get<0>(pair.first),
			0.5 + std::get<1>(pair.first),
			0.5 + std::get<2>(pair.first)) * manhattan.voxel_size;

		if(dir == Direction::YN) {
			if(pair.second.state == VoxelState::OCCUPIED &&
				std::get<1>(pair.first) == index) {

				const Eigen::Vector3f voxel_c_local = plane.getLocalToWorld().inverse() * (voxel_center - plane.center);

				const int cx = (voxel_c_local.x() / tile_size + 0.5) * tex_size;
				const int cy = tex_size - (voxel_c_local.y() / tile_size + 0.5) * tex_size;

				cv::rectangle(mask,
					cv::Point(cx - 5, cy - 5),
					cv::Point(cx + 5, cy + 5),
					cv::Scalar(255),
					CV_FILLED);
			}
		} else if(dir == Direction::ZN) {
			if(pair.second.state == VoxelState::OCCUPIED &&
				std::get<2>(pair.first) == index) {
				const int cx = tex_size * (0.5 + voxel_center.y() / tile_size);
				const int cy = tex_size - tex_size * (0.5 + voxel_center.x() / tile_size);

				cv::rectangle(mask,
					cv::Point(cx - 5, cy - 5),
					cv::Point(cx + 5, cy + 5),
					cv::Scalar(255),
					CV_FILLED);
			}
		} else {
			assert(false);
		}
	}

	// TODO: getRGBImage relies on raw RGB in FrameBelief!!
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

	plane.texture = synthesizeTexture(texture, mask);
	return plane;
}

cv::Mat SceneBelief::synthesizeTexture(const cv::Mat image, const cv::Mat mask) {
	assert(image.type() == CV_8UC3);
	assert(mask.type() == CV_8U);
	assert(image.size() == mask.size());

	cv::Mat texture(image.rows, image.cols, CV_8UC3);
	texture = cv::Scalar(0, 0, 0);

	// Visualize image & mask w/o synthesis.
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
	const float y_floor = (iy_floor + 1) * manhattan.voxel_size;

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
		Eigen::Vector3f((ix0 + 0.5) * manhattan.voxel_size, 0, 2),
		0,
		Eigen::Vector3f(0.03, 3, 4),
		Eigen::Vector3f(0, 255, 0),
		true);

	objects.emplace_back(
		Eigen::Vector3f((ix1 + 0.5) * manhattan.voxel_size, 0, 2),
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
				(std::get<0>(wall_tile.first) + 0.5) * manhattan.voxel_size,
				0,
				(std::get<1>(wall_tile.first) + 0.5) * manhattan.voxel_size),
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
				0.5 + std::get<2>(pair.first)) * manhattan.voxel_size;

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

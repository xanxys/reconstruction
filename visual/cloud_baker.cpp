#include "cloud_baker.h"

#include <array>
#include <iostream>
#include <fstream>
#include <limits>

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
#include <visual/scene_converter.h>
#include <visual/texture_conversion.h>

namespace visual {

using Tuple3i = std::tuple<int, int, int>;

void TexturedMesh::writeWavefrontObject(std::string dir_name) const {
	const boost::filesystem::path dir_path(dir_name);
	boost::filesystem::create_directory(dir_path);

	const boost::filesystem::path name_obj("object.obj");
	const boost::filesystem::path name_mtl("object.mtl");
	const boost::filesystem::path name_diffuse("diffuse.png");

	// Write bunch of files.
	std::ofstream f_obj((dir_path / name_obj).string());
	mesh.serializeObjWithUv(f_obj, name_mtl.string());

	std::ofstream f_mtl((dir_path / name_mtl).string());
	writeObjMaterial(f_mtl, name_diffuse.string());

	cv::imwrite((dir_path / name_diffuse).string(), diffuse);
}


VoxelDescription::VoxelDescription() : average_image_color(0, 0, 0), guess(false) {
}


Voxelizer::Voxelizer(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float voxel_size) :
	cloud(cloud), voxel_size(voxel_size) {
}

std::map<Tuple3i, VoxelDescription> Voxelizer::getVoxelsDetailed() const {
	std::map<Tuple3i, VoxelDescription> vx;
	return vx;
}

std::map<Tuple3i, VoxelDescription> Voxelizer::getVoxelsDetailedWithoutGuess() const {

	// known to be filled
	std::map<Tuple3i, bool> voxels;
	std::map<Tuple3i, Eigen::Vector3f> voxels_accum;
	std::map<Tuple3i, int> voxels_count;
	for(const auto& pt : cloud->points) {
		if(!std::isfinite(pt.x)) {
			continue;
		}

		auto ix = pt.getVector3fMap() / voxel_size;
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

	// TODO: remove and use frame.camera_pos (need to consolidate
	// frame.camera_pos semantics)
	const auto& camera_origin = Eigen::Vector3f::Zero();

	std::map<Tuple3i, bool> voxels_empty;
	for(const auto& pair_filled : voxels) {
		// cast ray from camera
		const auto pos = Eigen::Vector3f(
			std::get<0>(pair_filled.first) + 0.5,
			std::get<1>(pair_filled.first) + 0.5,
			std::get<2>(pair_filled.first) + 0.5) * voxel_size;

		const auto dir = (pos - camera_origin).normalized();

		// traverse until hit.
		VoxelTraversal traversal(voxel_size, camera_origin, dir);
		for(int i : boost::irange(0, 100)) {
			const auto key = traversal.next();

			// Hit wall.
			if(voxels.find(key) != voxels.end()) {
				break;
			}

			voxels_empty[key] = true;
		}
	}

	std::map<Tuple3i, VoxelDescription> voxel_merged;
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


CloudBaker::CloudBaker(const Json::Value& cloud_json) {
	cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
	for(const auto& point : cloud_json) {
		pcl::PointXYZRGB pt;
		pt.x = point["x"].asDouble();
		pt.y = point["y"].asDouble();
		pt.z = point["z"].asDouble();
		pt.r = point["r"].asDouble();
		pt.g = point["g"].asDouble();
		pt.b = point["b"].asDouble();
		cloud->points.push_back(pt);
	}
}

TexturedMesh CloudBaker::generateRoomMesh() {
	const auto cloud_colorless = decolor(*cloud);
	TriangleMesh<std::nullptr_t> mesh = OBBFitter(cloud_colorless).extract();
	const auto mesh_uv = mapSecond(assignUV(mesh));

	// Project points to the surface and draw circles onto texture.
	const int tex_size = 2048;
	cv::Mat diffuse(tex_size, tex_size, CV_8UC3);
	diffuse = cv::Scalar(0, 0, 0);
	for(const auto& point : cloud->points) {
		const Eigen::Vector3f pos = point.getVector3fMap();
		const auto uv = nearestCoordinate(mesh_uv, pos);
		const cv::Scalar color(point.b, point.g, point.r);
		cv::circle(
			diffuse, eigenToCV(swapY(uv) * tex_size), 1,
			color, -1);
	}
	fillHoles(diffuse, cv::Vec3b(0, 0, 0), 5);

	TexturedMesh tm;
	tm.diffuse = diffuse;
	tm.mesh = mesh_uv;
	return tm;
}

void CloudBaker::fillHoles(cv::Mat& image, const cv::Vec3b undefined, int iteration) {
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

void CloudBaker::writeWavefrontObject() {
	auto tm = generateRoomMesh();
	tm.writeWavefrontObject("room_box");
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CloudBaker::decolor(const pcl::PointCloud<pcl::PointXYZRGB>& cloud) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_colorless(new pcl::PointCloud<pcl::PointXYZ>());
	for(const auto& point : cloud) {
		pcl::PointXYZ pt;
		pt.x = point.x;
		pt.y = point.y;
		pt.z = point.z;
		cloud_colorless->points.push_back(pt);
	}
	return cloud_colorless;
}

}  // namespace

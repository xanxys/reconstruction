#include "cloud_baker.h"

#include <array>
#include <iostream>
#include <fstream>

#include <boost/filesystem.hpp>
#include <boost/range/irange.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <geom/packing.h>
#include <math_util.h>
#include <range2.h>
#include <visual/cloud_conversion.h>
#include <visual/texture_conversion.h>

namespace recon {

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

void TexturedMesh::writeWavefrontObjectFlat(std::string prefix) const {
	const boost::filesystem::path name_obj(prefix + "object.obj");
	const boost::filesystem::path name_mtl(prefix + "object.mtl");
	const boost::filesystem::path name_diffuse(prefix + "diffuse.png");

	// Write bunch of files.
	std::ofstream f_obj(name_obj.string());
	mesh.serializeObjWithUv(f_obj, name_mtl.filename().string());

	std::ofstream f_mtl(name_mtl.string());
	writeObjMaterial(f_mtl, name_diffuse.filename().string());

	cv::imwrite(name_diffuse.string(), diffuse);
}

TexturedMesh mergeTexturedMeshes(
		const std::vector<TexturedMesh>& meshes) {
	assert(!meshes.empty());
	// Plan texture packing.
	std::vector<Eigen::Vector2f> rects;
	for(const auto& mesh : meshes) {
		rects.emplace_back(mesh.diffuse.cols, mesh.diffuse.rows);
	}
	const auto packing = packRectangles(rects);
	// Actually pack texture.
	const int size = ceilToPowerOf2(packing.first);
	cv::Mat result_diffuse = cv::Mat::zeros(size, size, CV_8UC3);
	for(const int i : boost::irange(0, (int)meshes.size())) {
		assert(meshes[i].diffuse.type() == CV_8UC3);
		meshes[i].diffuse.copyTo(
			result_diffuse(cv::Rect(
				packing.second[i](0),
				packing.second[i](1),
				meshes[i].diffuse.cols,
				meshes[i].diffuse.rows)));
	}
	// Pack meshes.
	TriangleMesh<Eigen::Vector2f> result_mesh;
	for(const int i : boost::irange(0, (int)meshes.size())) {
		const auto& mesh = meshes[i].mesh;
		const int i_vert_offset = result_mesh.vertices.size();
		for(const auto& v : mesh.vertices) {
			result_mesh.vertices.emplace_back(
				v.first,
				(v.second.cwiseProduct(rects[i]) + packing.second[i]) / size);
		}
		for(const auto& tri : mesh.triangles) {
			result_mesh.triangles.push_back({{
				i_vert_offset + tri[0],
				i_vert_offset + tri[1],
				i_vert_offset + tri[2]}});
		}
	}

	TexturedMesh result;
	result.diffuse = result_diffuse;
	result.mesh = result_mesh;
	return result;
}

}  // namespace

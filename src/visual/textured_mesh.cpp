#include "cloud_baker.h"

#include <array>
#include <cmath>
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

TexturedMesh::TexturedMesh(
		const TriangleMesh<Eigen::Vector2f>& mesh,
		const cv::Mat& diffuse) :
		has_normal(false), mesh(mesh), diffuse(diffuse) {
}

TexturedMesh::TexturedMesh(
		const TriangleMesh<std::pair<Eigen::Vector3f, Eigen::Vector2f>>& mesh,
		const cv::Mat& diffuse) :
		has_normal(true), mesh_w_normal(mesh), diffuse(diffuse) {
}

void TexturedMesh::writeWavefrontObject(
		const std::string& dir_name) const {
	const boost::filesystem::path dir_path(dir_name);
	boost::filesystem::create_directory(dir_path);

	const boost::filesystem::path name_obj("object.obj");
	const boost::filesystem::path name_mtl("object.mtl");
	const boost::filesystem::path name_diffuse("diffuse.png");

	// Write bunch of files.
	std::ofstream f_obj((dir_path / name_obj).string());
	mesh.serializeObjWithUv(f_obj, name_mtl.string(), "the_material");

	std::ofstream f_mtl((dir_path / name_mtl).string());
	writeObjMaterial(f_mtl, name_diffuse.string(), "the_material");

	cv::imwrite((dir_path / name_diffuse).string(), diffuse);
}

void TexturedMesh::writeWavefrontObjectFlat(
		const std::string& prefix) const {
	const boost::filesystem::path name_obj(prefix + "object.obj");
	const boost::filesystem::path name_mtl(prefix + "object.mtl");
	const boost::filesystem::path name_diffuse(prefix + "diffuse.png");

	// Write bunch of files.
	std::ofstream f_obj(name_obj.string());
	mesh.serializeObjWithUv(f_obj, name_mtl.filename().string(),
		"the_material");

	std::ofstream f_mtl(name_mtl.string());
	writeObjMaterial(f_mtl, name_diffuse.filename().string(),
		"the_material");

	cv::imwrite(name_diffuse.string(), diffuse);
}

void TexturedMesh::extrapolateAtlasBoundary() {
	const int tex_size = getTextureSize();
	const Eigen::Vector3f invalid(1e5, 1e5, 1e5);
	const cv::Vec3f invalid_v(invalid.x(), invalid.y(), invalid.z());
	cv::Mat pos_map;
	if(has_normal) {
		pos_map = getPositionMapInUV(mapSecond(mesh_w_normal), tex_size, invalid);
	} else {
		pos_map = getPositionMapInUV(mesh, tex_size, invalid);
	}

	cv::Mat missing_mask(tex_size, tex_size, CV_8U);
	for(const int y : boost::irange(0, tex_size)) {
		for(const int x : boost::irange(0, tex_size)) {
			missing_mask.at<uint8_t>(y, x) =
				(pos_map.at<cv::Vec3f>(y, x) == invalid_v) ? 255 : 0;
		}
	}

	cv::Mat new_diffuse;
	cv::inpaint(diffuse, missing_mask, new_diffuse, 3, cv::INPAINT_TELEA);
	diffuse = new_diffuse;
}

int TexturedMesh::getTextureSize() const {
	assert(diffuse.cols > 0 && diffuse.cols == diffuse.rows);
	return diffuse.cols;
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
		for(const auto& vertex : mesh.vertices) {
			result_mesh.vertices.emplace_back(
				vertex.first,
				Eigen::Vector2f(
					(vertex.second.x() * rects[i].x() + packing.second[i].x()) / size,
					1 - ((1 - vertex.second.y()) * rects[i].y() + packing.second[i].y()) / size));
		}
		for(const auto& tri : mesh.triangles) {
			result_mesh.triangles.push_back({{
				i_vert_offset + tri[0],
				i_vert_offset + tri[1],
				i_vert_offset + tri[2]}});
		}
	}

	return TexturedMesh(result_mesh, result_diffuse);
}

void writeObjMaterial(
		std::ostream& output,
		const std::string& texture_path,
		const std::string& material_name) {
	output << "newmtl " << material_name << std::endl;
	output << "Ka 1.0 1.0 1.0" << std::endl;
	output << "Kd 1.0 1.0 1.0" << std::endl;
	output << "Ks 0.0 0.0 0.0" << std::endl;
	output << "map_Kd " << texture_path << std::endl;
}

cv::Mat getPositionMapInUV(
		const TriangleMesh<Eigen::Vector2f>& mesh, int tex_size,
		const Eigen::Vector3f& default_value) {
	assert(tex_size > 0);
	cv::Mat pos_map(tex_size, tex_size, CV_32FC3);
	pos_map = cv::Scalar(default_value.x(), default_value.y(), default_value.z());

	const Eigen::Vector2i bnd_tex_low(0, 0);
	const Eigen::Vector2i bnd_tex_high(tex_size, tex_size);
	for(const auto& tri : mesh.triangles) {
		// Triangle on x-y space of texture.
		const Eigen::Vector2f p0 = swapY(mesh.vertices[tri[0]].second) * tex_size;
		const Eigen::Vector2f p1 = swapY(mesh.vertices[tri[1]].second) * tex_size;
		const Eigen::Vector2f p2 = swapY(mesh.vertices[tri[2]].second) * tex_size;
		Eigen::Matrix2f ps;
		ps.col(0) = p1 - p0;
		ps.col(1) = p2 - p0;
		if(std::abs(ps.determinant()) < 1e-5) {
			// degenerate triangle -> zero matrix (= just use p0)
			ps = Eigen::Matrix2f::Zero();
		} else {
			ps = ps.inverse().eval();
		}

		Eigen::Matrix3f vs;
		vs.col(0) = mesh.vertices[tri[0]].first;
		vs.col(1) = mesh.vertices[tri[1]].first;
		vs.col(2) = mesh.vertices[tri[2]].first;

		// Fill all pixels within AABB of the triangle.
		const Eigen::Vector2f p_min = p0.cwiseMin(p1).cwiseMin(p2);
		const Eigen::Vector2f p_max = p0.cwiseMax(p1).cwiseMax(p2);
		const Eigen::Vector2i pi_min = Eigen::Vector2i(p_min(0) - 1, p_min(1) - 1).cwiseMax(bnd_tex_low);
		const Eigen::Vector2i pi_max = Eigen::Vector2i(p_max(0) + 1, p_max(1) + 1).cwiseMin(bnd_tex_high);

		for(const auto p : range2(pi_min, pi_max)) {
			const Eigen::Vector2f ts_pre = ps * (p.cast<float>() - p0);
			const Eigen::Vector3f ts(1 - ts_pre.sum(), ts_pre(0), ts_pre(1));
			if(ts(0) < 0 || ts(1) < 0 || ts(2) < 0) {
				// p is in outside of the triangle.
				continue;
			}
			const Eigen::Vector3f pos_w = vs * ts;
			assert(std::isfinite(pos_w.x()));
			assert(std::isfinite(pos_w.y()));
			assert(std::isfinite(pos_w.z()));
			pos_map.at<cv::Vec3f>(p(1), p(0)) = cv::Vec3f(pos_w.x(), pos_w.y(), pos_w.z());
		}
	}
	return pos_map;
}

}  // namespace

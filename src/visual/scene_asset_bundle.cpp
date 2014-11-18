#include "scene_asset_bundle.h"

#include <boost/filesystem.hpp>

#include <fstream>

namespace visual {

using boost::filesystem::path;

SceneAssetBundle::SceneAssetBundle(std::string dir_path, bool debug) :
		debug_count(0), dir_path(dir_path), do_finalize(true), debug(debug) {
	recreateDirectory(dir_path);
}

SceneAssetBundle::~SceneAssetBundle() {
	if(do_finalize) {
		serializeIntoDirectory(dir_path);
	} else {
		std::ofstream json_file((dir_path / path("small_data.json")).string());
		json_file << Json::FastWriter().write(serializeSmallData());
	}
}

bool SceneAssetBundle::isDebugEnabled() const {
	return debug;
}

void SceneAssetBundle::recreateDirectory(std::string dir_path_raw) const {
	using boost::filesystem::create_directory;
	using boost::filesystem::remove_all;

	const path dir_path(dir_path_raw);

	// Remove directory if exists, mimicing overwriting semantics
	// of a single file.
	remove_all(dir_path);
	create_directory(dir_path);
}

void SceneAssetBundle::serializeIntoDirectory(std::string dir_path_raw) {
	using boost::filesystem::create_directory;
	using boost::filesystem::remove_all;

	const path dir_path(dir_path_raw);

	exterior_mesh.writeWavefrontObject(
		(dir_path / path("exterior_mesh")).string());
	{
		std::ofstream json_file((dir_path / path("small_data.json")).string());
		json_file << Json::FastWriter().write(serializeSmallData());
	}
	int count = 0;
	for(const auto& interior : interior_objects) {
		object_ids.push_back(std::to_string(count));
		const std::string name = "flat_poly_" + std::to_string(count++);
		interior.writeWavefrontObjectFlat((dir_path / name).string());
	}
}


Json::Value SceneAssetBundle::loadJson(std::string name) const {
	std::ifstream f_input((path(dir_path) / path(name)).string());
	if(!f_input.is_open()) {
		throw std::runtime_error("Cloudn't open " + name);
	}
	Json::Value root;
	Json::Reader().parse(f_input, root);
	return root;
}

void SceneAssetBundle::addInteriorObject(const TexturedMesh& mesh) {
	if(debug) {
		addMesh(
			"poly_" + std::to_string(interior_objects.size()),
			mesh);
	}
	interior_objects.push_back(mesh);
}

void SceneAssetBundle::addDebugPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
	const int id = debug_count++;
	std::ofstream debug_points_file((dir_path / path("debug_" + std::to_string(id) + ".ply")).string());
	serializeDebugPoints(cloud).serializePLYWithRgb(debug_points_file);
}

void SceneAssetBundle::addDebugPointCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud) {
	const int id = debug_count++;
	std::ofstream debug_points_file((dir_path / path("debug_" + std::to_string(id) + ".ply")).string());
	serializeDebugPoints(cloud).serializePLYWithRgbNormal(debug_points_file);
}

void SceneAssetBundle::addDebugPointCloud(std::string name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
	std::ofstream debug_points_file((dir_path / path("debug_" + name + ".ply")).string());
	serializeDebugPoints(cloud).serializePLYWithRgb(debug_points_file);
}

void SceneAssetBundle::addDebugPointCloud(std::string name, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud) {
	std::ofstream debug_points_file((dir_path / path("debug_" + name + ".ply")).string());
	serializeDebugPoints(cloud).serializePLYWithRgbNormal(debug_points_file);
}

void SceneAssetBundle::addMesh(std::string name, const TriangleMesh<std::nullptr_t>& mesh) {
	std::ofstream mesh_f((dir_path / path(name + ".ply")).string());
	mesh.serializePLY(mesh_f);
}

void SceneAssetBundle::addMesh(std::string name, const TexturedMesh& mesh) {
	mesh.writeWavefrontObject((dir_path / path(name)).string());
}

void SceneAssetBundle::addMeshFlat(std::string name, const TexturedMesh& mesh) {
	mesh.writeWavefrontObjectFlat((dir_path / path(name)).string());
}


TriangleMesh<Eigen::Vector3f> SceneAssetBundle::serializeDebugPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) const {
	TriangleMesh<Eigen::Vector3f> mesh;
	for(const auto& pt : cloud->points) {
		mesh.vertices.push_back(std::make_pair(
			pt.getVector3fMap(),
			Eigen::Vector3f(pt.r, pt.g, pt.b)));
	}
	return mesh;
}

TriangleMesh<std::tuple<Eigen::Vector3f, Eigen::Vector3f>> SceneAssetBundle::serializeDebugPoints(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud) const {
	TriangleMesh<std::tuple<Eigen::Vector3f, Eigen::Vector3f>> mesh;
	for(const auto& pt : cloud->points) {
		mesh.vertices.push_back(std::make_pair(
			pt.getVector3fMap(),
			std::make_tuple(
				Eigen::Vector3f(pt.r, pt.g, pt.b),
				pt.getNormalVector3fMap())));
	}
	return mesh;
}

Json::Value SceneAssetBundle::serializeSmallData() const {
	Json::Value small_data;
	for(const auto& pos : point_lights) {
		Json::Value light;
		light["pos"]["x"] = pos(0);
		light["pos"]["y"] = pos(1);
		light["pos"]["z"] = pos(2);
		small_data["lights"].append(light);
	}
	for(const auto& oid : object_ids) {
		small_data["objects"].append(oid);
	}
	return small_data;
}

}  // namespace

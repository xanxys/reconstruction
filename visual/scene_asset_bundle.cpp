#include "scene_asset_bundle.h"

#include <fstream>

#include <boost/filesystem.hpp>

namespace visual {

SceneAssetBundle::SceneAssetBundle(std::string dir_path) : debug_count(0), dir_path(dir_path) {
}

SceneAssetBundle::~SceneAssetBundle() {
	serializeIntoDirectory(dir_path);
}

void SceneAssetBundle::serializeIntoDirectory(std::string dir_path_raw) const {
	using boost::filesystem::create_directory;
	using boost::filesystem::path;
	using boost::filesystem::remove_all;

	const path dir_path(dir_path_raw);

	// Remove directory if exists, mimicing overwriting semantics
	// of a single file.
	remove_all(dir_path);
	create_directory(dir_path);
	exterior_mesh.writeWavefrontObject(
		(dir_path / path("exterior_mesh")).string());
	{
		std::ofstream debug_points_file((dir_path / path("debug_points_interior.ply")).string());
		serializeDebugPoints(debug_points_interior).serializePLYWithRgb(debug_points_file);
	}
	{
		std::ofstream debug_points_file((dir_path / path("debug_points_interior_2d.ply")).string());
		serializeDebugPoints(debug_points_interior_2d).serializePLYWithRgb(debug_points_file);
	}
	{
		std::ofstream debug_points_file((dir_path / path("debug_points_interior_distance.ply")).string());
		serializeDebugPoints(debug_points_interior_distance).serializePLYWithRgb(debug_points_file);
	}
	{
		std::ofstream debug_points_file((dir_path / path("debug_points_merged.ply")).string());
		serializeDebugPoints(debug_points_merged).serializePLYWithRgb(debug_points_file);
	}
	{
		std::ofstream json_file((dir_path / path("small_data.json")).string());
		json_file << Json::FastWriter().write(serializeSmallData());
	}
	int count = 0;
	for(const auto& interior : interior_objects) {
		const std::string name = "interior_" + std::to_string(count++);
		interior.writeWavefrontObject((dir_path / name).string());
	}
}

void SceneAssetBundle::addDebugPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
	using boost::filesystem::path;

	const int id = debug_count++;
	std::ofstream debug_points_file((dir_path / path("debug_" + std::to_string(id) + ".ply")).string());
	serializeDebugPoints(cloud).serializePLYWithRgb(debug_points_file);
}

void SceneAssetBundle::addDebugPointCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud) {
	using boost::filesystem::path;

	const int id = debug_count++;
	std::ofstream debug_points_file((dir_path / path("debug_" + std::to_string(id) + ".ply")).string());
	serializeDebugPoints(cloud).serializePLYWithRgb(debug_points_file);
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

TriangleMesh<Eigen::Vector3f> SceneAssetBundle::serializeDebugPoints(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud) const {
	TriangleMesh<Eigen::Vector3f> mesh;
	for(const auto& pt : cloud->points) {
		mesh.vertices.push_back(std::make_pair(
			pt.getVector3fMap(),
			Eigen::Vector3f(pt.r, pt.g, pt.b)));
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
	return small_data;
}

}  // namespace
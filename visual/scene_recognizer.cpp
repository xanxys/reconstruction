#include "scene_recognizer.h"

#include <fstream>

#include <boost/filesystem.hpp>
#include <boost/range/irange.hpp>

#include <visual/cloud_baker.h>

namespace visual {

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

	std::ofstream json_file((dir_path / path("small_data.json")).string());
	json_file << Json::FastWriter().write(serializeSmallData());
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


SingleScan::SingleScan(Json::Value& cloud_json) {
	// Convert input coords. space to make Z+ up.
	// Only required for old dataset which was Y+ up.
	Eigen::Matrix3f m;
	m.col(0) = Eigen::Vector3f(0, 1, 0);
	m.col(1) = Eigen::Vector3f(0, 0, 1);
	m.col(2) = Eigen::Vector3f(1, 0, 0);
	cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
	for(const auto& point : cloud_json) {
		pcl::PointXYZRGB pt;
		pt.x = point["x"].asDouble();
		pt.y = point["y"].asDouble();
		pt.z = point["z"].asDouble();
		pt.r = point["r"].asDouble();
		pt.g = point["g"].asDouble();
		pt.b = point["b"].asDouble();

		pt.getVector3fMap() = m * pt.getVector3fMap();
		cloud->points.push_back(pt);
	}
}

std::vector<Eigen::Vector3f> recognize_lights(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
	std::vector<Eigen::Vector3f> lights;
	lights.emplace_back(1, 1.1, 2.5);
	return lights;
}

namespace scene_recognizer {

SceneAssetBundle recognizeScene(const std::vector<SingleScan>& scans) {
	assert(scans.size() > 0);

	SceneAssetBundle bundle;
	bundle.exterior_mesh = visual::CloudBaker(scans[0].cloud).generateRoomMesh();
	bundle.point_lights = visual::recognize_lights(scans[0].cloud);
	return bundle;
}

}  // namespace

}  // namespace

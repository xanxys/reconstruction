#include "scene_recognizer.h"

#include <boost/filesystem.hpp>
#include <boost/range/irange.hpp>

#include <visual/cloud_baker.h>

namespace visual {

void SceneAssetBundle::serializeIntoDirectory(std::string dir_path) const {
	using boost::filesystem::create_directory;
	using boost::filesystem::path;

	create_directory(path(dir_path));
	exterior_mesh.writeWavefrontObject(
		(path(dir_path) / path("exterior_mesh")).string());
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

namespace scene_recognizer {

SceneAssetBundle recognizeScene(const std::vector<SingleScan>& scans) {
	assert(scans.size() > 0);

	SceneAssetBundle bundle;
	bundle.exterior_mesh = visual::CloudBaker(scans[0].cloud).generateRoomMesh();
	return bundle;
}

}  // namespace

}  // namespace

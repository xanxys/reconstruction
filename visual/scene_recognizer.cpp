#include "scene_recognizer.h"

#include <boost/filesystem.hpp>

#include <visual/cloud_baker.h>

namespace visual {

void SceneAssetBundle::serializeIntoDirectory(std::string dir_path) const {
	using boost::filesystem::create_directory;
	using boost::filesystem::path;

	create_directory(path(dir_path));
	exterior_mesh.writeWavefrontObject(
		(path(dir_path) / path("exterior_mesh")).string());
}


namespace scene_recognizer {

SceneAssetBundle recognizeScene(const std::vector<SingleScan>& scans) {
	assert(scans.size() > 0);

	SceneAssetBundle bundle;
	bundle.exterior_mesh = visual::CloudBaker(scans[0].old_style).generateRoomMesh();
	return bundle;
}

}  // namespace

}  // namespace

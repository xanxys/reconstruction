#include "scene_asset_bundle.h"

#include <fstream>

namespace recon {

namespace fs = boost::filesystem;

SceneAssetBundle::SceneAssetBundle(
		const std::string& dir_path, bool debug) :
		debug_count(0), collision_count(0),
		dir_path(boost::filesystem::absolute(dir_path)),
		do_finalize(true), debug(debug) {
	cleanDirectory(dir_path);
}

SceneAssetBundle::~SceneAssetBundle() {
	if(do_finalize) {
		serializeIntoDirectory(dir_path);
	} else {
		std::ofstream json_file((dir_path / fs::path("small_data.json")).string());
		json_file << Json::FastWriter().write(serializeSmallData());
	}
	if(debug) {
		serializeWholeScene();
	}
}

bool SceneAssetBundle::isDebugEnabled() const {
	return debug;
}

bool SceneAssetBundle::hasAlignmentCheckpoint() {
	return boost::filesystem::exists(dir_path / cp_alignment_path);
}

Json::Value SceneAssetBundle::getAlignmentCheckpoint() {
	assert(hasAlignmentCheckpoint());
	std::ifstream fs((dir_path / cp_alignment_path).string());
	Json::Value v;
	Json::Reader().parse(fs, v);
	return v;
}

void SceneAssetBundle::setAlignmentCheckpoint(const Json::Value& cp) {
	std::ofstream fs((dir_path / cp_alignment_path).string());
	fs << Json::FastWriter().write(cp);
}

void SceneAssetBundle::cleanDirectory(const fs::path& dir_path) {
	const fs::path cp_dir_path = dir_path / fs::path("checkpoints");
	// Create the directory if there's none.
	if(!fs::exists(dir_path)) {
		fs::create_directory(dir_path);
	}
	// Traverse and remove non-checkpoints files & directories.
	for(auto it = fs::directory_iterator(dir_path);
			it != fs::directory_iterator(); it++) {
		const fs::path& p = *it;
		if(p.filename() == "checkpoints") {
			continue;
		}
		DEBUG("Removing", p.string());
		fs::remove_all(p);
	}
	// Create checkpoint directory if there's nothing.
	if(!fs::exists(cp_dir_path)) {
		fs::create_directory(cp_dir_path);
	}
}

void SceneAssetBundle::serializeIntoDirectory(const fs::path& dir_path) {
	{
		std::ofstream json_file((dir_path / fs::path("small_data.json")).string());
		json_file << Json::FastWriter().write(serializeSmallData());
	}
	int count = 0;
	for(const auto& interior : interior_objects) {
		object_ids.push_back(std::to_string(count));
		const std::string name = "flat_poly_" + std::to_string(count++);
		interior.writeWavefrontObjectFlat((dir_path / name).string());
	}
}

std::string SceneAssetBundle::reservePath(const std::string& filename) {
	return (dir_path / fs::path(filename)).string();
}

Json::Value SceneAssetBundle::loadJson(std::string name) const {
	std::ifstream f_input((dir_path / fs::path(name)).string());
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

void SceneAssetBundle::setExteriorMesh(const TexturedMesh& mesh) {
	exterior_mesh = mesh;
	exterior_mesh.writeWavefrontObject(
		(dir_path / fs::path("exterior_mesh")).string());
}

void SceneAssetBundle::addDebugPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
	const int id = debug_count++;
	std::ofstream debug_points_file((dir_path / fs::path("debug_" + std::to_string(id) + ".ply")).string());
	serializeDebugPoints(cloud).serializePLYWithRgb(debug_points_file);
}

void SceneAssetBundle::addDebugPointCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud) {
	const int id = debug_count++;
	std::ofstream debug_points_file((dir_path / fs::path("debug_" + std::to_string(id) + ".ply")).string());
	serializeDebugPoints(cloud).serializePLYWithRgbNormal(debug_points_file);
}

void SceneAssetBundle::addDebugPointCloud(std::string name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
	std::ofstream debug_points_file((dir_path / fs::path("debug_" + name + ".ply")).string());
	serializeDebugPoints(cloud).serializePLYWithRgb(debug_points_file);
}

void SceneAssetBundle::addDebugPointCloud(std::string name, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud) {
	std::ofstream debug_points_file((dir_path / fs::path("debug_" + name + ".ply")).string());
	serializeDebugPoints(cloud).serializePLYWithRgbNormal(debug_points_file);
}

void SceneAssetBundle::addMesh(std::string name, const TriangleMesh<std::nullptr_t>& mesh) {
	std::ofstream mesh_f((dir_path / fs::path(name + ".ply")).string());
	mesh.serializePLY(mesh_f);
}

void SceneAssetBundle::addMesh(std::string name, const TexturedMesh& mesh) {
	mesh.writeWavefrontObject((dir_path / fs::path(name)).string());
}

void SceneAssetBundle::addMeshFlat(std::string name, const TexturedMesh& mesh) {
	mesh.writeWavefrontObjectFlat((dir_path / fs::path(name)).string());
}

void SceneAssetBundle::serializeWholeScene() const {
	TriangleMesh<std::nullptr_t> whole_scene;
	whole_scene.merge(dropAttrib(exterior_mesh.mesh));
	for(const auto& interior_object : interior_objects) {
		whole_scene.merge(dropAttrib(interior_object.mesh));
	}

	std::ofstream mesh_f((dir_path / fs::path("whole.ply")).string());
	whole_scene.serializePLY(mesh_f);
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

void SceneAssetBundle::addCollisionSoundFromDir(const std::string& src_dir_path) {
	for(auto it = fs::directory_iterator(fs::path(src_dir_path));
			it != fs::directory_iterator(); it++) {
		const fs::path& p_src = *it;
		const std::string name = "collision-" +
			std::to_string(collision_count++) + ".wav";
		fs::copy_file(
			p_src, dir_path / fs::path(name),
			fs::copy_option::overwrite_if_exists);
	}
	INFO("Total added #collisions is now", collision_count);
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
	small_data["collision_count"] = collision_count;
	small_data["accel"] = accel;
	return small_data;
}

}  // namespace

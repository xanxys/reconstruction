#include "scene_asset_bundle.h"

#include <fstream>

namespace recon {

namespace fs = boost::filesystem;

SceneAssetBundle::SceneAssetBundle(
		const std::string& dir_path, bool debug) :
		debug_count(0), collision_count(0),
		z_floor(0),
		dir_path(boost::filesystem::absolute(dir_path)),
		debug(debug) {
	cleanDirectory(dir_path);
}

SceneAssetBundle::~SceneAssetBundle() {
	serializeIntoDirectory(dir_path);
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

void SceneAssetBundle::setFloorLevel(float z_floor) {
	this->z_floor = z_floor;
}

void SceneAssetBundle::addPointLight(const Eigen::Vector3f& pos) {
	point_lights.push_back(pos);
}

void SceneAssetBundle::serializeIntoDirectory(const fs::path& dir_path) {
	Json::Value metadata;
	metadata["unit_per_meter"] = world_scale;
	// Add json-only objects.
	for(const auto& pos_w : point_lights) {
		const Eigen::Vector3f pos_uw = pos_w * world_scale;
		Json::Value light;
		light["pos"]["x"] = pos_uw(0);
		light["pos"]["y"] = pos_uw(1);
		light["pos"]["z"] = pos_uw(2);
		metadata["lights"].append(light);
	}
	metadata["collision_count"] = collision_count;

	// Add json+file data.
	const float ws = world_scale;
	int count = 0;
	auto serializeMesh = [&count, &dir_path, ws](
			const TexturedMesh& tm) {
		// Issue id and path.
		const std::string id = std::to_string(count++);
		const std::string mesh_name = "sm_" + id + ".obj";
		const std::string tex_name = "diffuse_" + id + ".png";
		// Predicted assets name when using
		// IAssetTools::ImportAssets. LoaderPlugin
		// need to make sure asset auto naming convention
		// is kept same.
		const std::string mesh_asset = "sm_" + id;
		const std::string tex_asset = "diffuse_" + id;
		// Write to paths.
		std::ofstream of((dir_path / fs::path(mesh_name)).string());
		TriangleMesh<Eigen::Vector2f> mesh_scaled = tm.mesh;
		for(auto& vert : mesh_scaled.vertices) {
			vert.first *= ws;
		}
		assignNormal(mesh_scaled).serializeObjWithUvNormal(of, "");
		cv::imwrite((dir_path / fs::path(tex_name)).string(), tm.diffuse);

		Json::Value meta_object;
		meta_object["static_mesh"] = mesh_name;
		meta_object["static_mesh:asset"] = mesh_asset;
		meta_object["material"]["diffuse"] = tex_name;
		meta_object["material"]["diffuse:asset"] = tex_asset;
		return meta_object;
	};

	for(const auto& interior : interiors) {
		auto meta_object = serializeMesh(interior.getMesh());
		metadata["interior_objects"].append(meta_object);
	}
	metadata["exterior"] = serializeMesh(exterior_mesh);
	metadata["floor_level"] = world_scale * z_floor;

	{
		std::ofstream json_file((dir_path / fs::path("meta.json")).string());
		json_file << Json::FastWriter().write(metadata);
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

void SceneAssetBundle::addInteriorObject(const InteriorObject& iobj) {
	interiors.push_back(iobj);
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

}  // namespace

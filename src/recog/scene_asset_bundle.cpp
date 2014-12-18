#include "scene_asset_bundle.h"

#include <fstream>

#include <boost/range/irange.hpp>

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
	metadata["lights"] = Json::arrayValue;
	for(const auto& pos_w : point_lights) {
		Json::Value light;
		light["pos"] = serializeLocationWithConversion(pos_w);
		metadata["lights"].append(light);
	}
	metadata["collisions"] = Json::arrayValue;
	for(const int i : boost::irange(0, collision_count)) {
		const std::string id = std::to_string(i);
		Json::Value entry;
		entry["sound"] = "collision_" + id + ".wav";
		entry["sound:asset"] = "collision_" + id;
		metadata["collisions"].append(entry);
	}

	// Add json+file data.
	const float ws = world_scale;
	int count = 0;
	auto serializeMesh = [&count, &dir_path, ws](
			const TexturedMesh& tm) {
		// Issue id and path.
		const std::string id = std::to_string(count++);
		const std::string mesh_path = "sm_" + id + ".obj";
		const std::string tex_path = "diffuse_" + id + ".png";
		const std::string mtl_path = "mat_" + id + ".mtl";
		// Predicted assets name when using
		// IAssetTools::ImportAssets. LoaderPlugin
		// need to make sure asset auto naming convention
		// is kept same.
		const std::string mesh_asset = "sm_" + id;
		const std::string tex_asset = "diffuse_" + id;
		const std::string mtl_asset = "mat_" + id;
		// Write to paths.
		TriangleMesh<Eigen::Vector2f> mesh_scaled = tm.mesh;
		for(auto& vert : mesh_scaled.vertices) {
			vert.first *= ws;
		}
		{
			std::ofstream of((dir_path / fs::path(mesh_path)).string());
			assignNormal(mesh_scaled).serializeObjWithUvNormal(of,
				mtl_path, mtl_asset);
		}
		cv::imwrite((dir_path / fs::path(tex_path)).string(), tm.diffuse);
		{
			std::ofstream of((dir_path / fs::path(mtl_path)).string());
			writeObjMaterial(of, tex_path, mtl_asset);
		}

		Json::Value meta_object;
		meta_object["static_mesh"] = mesh_path;
		meta_object["material"]["diffuse"] = tex_path;
		meta_object["static_mesh:asset"] = mesh_asset;
		meta_object["material:asset"] = mtl_asset;
		meta_object["material"]["diffuse:asset"] = tex_asset;
		return meta_object;
	};

	metadata["interior_objects"] = Json::arrayValue;
	for(const auto& interior : interiors) {
		auto meta_object = serializeMesh(interior.getMesh());
		meta_object["pose"] =
			serializePoseWithConversion(interior.getPose());
		meta_object["collision_boxes"] =
			serializeCollisionShapeWithConversion(interior.getCollision());
		metadata["interior_objects"].append(meta_object);
	}
	// serialize boundary.
	{
		const float col_thickness_w = 0.1;
		if(!boundary) {
			throw std::runtime_error(
				"You must call setInteriorBoundary before trying to serialize!");
		}
		auto meta_bnd = serializeMesh(boundary->getMesh());
		meta_bnd["pose"] =
			serializePoseWithConversion(boundary->getPose());
		meta_bnd["collision_boxes"] =
			serializeCollisionShapeWithConversion(
				boundary->getCollision(col_thickness_w));
		metadata["boundary"] = meta_bnd;
	}
	metadata["floor_level"] = world_scale * z_floor;

	{
		std::ofstream json_file((dir_path / fs::path("meta.json")).string());
		json_file << Json::FastWriter().write(metadata);
	}
}

Json::Value SceneAssetBundle::serializePoseWithConversion(
		const Eigen::Affine3f& transf) {
	const auto trans = transf.translation() * world_scale;
	const auto quat = Eigen::Quaternionf(transf.linear());
	// I think we can leave quaternion (or any other rotation)
	// as is? Because same representation corresponds to
	// inverted rotation in LHS vs. RHS coordinates,
	// and that's ok.
	return serializePose(quat,
		Eigen::Vector3f(
			trans.x(),
			-trans.y(),
			trans.z()));
}

Json::Value SceneAssetBundle::serializePose(const Eigen::Quaternionf& quat, const Eigen::Vector3f& trans) {
	Json::Value pose;
	pose["pos"]["x"] = trans.x();
	pose["pos"]["y"] = trans.y();
	pose["pos"]["z"] = trans.z();
	pose["quat"]["x"] = quat.x();
	pose["quat"]["y"] = quat.y();
	pose["quat"]["z"] = quat.z();
	pose["quat"]["w"] = quat.w();
	return pose;
}

Json::Value SceneAssetBundle::serializeLocationWithConversion(const Eigen::Vector3f& loc) {
	Json::Value p;
	p["x"] = loc.x() * world_scale;
	p["y"] = -loc.y() * world_scale;
	p["z"] = loc.z() * world_scale;
	return p;
}

// Make it FKBoxElem (in UE4)-friendly.
Json::Value SceneAssetBundle::serializeCollisionShapeWithConversion(const std::vector<OBB3f>& obbs) {
	assert(!obbs.empty());
	Json::Value col_shape;
	for(const auto& obb : obbs) {
		const auto ca = obb.getCenterAndAxis();
		Json::Value geom;
		geom["type"] = "OBB";

		Eigen::Matrix3f axis = ca.second;
		const Eigen::Vector3f size(
			axis.col(0).norm(), axis.col(1).norm(), axis.col(2).norm());
		axis.col(0) /= size(0);
		axis.col(1) /= size(1);
		axis.col(2) /= size(2);
		// axis should be rotational now.
		assert(std::abs(axis.determinant() - 1) < 1e-3);

		Eigen::Affine3f trans;
		trans.linear() = axis;
		trans.translation() = ca.first;
		geom["pose"] = serializePoseWithConversion(trans);

		const Eigen::Vector3f size_uw = size * world_scale;
		geom["size"]["x"] = size_uw.x();
		// this is not a vector, don't flip it! (it won't matter anyway because it's cnetered OBB)
		geom["size"]["y"] = size_uw.y();
		geom["size"]["z"] = size_uw.z();

		col_shape.append(geom);
	}
	return col_shape;
}

std::string SceneAssetBundle::reservePath(const std::string& filename) {
	INFO("Someone is trying to access", filename);
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

void SceneAssetBundle::addInteriorObject(const InteriorObject& iobj) {
	if(debug) {
		addMesh(
			"debug_poly_" + std::to_string(interiors.size()),
			iobj.getMesh());
	}
	interiors.push_back(iobj);
}

void SceneAssetBundle::setInteriorBoundary(const InteriorBoundary& ibnd) {
	if(debug) {
		ibnd.getMesh().writeWavefrontObject(
			(dir_path / fs::path("debug_exterior_mesh")).string());
	}
	boundary = ibnd;
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
		const std::string name = "collision_" +
			std::to_string(collision_count++) + ".wav";
		fs::copy_file(
			p_src, dir_path / fs::path(name),
			fs::copy_option::overwrite_if_exists);
	}
	INFO("Total added #collisions is now", collision_count);
}

}  // namespace

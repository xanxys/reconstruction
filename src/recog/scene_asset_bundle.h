#pragma once

#include <map>
#include <string>
#include <tuple>
#include <vector>

// Hack until boost::fs::copy_file is fixed
// https://www.robertnitsch.de/notes/cpp/cpp11_boost_filesystem_undefined_reference_copy_file
#define BOOST_NO_CXX11_SCOPED_ENUMS

#include <boost/filesystem.hpp>
#include <boost/optional.hpp>
#include <Eigen/Dense>
#include <jsoncpp/json/json.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <logging.h>
#include <recog/interior_object.h>
#include <visual/textured_mesh.h>

namespace recon {

// A complete information to be loaded by EqExperiment/LoaderPlugin.
// Also contains bunch of data for debugging purpose.
class SceneAssetBundle {
public:
	// Initialize empty data with output directory.
	// The output directory will be filled by files as data
	// become available.
	SceneAssetBundle(const std::string& dir_path, bool debug);
	~SceneAssetBundle();

	// Checkpoints.
	// These data are opaque Json value to SceneAssetBundle.
	// (i.e. SceneAssetBundle won't perform any kind of verification / sanitization)
	bool hasAlignmentCheckpoint();
	Json::Value getAlignmentCheckpoint();
	void setAlignmentCheckpoint(const Json::Value& cp);

	void addDebugPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
	void addDebugPointCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);

	void addDebugPointCloud(std::string name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
	void addDebugPointCloud(std::string name, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);

	// Coordinate system.
	void setFloorLevel(float z_floor);

	// Add a TexturedMesh corresponding to a single object.
	// Debug results are written immediately to disk,
	// while final results are written at destruction.
	[[deprecated]]
	void addInteriorObject(const TexturedMesh& mesh);
	void addInteriorObject(const InteriorObject& iobj);

	// Set the exterior mesh.
	void setExteriorMesh(const TexturedMesh& mesh);

	// Sound
	void addCollisionSoundFromDir(const std::string& path);

	void addPointLight(const Eigen::Vector3f& pos);

	// Queue to serialize given mesh, create a directory to contain
	// bunch of material/texture files if necessary.
	void addMesh(std::string name, const TriangleMesh<std::nullptr_t>& mesh);
	void addMesh(std::string name, const TexturedMesh& mesh);

	// Reserve given filename and return path for that.
	std::string reservePath(const std::string& filename);

	// Don't create directory and expand multiple files (obj & material & texture)
	// into sigle directory ("flat"-ly).
	void addMeshFlat(std::string name, const TexturedMesh& mesh);

	Json::Value loadJson(std::string name) const;

	bool isDebugEnabled() const;

	// Remove everything other than checkpoints.
	static void cleanDirectory(const boost::filesystem::path& dir_path);
private:
	// Put bunch of files into specified directory.
	// Directory must exist and be empty.
	// Behavior is undefined when the directory already exists.
	// The directory might be nested.
	void serializeIntoDirectory(const boost::filesystem::path& dir_path);

	void serializeWholeScene() const;

	TriangleMesh<Eigen::Vector3f> serializeDebugPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) const;
	TriangleMesh<std::tuple<Eigen::Vector3f, Eigen::Vector3f>> serializeDebugPoints(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud) const;
private:
	// UE4 export settings.
	const float world_scale = 100;  // meter -> uu

	// actual room contents
	float z_floor;
	std::vector<Eigen::Vector3f> point_lights;
	TexturedMesh exterior_mesh;
	std::vector<TexturedMesh> interior_objects;
	std::vector<InteriorObject> interiors;

	// book keeping
	bool debug;
	int debug_count;

	// sound bookkeeping
	int collision_count;

	const boost::filesystem::path dir_path;
	const boost::filesystem::path cp_alignment_path = "checkpoints/alignment.json";
};

}  // namespace

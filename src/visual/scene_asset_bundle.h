#pragma once

#include <map>
#include <string>
#include <tuple>

#include <boost/optional.hpp>
#include <Eigen/Dense>
#include <jsoncpp/json/json.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <logging.h>
#include <visual/textured_mesh.h>

namespace visual {

// A complete information to be loaded by EqExperiment/LoaderPlugin.
// Also contains bunch of data for debugging purpose.
// TODO: might need option to turn throw away debug info, if data
// gets too large.
class SceneAssetBundle {
public:
	// Initialize empty data with output directory.
	// The output directory will be filled by files as data
	// become available.
	SceneAssetBundle(std::string dir_path, bool debug);
	~SceneAssetBundle();

	void addDebugPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
	void addDebugPointCloud(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);

	void addDebugPointCloud(std::string name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
	void addDebugPointCloud(std::string name, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);

	// Add a TexturedMesh corresponding to a single object.
	// Debug results are written immediately to disk,
	// while final results are written at destruction.
	void addInteriorObject(const TexturedMesh& mesh);

	// Queue to serialize given mesh, create a directory to contain
	// bunch of material/texture files if necessary.
	void addMesh(std::string name, const TriangleMesh<std::nullptr_t>& mesh);
	void addMesh(std::string name, const TexturedMesh& mesh);

	// Don't create directory and expand multiple files (obj & material & texture)
	// into sigle directory ("flat"-ly).
	void addMeshFlat(std::string name, const TexturedMesh& mesh);

	Json::Value loadJson(std::string name) const;

	bool isDebugEnabled() const;
private:
	void recreateDirectory(std::string dir_path) const;

	// Put bunch of files into specified directory.
	// Directory must exist and be empty.
	// Behavior is undefined when the directory already exists.
	// The directory might be nested.
	void serializeIntoDirectory(std::string dir_path);

	// Serialize relative small data that nicely fits into a JSON
	// (e.g. <100MB).
	// Don't store image-like things or huge triangle mesh here.
	Json::Value serializeSmallData() const;

	TriangleMesh<Eigen::Vector3f> serializeDebugPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) const;
	TriangleMesh<std::tuple<Eigen::Vector3f, Eigen::Vector3f>> serializeDebugPoints(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud) const;
public:
	// actual room contents
	// TODO: make these private for instant serialziation.
	TexturedMesh exterior_mesh;
	std::vector<Eigen::Vector3f> point_lights;
private:
	std::vector<TexturedMesh> interior_objects;
	// wtf???!
	// this should be derived from interior_objects in serializer,
	// not written temporarily to members!!!
	std::vector<std::string> object_ids;

	// book keeping
	bool debug;
	bool do_finalize;
	int debug_count;
	const std::string dir_path;
};

}  // namespace

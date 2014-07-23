#pragma once

#include <map>
#include <tuple>

#include <Eigen/Dense>
#include <jsoncpp/json/json.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <logging.h>
#include <visual/textured_mesh.h>

namespace visual {

// A complete information to be loaded by EqExperiment/LoaderPlugin.
class SceneAssetBundle {
public:
	// Put bunch of files into specified directory (newly created).
	// Behavior is undefined when the directory already exists.
	// TODO: erase it & re-create. just-like a single file.
	// The directory might be nested.
	void serializeIntoDirectory(std::string dir_path) const;
public:
	TexturedMesh exterior_mesh;
};


class SingleScan {
public:
	Json::Value old_style;
};

// Try avoiding classes for this kind of complex, pure operation.
namespace scene_recognizer {

// Takes several scans of a single room as input (in unordered way),
// and generates a SceneAsssetBundle.
SceneAssetBundle recognizeScene(const std::vector<SingleScan>& scans);

}  // namespace

}  // namespace

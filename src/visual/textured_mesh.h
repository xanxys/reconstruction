#pragma once

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <logging.h>
#include <visual/triangle_mesh.h>

namespace recon {

// A triangle mesh with single diffuse texture.
class TexturedMesh {
public:
	// Write texture, material, geometry into specified directory.
	void writeWavefrontObject(std::string dir_name) const;

	// Write bunch of files with specified prefix.
	// e.g. when prefix = "test/a"
	// * test/a_object.obj
	// * test/a_object.mtl
	// * test/a_diffuse.png
	void writeWavefrontObjectFlat(std::string prefix) const;
public:
	TriangleMesh<Eigen::Vector2f> mesh;
	cv::Mat diffuse;
};

}  // namespace

#pragma once

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <asset.pb.h>
#include <logging.h>
#include <visual/triangle_mesh.h>

namespace visual {

// A triangle mesh with single diffuse texture.
class TexturedMesh {
public:
	void writeWavefrontObject(std::string dir_name) const;
public:
	TriangleMesh<Eigen::Vector2f> mesh;
	cv::Mat diffuse;
};

}  // namespace

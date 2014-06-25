#pragma once

#include <Eigen/Dense>
#include <jsoncpp/json/json.h>
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

// CloudBaker takes a single point cloud and generates
// textured 3D mesh as asset.
class CloudBaker {
public:
	// Convert json-style XYZRGB point cloud.
	CloudBaker(const Json::Value& cloud);

	void writeWavefrontObject();
private:
	TexturedMesh generateRoomMesh();
	static pcl::PointCloud<pcl::PointXYZ>::Ptr decolor(const pcl::PointCloud<pcl::PointXYZRGB>& cloud);
private:
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
};

}  // namespace

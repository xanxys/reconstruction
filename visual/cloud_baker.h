#pragma once

#include <jsoncpp/json/json.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <asset.pb.h>
#include <logging.h>

namespace visual {

// CloudBaker takes a single point cloud and generates
// textured 3D mesh as asset.
class CloudBaker {
public:
	// Convert json-style XYZRGB point cloud.
	CloudBaker(const Json::Value& cloud);

	void writeWavefrontObject();
	// getters.
	//WavefrontObject getWavefrontObject();
private:
	//VirtualFile writeImage(std::string name, const cv::Mat& image);
	static pcl::PointCloud<pcl::PointXYZ>::Ptr decolor(const pcl::PointCloud<pcl::PointXYZRGB>& cloud);
private:
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
};

}  // namespace

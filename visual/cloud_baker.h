#pragma once

#include <jsoncpp/json/json.h>

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
private:
	// Change this to more concrete vars that captures 3D point cloud.
	const Json::Value& cloud;
};

}  // namespace

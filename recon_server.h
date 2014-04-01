#pragma once

#include <memory>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>

#include "analyzer/analyzer.h"
#include "data_source.h"
#include "web_server.h"

// A controller. Two models are OpenNI grabber and SceneBelief.
class ReconServer : public WebServer {
public:
	ReconServer();
	Response handleRequest(std::vector<std::string> uri,
		std::string method, std::string data) override;
private:
	Response handleScene(SceneBelief& belief);
	Response handleGrabcut(SceneBelief& belief, const std::string& data);

	Json::Value serializeCamera(SceneBelief& belief);
	Json::Value serializePoints(SceneBelief& belief);
	Json::Value serializeVoxels(SceneBelief& belief, bool extract_empty);
	Json::Value serializeRGB(SceneBelief& belief);
	Json::Value serializeObjects(SceneBelief& belief);
	Json::Value serializePlanes(SceneBelief& belief);
	Json::Value serializePeeling(SceneBelief& belief);

	// Common serialization utils.
	static Json::Value serialize(const Eigen::Vector3f v);
	static Json::Value serialize(Direction d);

	// Perceptive data conversion.
	static cv::Mat depthToRGB(const cv::Mat& depth);
	
	// Data conversion utils.
	static cv::Mat imageFromDataURL(const std::string& url);
	static std::string dataURLFromImage(const cv::Mat& image);
	static Response sendImage(cv::Mat image);
private:
	DataSource data_source;
};

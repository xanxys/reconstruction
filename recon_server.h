#pragma once

#include <memory>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>

#include "data_source.h"
#include "analyzer.h"
#include "web_server.h"

// A controller. Two models are OpenNI grabber and SceneAnalyzer.
class ReconServer : public WebServer {
public:
	ReconServer();
	Response handleRequest(std::vector<std::string> uri,
		std::string method, std::string data) override;
private:
	Response handleScene(SceneAnalyzer& analyzer);
	Response handleGrabcut(SceneAnalyzer& analyzer, const std::string& data);

	Json::Value serializeCamera(SceneAnalyzer& analyzer);
	Json::Value serializePoints(SceneAnalyzer& analyzer);
	Json::Value serializeVoxels(SceneAnalyzer& analyzer, bool extract_empty);
	Json::Value serializeRGB(SceneAnalyzer& analyzer);
	Json::Value serializeObjects(SceneAnalyzer& analyzer);
	Json::Value serializePlanes(SceneAnalyzer& analyzer);
	Json::Value serializePeeling(SceneAnalyzer& analyzer);

	// Perceptive data conversion.
	static cv::Mat depthToRGB(const cv::Mat& depth);
	
	// Data conversion utils.
	static cv::Mat imageFromDataURL(const std::string& url);
	static std::string dataURLFromImage(const cv::Mat& image);
	static Response sendImage(cv::Mat image);
private:
	DataSource data_source;
};

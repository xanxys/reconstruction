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
	Response handlePoints(SceneAnalyzer& analyzer);
	Response handleVoxels(SceneAnalyzer& analyzer, bool extract_empty);
	Response handleRGB(SceneAnalyzer& analyzer);
	Response handleGrabcut(SceneAnalyzer& analyzer, const std::string& data);
	Response handleObjects(SceneAnalyzer& analyzer);
	Response handlePlanes(SceneAnalyzer& analyzer);

	Response handlePeeling(SceneAnalyzer& analyzer);

	// Data conversion utils.
	static cv::Mat imageFromDataURL(const std::string& url);
	static std::string dataURLFromImage(const cv::Mat& image);
	static Response sendImage(cv::Mat image);
private:
	DataSource data_source;
};

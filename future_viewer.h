#pragma once

#include <vector>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>

#include "datasource.h"
#include "web_server.h"

// A controller. Two models are OpenNI grabber and SceneAnalyzer.
class ReconServer : public WebServer {
public:
	ReconServer();
	Response handleRequest(std::vector<std::string> uri,
		std::string method, std::string data) override;
private:
	Response handlePoints(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud);
	Response handleVoxels(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud);
	Response handleRGB(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud);
	Response handleGrabcut(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud, const std::string& data);
	Response handleObjects(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud);
	
	static cv::Mat imageFromDataURL(const std::string& url);
	static std::string dataURLFromImage(const cv::Mat& image);
	static Response sendImage(cv::Mat image);
private:
	DataSource data_source;
};

#pragma once

// Necessary for OpenNI in PCL.
#define linux 1
#define __x86_64__ 1

#include <chrono>
#include <mutex>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>

#include "web_server.h"

// A controller. Two models are OpenNI grabber and SceneAnalyzer.
class ReconServer : public WebServer {
public:
	ReconServer();
	void cloudCallback(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud);
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
	int new_id;

	// Used to pass point cloud from grabber thread to handler thread.
	std::mutex latest_cloud_lock;
	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr latest_cloud;
	std::map<std::string, pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr> clouds;
};

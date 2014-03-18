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


class VoxelTraversal {
public:
	VoxelTraversal(float size, Eigen::Vector3f org, Eigen::Vector3f dir);
	std::tuple<int, int, int> next();
private:
	const Eigen::Vector3f org;
	const Eigen::Vector3f dir;
	Eigen::Vector3i index;
	Eigen::Vector3f frac;
};


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

	static Response sendImage(cv::Mat image);
private:
	int new_id;

	// Used to pass point cloud from grabber thread to handler thread.
	std::mutex latest_cloud_lock;
	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr latest_cloud;
	std::map<std::string, pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr> clouds;
};

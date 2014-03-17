#pragma once

// Necessary for OpenNI in PCL.
#define linux 1
#define __x86_64__ 1

#include <chrono>
#include <mutex>
#include <vector>

#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "web_server.h"

// PCLVisualizer is useless for cool-looking graphics.
// Stick with point cloud modification.
class FutureViewer {
public:
	FutureViewer();
	void cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud);
	void run();
private:
	//void trackTarget(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud, TrackingTarget& target);
	//std::vector<TrackingTarget> findAllTargets(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud);
private:
	// Used to pass point cloud from grabber thread to main(UI) thread.
	std::mutex latest_cloud_lock;
	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr latest_cloud;

	pcl::visualization::PCLVisualizer visualizer;
};

class ReconServer : public WebServer {
public:
	ReconServer();
	void cloudCallback(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud);
	Response handleRequest(std::vector<std::string> uri,
		std::string method, std::string data) override;
private:
	Response handlePoints();
	Response handleVoxels();
private:
	// Used to pass point cloud from grabber thread to handler thread.
	std::mutex latest_cloud_lock;
	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr latest_cloud;
};

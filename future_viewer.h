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

class TrackingTarget {
public:
	TrackingTarget(Eigen::Vector3f initial_pos);
public:
	// stat (10% blend in)
	Eigen::Vector3f position_avg10;
	std::vector<Eigen::Vector3f> history;

	// snapshot
	std::chrono::time_point<std::chrono::system_clock> time;
	Eigen::Vector3f position;
	Eigen::Vector3f velocity;

	// visualization
	uint8_t r;
	uint8_t g;
	uint8_t b;
};


// PCLVisualizer is useless for cool-looking graphics.
// Stick with point cloud modification.
class FutureViewer {
public:
	FutureViewer();
	void cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud);
	void run();
private:
	void trackTarget(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud, TrackingTarget& target);
	std::vector<TrackingTarget> findAllTargets(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud);
private:
	// Used to pass point cloud from grabber thread to main(UI) thread.
	std::mutex latest_cloud_lock;
	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr latest_cloud;

	pcl::visualization::PCLVisualizer visualizer;
	std::vector<TrackingTarget> targets;
};

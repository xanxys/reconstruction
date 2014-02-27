#pragma once

// Necessary for OpenNI in PCL.
#define linux 1
#define __x86_64__ 1

#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

// PCLVisualizer is useless for cool-looking graphics.
// Stick with point cloud modification.
class FutureViewer {
public:
	FutureViewer();
	void cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud);
	void run();
private:
	bool trackTarget(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud);
	bool findTarget(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud);
private:
	pcl::visualization::CloudViewer viewer;

	bool tracking_ok;
	Eigen::Vector3d target_position;
};

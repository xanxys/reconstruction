#pragma once

#include <chrono>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Load scene from dataset or connected xtion.
class DataSource {
public:
	DataSource(bool enable_xtion);

	// random access + listting
	std::vector<std::string> listScenes();
	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr getScene(std::string id);

	// xtion IO
	std::string takeSnapshot();
	void cloudCallback(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud);
private:
	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr loadFromMSDataset(std::string path);
private:
	int new_id;

	// Used to pass point cloud from grabber thread to handler thread.
	std::mutex latest_cloud_lock;
	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr latest_cloud;
	std::map<std::string, pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr> xtion_clouds;
};

#pragma once

#include <chrono>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


class DataSourceInterface {
public:
	// Return list of ids reasonably fast. (< 10ms)
	virtual std::vector<std::string> listScenes() = 0;

	// Return specified scene data.
	virtual pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr getScene(std::string id) = 0;
};


class MSDataSource : public DataSourceInterface {
public:
	MSDataSource(std::string dataset_path_prefix);
	std::vector<std::string> listScenes() override;
	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr getScene(std::string id) override;
private:
	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr loadFromMSDataset(std::string path);
private:
	const std::string dataset_path_prefix;
};


// Facade of multiple sub-DataSources.
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
	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr loadFromNYU2(std::string path);
private:
	const std::string dataset_path_prefix;
private:  // xtion things
	int new_id;

	// Used to pass point cloud from grabber thread to handler thread.
	std::mutex latest_cloud_lock;
	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr latest_cloud;
	std::map<std::string, pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr> xtion_clouds;
private:  // unified dataset
	std::map<std::string, std::unique_ptr<DataSourceInterface>> sources;
	
};


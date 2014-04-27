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
	// Calling getScene with id not in listScenes results in undefined behavior.
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


class NYU2DataSource : public DataSourceInterface {
public:
	NYU2DataSource(std::string dataset_path_prefix);
	std::vector<std::string> listScenes() override;
	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr getScene(std::string id) override;
private:
	const std::string dataset_path_prefix;
};


class XtionDataSource : public DataSourceInterface {
public:
	XtionDataSource();
	std::vector<std::string> listScenes() override;
	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr getScene(std::string id) override;

	std::string takeSnapshot();
private:
	void cloudCallback(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud);
private:
	int new_id;

	// Used to pass point cloud from grabber thread to handler thread.
	std::mutex latest_cloud_lock;
	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr latest_cloud;
	std::map<std::string, pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr> xtion_clouds;
};


// Facade of multiple sub-DataSources.
// Load scene from dataset or connected xtion.
class DataSource : public DataSourceInterface{
public:
	DataSource(bool enable_xtion);

	// random access + listting
	std::vector<std::string> listScenes() override;
	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr getScene(std::string id) override;

	// xtion IO
	std::string takeSnapshot();
private:
	std::map<std::string, std::unique_ptr<DataSourceInterface>> sources;

	const std::string xtion_prefix;
	
	// optional XtionDataSource (borrowed)
	XtionDataSource* xtion;
};


#include "data_source.h"

#include <exception>

// Necessary for OpenNI in PCL.
#define linux 1
#define __x86_64__ 1

#include <boost/format.hpp>
#include <boost/range/irange.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>

using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using ColorCloud = pcl::PointCloud<pcl::PointXYZRGBA>;
using RigidTrans3f = Eigen::Transform<float, 3, Eigen::AffineCompact>;


MSDataSource::MSDataSource(std::string dataset_path_prefix) :
	dataset_path_prefix(dataset_path_prefix) {
}

std::vector<std::string> MSDataSource::listScenes() {
	const std::vector<std::string> places = {
		"chess",
		"heads",
		"stairs",
		"fire",
		"office",
		"pumpkin",
		"redkitchen",
	};

	std::vector<std::string> list;
	for(const auto& place : places) {
		list.push_back(place + "-1");
		list.push_back(place + "-100");
		list.push_back(place + "-200");
		list.push_back(place + "-300");
	}
	return list;
}

pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr MSDataSource::getScene(std::string name_and_frame) {
	const int index = name_and_frame.find("-");
	const std::string name = name_and_frame.substr(0, index);
	const int frame = std::stoi(name_and_frame.substr(index + 1));

	return loadFromMSDataset(dataset_path_prefix +
		"/" + name + "-1/" + boost::str(boost::format("frame-%06d") % frame));
}

// Load http://research.microsoft.com/en-us/projects/7-scenes/
// Quote:
// Please note: The RGB and depth camera have not been calibrated
// and we can't provide calibration parameters at the moment.
// The recorded frames correspond to the raw, uncalibrated camera images.
// In the KinectFusion pipeline we used the following default intrinsics
// for the depth camera: Principle point (320,240), Focal length (585,585).
ColorCloud::ConstPtr MSDataSource::loadFromMSDataset(std::string path) {
	cv::Mat rgb = cv::imread(path +".color.png", CV_LOAD_IMAGE_COLOR);
	cv::Mat depth_mm = cv::imread(path +".depth.png", CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_GRAYSCALE);

	if(rgb.size() != cv::Size(640, 480) || depth_mm.size() != cv::Size(640, 480)) {
		throw std::runtime_error("Invalid format");
	}

	// Camera parameter
	const float cx = 320;
	const float cy = 240;
	const float fx = 585;
	const float fy = 585;

	ColorCloud::Ptr cloud(new ColorCloud());
	cloud->width = 640;
	cloud->height = 480;
	for(int y : boost::irange(0, 480)) {
		for(int x : boost::irange(0, 640)) {
			pcl::PointXYZRGBA pt;
			const uint16_t depth = depth_mm.at<uint16_t>(y, x);
			if(depth == 0xffff || depth < 10) {
				pt.x = std::nan("");
				pt.y = std::nan("");
				pt.z = std::nan("");
			} else {
				pt.z = depth / 1000.0;

				pt.x = (x - cx) / fx * pt.z;
				pt.y = (y - cy) / fy * pt.z;
			}

			const auto color = rgb.at<cv::Vec3b>(y, x);
			pt.r = color[0];
			pt.g = color[1];
			pt.b = color[2];

			cloud->points.push_back(pt);
		}
	}
	assert(cloud->points.size() == 640 * 480);
	return cloud;
}


NYU2DataSource::NYU2DataSource(std::string dataset_path_prefix) :
	dataset_path_prefix(dataset_path_prefix) {
}

std::vector<std::string> NYU2DataSource::listScenes() {
	std::vector<std::string> list = {
		"test"
	};
	return list;
}

pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr NYU2DataSource::getScene(std::string name_and_frame) {
	const std::string depth_path = dataset_path_prefix + "/basement_0001a/d-1316653600.562170-2521435723.pgm";
	const std::string rgb_path = dataset_path_prefix + "/basement_0001a/r-1316653581.608081-1384510396.ppm";

	cv::Mat depth_map = cv::imread(depth_path, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_GRAYSCALE);
	assert(depth_map.type() == CV_16U);

	cv::Mat rgb = cv::imread(rgb_path, CV_LOAD_IMAGE_COLOR);
	assert(rgb.type() == CV_8UC3);

	// TODO: Use values from NYU2 toolbox matlab code.
	// Camera parameter
	// http://cs.nyu.edu/~silberman/code/toolbox_nyu_depth_v2.zip

	// Depth Intrinsic Parameters
	const float fx_d = 5.8262448167737955e+02;
	const float fy_d = 5.8269103270988637e+02;
	const float cx_d = 3.1304475870804731e+02;
	const float cy_d = 2.3844389626620386e+02;

	// Parameters for making depth absolute.
	const float depthParam1 = 351.3;
	const float depthParam2 = 1092.5;

	ColorCloud::Ptr cloud(new ColorCloud());
	cloud->width = 640;
	cloud->height = 480;
	for(int y : boost::irange(0, 480)) {
		for(int x : boost::irange(0, 640)) {
			pcl::PointXYZRGBA pt;
			uint16_t depth_raw = depth_map.at<uint16_t>(y, x);
			depth_raw = ((depth_raw & 0xff) << 8) + (depth_raw >> 8);  // swap endian

			const float depth = depthParam1 / (depthParam2 - depth_raw);

			if(depth < 0.1 || depth > 10) {
				pt.x = std::nan("");
				pt.y = std::nan("");
				pt.z = std::nan("");
			} else {
				pt.z = depth;

				pt.x = (x - cx_d) / fx_d * pt.z;
				pt.y = (y - cy_d) / fy_d * pt.z;
			}

			const auto color = rgb.at<cv::Vec3b>(y, x);
			pt.r = color[0];
			pt.g = color[1];
			pt.b = color[2];

			cloud->points.push_back(pt);
		}
	}
	assert(cloud->points.size() == 640 * 480);
	return cloud;
}


XtionDataSource::XtionDataSource() : new_id(0) {
	pcl::Grabber* source = new pcl::OpenNIGrabber();

	boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
		boost::bind(&XtionDataSource::cloudCallback, this, _1);

	source->registerCallback(f);
	source->start();
}

void XtionDataSource::cloudCallback(const ColorCloud::ConstPtr& cloud) {
	{
		std::lock_guard<std::mutex> lock(latest_cloud_lock);
		latest_cloud = cloud;
	}
}

std::vector<std::string> XtionDataSource::listScenes() {
	std::vector<std::string> list;
	for(const auto& cloud : xtion_clouds) {
		list.push_back(cloud.first);
	}
	return list;
}

pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr XtionDataSource::getScene(std::string id) {
	return xtion_clouds[id];
}

std::string XtionDataSource::takeSnapshot() {
	const std::string id = std::to_string(new_id++);
	{
		std::lock_guard<std::mutex> lock(latest_cloud_lock);
		// TODO: might need copying (if the grabber reuses cloud every frame)
		xtion_clouds[id] = latest_cloud;
	}
	return id;
}


DataSource::DataSource(bool enable_xtion) : xtion_prefix("xtion") {
	const std::string dataset_path_prefix = "/data-new/research/2014/reconstruction";

	sources["MS"].reset(new MSDataSource(dataset_path_prefix + "/MS"));
	sources["NYU"].reset(new NYU2DataSource(dataset_path_prefix + "/NYU2"));

	if(enable_xtion) {
		xtion = new XtionDataSource();
		sources[xtion_prefix].reset(xtion);
	} else {
		xtion = nullptr;
	}
}

std::vector<std::string> DataSource::listScenes() {
	std::vector<std::string> list;
	for(const auto& source : sources) {
		for(const std::string& sub_id : source.second->listScenes()) {
			list.push_back(source.first + "-" + sub_id);
		}
	}
	return list;
}

pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr DataSource::getScene(std::string id) {
	const int index = id.find("-");
	assert(index != std::string::npos);

	const std::string source_prefix = id.substr(0, index);
	const std::string source_id = id.substr(index + 1);

	return sources[source_prefix]->getScene(source_id);
}

std::string DataSource::takeSnapshot() {
	assert(xtion);
	return xtion_prefix + "-" + xtion->takeSnapshot();
}

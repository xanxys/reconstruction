#include "data_source.h"

#include <exception>

// Necessary for OpenNI in PCL.
#define linux 1
#define __x86_64__ 1

#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <boost/optional.hpp>
#include <boost/range/irange.hpp>
#include <opencv2/opencv.hpp>
#ifdef ENABLE_USB_IO
#include <pcl/io/openni_grabber.h>
#endif
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>

namespace server {

using boost::filesystem::path;

using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using ColorCloud = pcl::PointCloud<pcl::PointXYZRGBA>;
using RigidTrans3f = Eigen::Transform<float, 3, Eigen::AffineCompact>;

MSDataSource::MSDataSource(std::string dataset_path_prefix) :
	dataset_path_prefix(dataset_path_prefix) {
	if(!boost::filesystem::exists(dataset_path_prefix) || !boost::filesystem::is_directory(dataset_path_prefix)) {
		throw std::runtime_error("Failed to find MS dataset");
	}
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

	for(const auto& p : boost::make_iterator_range(
		boost::filesystem::directory_iterator(path(dataset_path_prefix)),
		boost::filesystem::directory_iterator())) {
		names.push_back(boost::filesystem::basename(p));
	}
}

std::vector<std::string> NYU2DataSource::listScenes() {
	return names;
}

pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr NYU2DataSource::getScene(std::string name) {
	const auto pair = getFramePair(name);
	const std::string rgb_path = pair.first;
	const std::string depth_path = pair.second;

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

std::pair<std::string, std::string> NYU2DataSource::getFramePair(std::string name) {
	// File name examples:
	// RGB: r-1316558342.822162-2944600483.ppm
	// depth: d-1316558366.843700-90569730.pgm
	// It seems like [rd]-(epoch second)-(unknown parg).(ppm|pgm)
	//
	// Try to find a pair that the times won't differ too much.
	const double time_tolerance = 0.1;
	const path scene_dir(dataset_path_prefix + "/" + name);

	// Find an arbitrary RGB frame.
	boost::optional<path> rgb;
	for(const auto& p : boost::make_iterator_range(
		boost::filesystem::directory_iterator(scene_dir),
		boost::filesystem::directory_iterator())) {

		if(boost::filesystem::extension(p) == ".ppm") {
			rgb = p;
			break;
		}
	}
	if(!rgb) {
		throw std::runtime_error("Corrupt NYU2 database in " + name);
	}
	const double rgb_time = extractTime(*rgb);

	// Find a depth frame that's close enough to the RGB frame.
	boost::optional<path> depth;
	double nearest_distance = 1e3;
	for(const auto& p : boost::make_iterator_range(
		boost::filesystem::directory_iterator(scene_dir),
		boost::filesystem::directory_iterator())) {

		if(boost::filesystem::extension(p) != ".pgm") {
			continue;
		}

		// Get narest frame.
		const double depth_time = extractTime(p);
		const double distance = std::abs(rgb_time - depth_time);
		if(distance < nearest_distance) {
			nearest_distance = distance;
			depth = p;
		}
	}

	if(!(rgb && depth)) {
		throw std::runtime_error("Corrupt NYU2 database in " + name);
	}

	// Check if close enough.
	if(nearest_distance >= time_tolerance) {
		throw std::runtime_error("Temporally close frame pair not found");
	}

	return std::make_pair(rgb->string(), depth->string());
}

double NYU2DataSource::extractTime(const boost::filesystem::path& path_file) {
	const std::string name = boost::filesystem::basename(path_file);
	std::vector<std::string> parts;
	boost::split(parts, name, boost::is_any_of("-"));

	// Parts must be type, second, unknown.
	if(parts.size() != 3) {
		throw std::runtime_error("Unknown NYUv2 frame path encourtered");
	}

	return std::stod(parts[1]);
}


#ifdef ENABLE_USB_IO
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
#endif


DataSource::DataSource(bool enable_xtion) : xtion_prefix("xtion") {
	const std::string dataset_path_prefix = "/data-new/research/2014/reconstruction";

	// Don't write like sources["MS"].reset(...); they'll put null-pointer when
	// DataSource creation fails!
	try {
		auto ptr = new MSDataSource(dataset_path_prefix + "/MS");
		sources["MS"].reset(ptr);
	} catch(...) {
	}

	try {
		auto ptr = new NYU2DataSource(dataset_path_prefix + "/NYU2-slice");
		sources["NYU"].reset(ptr);
	} catch(...) {
	}

	#ifdef ENABLE_USB_IO
	xtion = nullptr;
	if(enable_xtion) {
		try {
			xtion = new XtionDataSource();
			sources[xtion_prefix].reset(xtion);
		} catch(pcl::PCLIOException exc) {
			// Give up xtion when IO error occurs.
		}
	}
	#endif
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

	if(sources[source_prefix]) {
		return sources[source_prefix]->getScene(source_id);
	} else {
		throw std::runtime_error("Scene not found");
	}
}

std::string DataSource::takeSnapshot() {
	#ifdef ENABLE_USB_IO
	assert(xtion);
	return xtion_prefix + "-" + xtion->takeSnapshot();
	#else
	throw std::runtime_error("Xtion support is not compiled");
	#endif
}

}  // namespace

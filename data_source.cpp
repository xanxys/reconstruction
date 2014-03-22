#include "data_source.h"

#include <exception>

// Necessary for OpenNI in PCL.
#define linux 1
#define __x86_64__ 1

#include <boost/range/irange.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>

using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using ColorCloud = pcl::PointCloud<pcl::PointXYZRGBA>;
using RigidTrans3f = Eigen::Transform<float, 3, Eigen::AffineCompact>;

DataSource::DataSource(bool enable_xtion) : new_id(0) {
	if(enable_xtion) {
		pcl::Grabber* source = new pcl::OpenNIGrabber();

		boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
			boost::bind(&DataSource::cloudCallback, this, _1);

		source->registerCallback(f);
		source->start();
	}
}

std::vector<std::string> DataSource::listScenes() {
	std::vector<std::string> list = {
		"MS-chess-1",
		"MS-heads-1",
		"MS-stairs-1",
		"MS-fire-1",
		"MS-office-1",
		"MS-pumpkin-1",
		"MS-redkitchen-1",
	};

	for(const auto& cloud : xtion_clouds) {
		list.push_back(cloud.first);
	}

	return list;
}

pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr DataSource::getScene(std::string id) {
	if(id.size() >= 3 && id.substr(0, 3) == "MS-") {
		return loadFromMSDataset("data/MS/" + id.substr(3) + "/frame-000000");
	} else {
		if(xtion_clouds.find(id) != xtion_clouds.end()) {
			return xtion_clouds[id];
		}
	}

	throw std::runtime_error("Specified scene not found");
}

std::string DataSource::takeSnapshot() {
	const std::string id = std::to_string(new_id++);
	{
		std::lock_guard<std::mutex> lock(latest_cloud_lock);
		// TODO: might need copying (if the grabber reuses cloud every frame)
		xtion_clouds[id] = latest_cloud;
	}
	return id;
}

void DataSource::cloudCallback(const ColorCloud::ConstPtr& cloud) {
	{
		std::lock_guard<std::mutex> lock(latest_cloud_lock);
		latest_cloud = cloud;
	}
}

float unsafeParseFloat(const uint8_t* ptr) {
	uint32_t v =
	(((uint32_t)ptr[3]) << 24) | (((uint32_t)ptr[2]) << 16) |
	(((uint32_t)ptr[1]) << 8) | (((uint32_t)ptr[0]));

	return *reinterpret_cast<const float*>(ptr);
}

float unsafeParseFloatBE(const uint8_t* ptr) {
	uint32_t v =
	(((uint32_t)ptr[0]) << 24) | (((uint32_t)ptr[1]) << 16) |
	(((uint32_t)ptr[2]) << 8) | (((uint32_t)ptr[3]));

	return *reinterpret_cast<const float*>(&v);
}

ColorCloud::ConstPtr loadFromCornellDataset(std::string path) {
	sensor_msgs::PointCloud2::Ptr cloud_org(new sensor_msgs::PointCloud2());
	Eigen::Vector4f camera_pos;
	Eigen::Quaternionf camera_rot;
	if(pcl::io::loadPCDFile(path, *cloud_org, camera_pos, camera_rot) < 0) {
		throw std::runtime_error("PCD load failed");
	}

	// Convert to our format.
	if(cloud_org->width * cloud_org->height != 640 * 480) {
		throw std::runtime_error("Unexpected frame size (you need to use single-frame dataset)");
	}

	std::cout << "IsBigEndian " << static_cast<bool>(cloud_org->is_bigendian) << std::endl;
	for(const auto& field : cloud_org->fields) {
		std::cout << "Field " << field.name << " @" << field.offset << " ::" << static_cast<int>(field.datatype) << std::endl;
	}

	ColorCloud::Ptr cloud(new ColorCloud());
	//pcl::fromROSMsg(*cloud_org, *cloud);
	cloud->width = 640;
	cloud->height = 480;
	//return cloud;
	const uint64_t stride = 48;
	const uint64_t offset_x = 19;
	const uint64_t offset_y = 15;
	const uint64_t offset_z = 11;
	const int offset_r = 16;
	const int offset_g = 20;
	const int offset_b = 18;

	if(cloud_org->data.size() != 640 * 480 * stride) {
		throw std::runtime_error("Unexpected point format (data size=" +
			std::to_string(cloud_org->data.size()));
	}

	for(int i : boost::irange(0, 640 * 480)) {
		pcl::PointXYZRGBA pt;
		pt.x = unsafeParseFloat(cloud_org->data.data() + (i * stride + offset_x));
		pt.y = unsafeParseFloat(cloud_org->data.data() + (i * stride + offset_y));
		pt.z = unsafeParseFloat(cloud_org->data.data() + (i * stride + offset_z));

		// +3 is good. +2,+1 shows some random pattern. +0 alamost black. (Z)
		// bigendian float?
		//pt.r = *(cloud_org->data.data() + (i * stride + offset_z + 3 + 8));
		//pt.g = *(cloud_org->data.data() + (i * stride + offset_z + 3 + 4));
		

		float v = 
		*(cloud_org->data.data() + (i * stride + 8 + 4)) * 255.0 +
		*(cloud_org->data.data() + (i * stride + 8 + 3));

		pt.b = v / 2;


		//std::cout << "LE " << pt.x << std::endl;
		//std::cout << "BE " << unsafeParseFloatBE(cloud_org->data.data() + (i * stride + offset_x)) << std::endl;
		cloud->points.push_back(pt);
	}

	return cloud;
}

// Load http://research.microsoft.com/en-us/projects/7-scenes/
// Quote:
// Please note: The RGB and depth camera have not been calibrated
// and we can't provide calibration parameters at the moment.
// The recorded frames correspond to the raw, uncalibrated camera images.
// In the KinectFusion pipeline we used the following default intrinsics
// for the depth camera: Principle point (320,240), Focal length (585,585).
ColorCloud::ConstPtr DataSource::loadFromMSDataset(std::string path) {
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

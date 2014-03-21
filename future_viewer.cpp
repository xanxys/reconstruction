#include "future_viewer.h"

#include <map>
#include <cmath>
#include <exception>
#include <iostream>
#include <sstream>
#include <random>

#include <boost/range/irange.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <sensor_msgs/PointCloud2.h>

#include "analyzer.h"
#include "base64.h"

using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using ColorCloud = pcl::PointCloud<pcl::PointXYZRGBA>;
using RigidTrans3f = Eigen::Transform<float, 3, Eigen::AffineCompact>;

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

ReconServer::ReconServer() : new_id(0) {
	/*
	pcl::Grabber* source = new pcl::OpenNIGrabber();

	boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
	boost::bind(&ReconServer::cloudCallback, this, _1);

	source->registerCallback(f);
	source->start();
	*/

	latest_cloud = loadFromCornellDataset("data/home_or_office/scene1_0.pcd");
}

void ReconServer::cloudCallback(const ColorCloud::ConstPtr& cloud) {
	{
		std::lock_guard<std::mutex> lock(latest_cloud_lock);
		latest_cloud = cloud;
	}
}

Response ReconServer::handleRequest(std::vector<std::string> uri,
	std::string method, std::string data) {

	if(uri.size() == 0) {
		return sendStaticFile("/index.html", "text/html");
	} else if(uri.size() == 2 && uri[0] == "static") {
		return sendStaticFile(uri[1]);
	} else if(uri.size() == 1 && uri[0] == "at" && method == "POST") {
		const std::string id = std::to_string(new_id++);
		{
			std::lock_guard<std::mutex> lock(latest_cloud_lock);
			// TODO: might need copying (if the grabber reuses cloud every frame)
			clouds[id] = latest_cloud;
		}
		Json::Value v;
		v["id"] = id;
		return v;
	} else if(uri.size() >= 3 && uri[0] == "at") {
		const std::string id = uri[1];

		if(clouds.find(id) == clouds.end()) {
			return Response::notFound();
		}

		const auto& cloud = clouds.find(id)->second;

		if(uri[2] == "points") {
			return handlePoints(cloud);
		} else if(uri[2] == "voxels") {
			return handleVoxels(cloud);
		} else if(uri[2] == "rgb") {
			return handleRGB(cloud);
		} else if(uri[2] == "grabcut" && method == "POST") {
			return handleGrabcut(cloud, data);
		} else if(uri[2] == "objects") {
			return handleObjects(cloud);
		}

		return Response::notFound();
	}
	return Response::notFound();
}

Response ReconServer::handlePoints(const ColorCloud::ConstPtr& cloud) {
	SceneAnalyzer analyzer(cloud);

	Json::Value vs;
	for(const auto& pt : analyzer.getCloud()->points) {
		if(!std::isfinite(pt.x)) {
			continue;
		}
		
		Json::Value p;
		p["x"] = pt.x;
		p["y"] = pt.y;
		p["z"] = pt.z;
		p["r"] = pt.r / 255.0;
		p["g"] = pt.g / 255.0;
		p["b"] = pt.b / 255.0;
		vs.append(p);
	}
	return Response(vs);
}

Response ReconServer::handleVoxels(const ColorCloud::ConstPtr& cloud) {
	SceneAnalyzer analyzer(cloud);

	Json::Value root;
	for(const auto& pair : analyzer.getVoxels()) {
		if(pair.second != VoxelState::OCCUPIED) {
			continue;
		}
		
		Json::Value vx;
		vx["x"] = std::get<0>(pair.first);
		vx["y"] = std::get<1>(pair.first);
		vx["z"] = std::get<2>(pair.first);
		root.append(vx);
	}
	return Response(root);
}

Response ReconServer::handleRGB(const ColorCloud::ConstPtr& cloud) {
	return sendImage(SceneAnalyzer(cloud).getRGBImage());
}

Response ReconServer::handleGrabcut(const ColorCloud::ConstPtr& cloud, const std::string& data) {
	Json::Value root;
	Json::Reader().parse(data, root);
	const cv::Mat mask = imageFromDataURL(root["image"].asString());
	if(!mask.data) {
		return Response(400, "Invalid image", "text/plain");
	}
	const cv::Mat image = SceneAnalyzer(cloud).getRGBImage();

	if(mask.size() != image.size()) {
		return Response(400, "Invalid image size", "text/plain");
	}

	cv::Mat mask_parsed(mask.rows, mask.cols, CV_8UC1);
	for(int y : boost::irange(0, mask.rows)) {
		for(int x : boost::irange(0, mask.cols)) {
			const auto color = mask.at<cv::Vec3b>(y, x);

			uint8_t val = cv::GC_PR_BGD;
			if(color[0] > 127) {
				// blueish -> FG
				val = cv::GC_FGD;
			} else if(color[2] > 127) {
				// reddish -> BG
				val = cv::GC_BGD;
			}
			mask_parsed.at<uint8_t>(y, x) = val;
		}
	}

	cv::Mat bgd_model;
	cv::Mat fgd_model;
	cv::grabCut(image, mask_parsed, cv::Rect(), bgd_model, fgd_model, 3, cv::GC_INIT_WITH_MASK);
	
	cv::Mat image_clipped(mask.rows, mask.cols, CV_8UC3);
	for(int y : boost::irange(0, mask.rows)) {
		for(int x : boost::irange(0, mask.cols)) {
			const uint8_t flag = mask_parsed.at<uint8_t>(y, x);

			if(flag == cv::GC_FGD || flag == cv::GC_PR_FGD) {
				image_clipped.at<cv::Vec3b>(y, x) = image.at<cv::Vec3b>(y, x);
			}
		}
	}

	Json::Value result;
	result["image"] = dataURLFromImage(image_clipped);
	return Response(result);
}

Response ReconServer::handleObjects(const ColorCloud::ConstPtr& cloud) {
	SceneAnalyzer analyzer(cloud);
	Json::Value result = analyzer.getObjects();
	return Response(result);
}

Response ReconServer::sendImage(cv::Mat image) {
	std::vector<uchar> buffer;
	cv::imencode(".jpeg", image, buffer);

	std::string buffer_s(buffer.begin(), buffer.end());
	return Response(buffer_s, "image/jpeg");
}


cv::Mat ReconServer::imageFromDataURL(const std::string& data_url) {
	const std::string binary = base64_decode(data_url.substr(data_url.find(",") + 1));
	const std::vector<uint8_t> blob(binary.begin(), binary.end());

	return cv::imdecode(blob, CV_LOAD_IMAGE_COLOR);
}

std::string ReconServer::dataURLFromImage(const cv::Mat& image) {
	std::vector<uchar> buffer;
	cv::imencode(".jpeg", image, buffer);
	const std::string buffer_s(buffer.begin(), buffer.end());

	return "data:image/jpeg;base64," +
	base64_encode(reinterpret_cast<const uint8_t*>(buffer_s.data()), buffer_s.size());
}

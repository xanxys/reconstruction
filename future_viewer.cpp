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
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "base64.h"

using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using ColorCloud = pcl::PointCloud<pcl::PointXYZRGBA>;
using RigidTrans3f = Eigen::Transform<float, 3, Eigen::AffineCompact>;

class ColorSet {
public:
	ColorSet();
	void next();
	void apply(pcl::PointXYZRGBA& pt);
private:
	int index;
	std::vector<std::vector<int>> cycle;
};

ColorSet::ColorSet() : index(0), cycle(
{
	{255, 0, 0},
	{127, 127, 0},
	{0, 255, 0},
	{0, 127, 127},
	{0, 0, 255},
	{127, 0, 127}
}) {
}

void ColorSet::next() {
	index = (index + 1) % cycle.size();
}

void ColorSet::apply(pcl::PointXYZRGBA& pt) {
	pt.r = cycle[index][0];
	pt.g = cycle[index][1];
	pt.b = cycle[index][2];
}



ColorCloud::Ptr generateQuad(float sx, float sy, const RigidTrans3f& trans) {
	float resolution = 0.005;
	ColorCloud::Ptr cloud(new ColorCloud());

	for(int iy = - sy / 2 / resolution; iy < sy / 2 / resolution; iy++) {
		for(int ix = - sx / 2 / resolution; ix < sx / 2 / resolution; ix++) {
			Eigen::Vector3f pos(ix * resolution, iy * resolution, 0);
			pos = trans * pos;

			pcl::PointXYZRGBA pt;
			pt.x = pos.x();
			pt.y = pos.y();
			pt.z = pos.z();

			if(ix % 100 == 0 || iy % 100 == 0) {
				pt.r = 0;
				pt.g = 0;
				pt.b = 0;
			} else {
				pt.r = 255;
				pt.g = 255;
				pt.b = 255;
			}
			pt.a = 255;
			cloud->points.push_back(pt);
		}
	}

	return cloud;
}

ColorCloud::Ptr concat(ColorCloud::Ptr c0, ColorCloud::Ptr c1) {
	ColorCloud::Ptr cloud(new ColorCloud());
	cloud->points.insert(cloud->points.end(), c0->points.begin(), c0->points.end());
	cloud->points.insert(cloud->points.end(), c1->points.begin(), c1->points.end());
	return cloud;
}


VoxelTraversal::VoxelTraversal(float size, Eigen::Vector3f org, Eigen::Vector3f dir) :
org(org / size), dir(dir) {
	index = Eigen::Vector3i(
		std::floor(org(0)),
		std::floor(org(1)),
		std::floor(org(2)));

	frac = org - index.cast<float>();
}

std::tuple<int, int, int> VoxelTraversal::next() {
	const auto key = std::make_tuple(index(0), index(1), index(2));

	const auto remaining = Eigen::Vector3f(
		(dir(0) < 0) ? -frac(0) : 1 - frac(0),
		(dir(1) < 0) ? -frac(1) : 1 - frac(1),
		(dir(2) < 0) ? -frac(2) : 1 - frac(2));

	const auto dt_xyz = remaining.cwiseQuotient(dir);

	// Select the direction with smallest dt. (tie-breaker: prefer X>Y>Z)
	if(dt_xyz(0) <= dt_xyz(1) && dt_xyz(0) <= dt_xyz(2)) {
		const int dix = (dir(0) < 0) ? -1 : 1;
		index(0) += dix;
		frac += dt_xyz(0) * dir;
		frac(0) -= dix;
	} else if(dt_xyz(1) <= dt_xyz(2)) {
		const int dix = (dir(1) < 0) ? -1 : 1;
		index(1) += dix;
		frac += dt_xyz(1) * dir;
		frac(1) -= dix;
	} else {
		const int dix = (dir(2) < 0) ? -1 : 1;
		index(2) += dix;
		frac += dt_xyz(2) * dir;
		frac(2) -= dix;
	}

	return key;
}

ReconServer::ReconServer() : new_id(0) {
	pcl::Grabber* source = new pcl::OpenNIGrabber();

	boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
	boost::bind(&ReconServer::cloudCallback, this, _1);

	source->registerCallback(f);
	source->start();
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
		}

		return Response::notFound();
	}
	return Response::notFound();
}

Response ReconServer::handlePoints(const ColorCloud::ConstPtr& cloud) {
	Json::Value vs;

	for(const auto& pt : cloud->points) {
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
	Json::Value root;


	const float size = 0.1;

	// known to be filled
	std::map<std::tuple<int, int, int>, bool> voxels;
	for(const auto& pt : cloud->points) {
		if(!std::isfinite(pt.x)) {
			continue;
		}

		auto ix = pt.getVector3fMap() / size;
		auto key = std::make_tuple(
			static_cast<int>(std::floor(ix.x())),
			static_cast<int>(std::floor(ix.y())),
			static_cast<int>(std::floor(ix.z())));
		voxels[key] = true;
	}

	const auto& camera_origin = Eigen::Vector3f::Zero();

	std::map<std::tuple<int, int, int>, bool> voxels_empty;
	for(const auto& pair_filled : voxels) {
		// cast ray from camera
		const auto pos = Eigen::Vector3f(
			std::get<0>(pair_filled.first) + 0.5,
			std::get<1>(pair_filled.first) + 0.5,
			std::get<2>(pair_filled.first) + 0.5) * size;

		const auto dir = (pos - camera_origin).normalized();

		// traverse until hit.
		VoxelTraversal traversal(size, camera_origin, dir);
		for(int i : boost::irange(0, 100)) {
			const auto key = traversal.next();

			// Hit wall.
			if(voxels.find(key) != voxels.end()) {
				break;
			}

			voxels_empty[key] = true;
		}
	}

	std::map<std::tuple<int, int, int>, bool> voxels_unknown;
	for(int ix : boost::irange(-12, 12)) {
		for(int iy : boost::irange(-12, 12)) {
			for(int iz : boost::irange(10, 35)) {
				const auto key = std::make_tuple(ix, iy, iz);

				if(voxels.find(key) == voxels.end() && voxels_empty.find(key) == voxels_empty.end()) {
					voxels_unknown[key] = true;
				}
			}
		}
	}

	for(const auto& pair : voxels) {
		Json::Value vx;
		vx["x"] = std::get<0>(pair.first);
		vx["y"] = std::get<1>(pair.first);
		vx["z"] = std::get<2>(pair.first);
		root.append(vx);
	}
	return Response(root);
}

Response ReconServer::handleRGB(const ColorCloud::ConstPtr& cloud) {
	return sendImage(extractImageFromPointCloud(cloud));
}

Response ReconServer::sendImage(cv::Mat image) {
	std::vector<uchar> buffer;
	cv::imencode(".jpeg", image, buffer);

	std::string buffer_s(buffer.begin(), buffer.end());
	return Response(buffer_s, "image/jpeg");
}

Response ReconServer::handleGrabcut(const ColorCloud::ConstPtr& cloud, const std::string& data) {
	Json::Value root;
	Json::Reader().parse(data, root);
	const std::string data_url = root["image"].asString();
	const std::string binary = base64_decode(data_url.substr(data_url.find(",") + 1));
	const std::vector<uint8_t> blob(binary.begin(), binary.end());

	const cv::Mat mask = cv::imdecode(blob, CV_LOAD_IMAGE_COLOR);
	if(!mask.data) {
		return Response(400, "Invalid image", "text/plain");
	}
	const cv::Mat image = extractImageFromPointCloud(cloud);

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

	std::vector<uchar> buffer;
	cv::imencode(".jpeg", image_clipped, buffer);
	const std::string buffer_s(buffer.begin(), buffer.end());

	Json::Value result;
	result["image"] = "data:image/jpeg;base64," +
		base64_encode(reinterpret_cast<const uint8_t*>(buffer_s.data()), buffer_s.size());
	return Response(result);
}

cv::Mat ReconServer::extractImageFromPointCloud(
	const ColorCloud::ConstPtr& cloud) {
	if(cloud->points.size() != cloud->width * cloud->height) {
		throw std::runtime_error("Point cloud is not an image");
	}

	cv::Mat rgb(cloud->height, cloud->width, CV_8UC3);
	for(int y : boost::irange(0, (int)cloud->height)) {
		for(int x : boost::irange(0, (int)cloud->width)) {
			const pcl::PointXYZRGBA& pt = cloud->points[y * cloud->width + x];
			rgb.at<cv::Vec3b>(y, x) = cv::Vec3b(pt.b, pt.g, pt.r);
		}
	}
	return rgb;
}

#include "recon_server.h"

#include <map>
#include <cmath>
#include <exception>
#include <iostream>
#include <sstream>

#include <boost/range/irange.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/point_types.h>

#include "base64.h"

using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using ColorCloud = pcl::PointCloud<pcl::PointXYZRGBA>;
using RigidTrans3f = Eigen::Transform<float, 3, Eigen::AffineCompact>;


ReconServer::ReconServer() : data_source(true) {
}

Response ReconServer::handleRequest(std::vector<std::string> uri,
	std::string method, std::string data) {

	if(uri.size() == 0) {
		return sendStaticFile("/index.html", "text/html");
	} else if(uri.size() == 2 && uri[0] == "static") {
		return sendStaticFile(uri[1]);
	} else if(uri.size() == 1 && uri[0] == "at" && method == "POST") {
		Json::Value v;
		v["id"] = data_source.takeSnapshot();
		return v;
	} else if(uri.size() == 2 && uri[0] == "at") {
		const std::string id = uri[1];
		const auto& cloud = data_source.getScene(id);
		SceneAnalyzer analyzer(cloud);

		return handleScene(analyzer);
	} else if(uri.size() >= 3 && uri[0] == "at") {
		const std::string id = uri[1];
		const auto& cloud = data_source.getScene(id);
		SceneAnalyzer analyzer(cloud);

		if(uri[2] == "grabcut" && method == "POST") {
			return handleGrabcut(analyzer, data);
		}

		return Response::notFound();
	} else if(uri.size() == 1 && uri[0] == "scenes") {
		Json::Value scenes;
		for(const auto& id : data_source.listScenes()) {
			Json::Value entry;
			entry["id"] = id;

			scenes.append(entry);
		}
		return scenes;
	}
	return Response::notFound();
}

Response ReconServer::handleScene(SceneAnalyzer& analyzer) {
	Json::Value scene;
	scene["points"] = serializePoints(analyzer);
	scene["voxels"] = serializeVoxels(analyzer, false);
	scene["voxels_empty"] = serializeVoxels(analyzer, true);
	scene["rgb"] = dataURLFromImage(analyzer.getRGBImage());
	scene["objects"] = serializeObjects(analyzer);
	scene["planes"] = serializePlanes(analyzer);
	scene["peeling"] = serializePeeling(analyzer);

	return Response(scene);
}

Json::Value ReconServer::serializePoints(SceneAnalyzer& analyzer) {
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
	return vs;
}

Json::Value ReconServer::serializeVoxels(SceneAnalyzer& analyzer, bool extract_empty) {
	Json::Value root;
	for(const auto& pair : analyzer.getVoxels()) {
		if(extract_empty) {
			if(pair.second != VoxelState::EMPTY) {
				continue;
			}
		} else {
			if(pair.second != VoxelState::OCCUPIED) {
				continue;
			}
		}
		
		Json::Value vx;
		vx["x"] = std::get<0>(pair.first);
		vx["y"] = std::get<1>(pair.first);
		vx["z"] = std::get<2>(pair.first);
		root.append(vx);
	}
	return root;
}

Response ReconServer::handleGrabcut(SceneAnalyzer& analyzer, const std::string& data) {
	Json::Value root;
	Json::Reader().parse(data, root);
	const cv::Mat mask = imageFromDataURL(root["image"].asString());
	if(!mask.data) {
		return Response(400, "Invalid image", "text/plain");
	}
	const cv::Mat image = analyzer.getRGBImage();

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

Json::Value ReconServer::serializeObjects(SceneAnalyzer& analyzer) {
	Json::Value result = analyzer.getObjects();
	return result;
}

Json::Value ReconServer::serializePlanes(SceneAnalyzer& analyzer) {
	const auto plane = analyzer.getPlanes()[0];

	Json::Value plane_s;
	plane_s["y"] = plane.y_offset;
	plane_s["tex"] = dataURLFromImage(plane.texture);

	Json::Value result;
	result["planes"] = plane_s;
	return result;
}

Json::Value ReconServer::serializePeeling(SceneAnalyzer& analyzer) {
	Json::Value peeling;

	peeling["target"] = dataURLFromImage(analyzer.getRGBImage());
	peeling["render"] = dataURLFromImage(analyzer.renderRGBImage());

	return peeling;
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
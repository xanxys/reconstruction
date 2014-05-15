#include "recon_server.h"

#include <cmath>
#include <exception>
#include <iostream>
#include <map>
#include <sstream>

#include <boost/range/irange.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/point_types.h>

#include "base64.h"

using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using ColorCloud = pcl::PointCloud<pcl::PointXYZRGBA>;
using RigidTrans3f = Eigen::Transform<float, 3, Eigen::AffineCompact>;


ReconServer::ReconServer() : data_source(true), job_pool(data_source) {
}

Response ReconServer::handleRequest(std::vector<std::string> uri,
	std::string method, std::string data) {

	if(uri.size() == 0) {
		return sendStaticFile("/index.html", "text/html");
	} else if(uri[0] == "static" && uri.size() == 2) {
		return sendStaticFile(uri[1]);
	} else if(uri[0] == "scene") {
		return handleSceneRequest(
			std::vector<std::string>(uri.begin() + 1, uri.end()), method, data);
	} else if(uri[0] == "job") {
		return handleJobRequest(
			std::vector<std::string>(uri.begin() + 1, uri.end()), method, data);
	}
	return Response::notFound();
}

Response ReconServer::handleSceneRequest(const std::vector<std::string> sub_uri,
	const std::string& method, const std::string& data) {
	if(sub_uri.size() == 0 && method == "GET") {
		Json::Value scenes(Json::arrayValue);
		for(const auto& id : data_source.listScenes()) {
			Json::Value entry;
			entry["id"] = id;

			scenes.append(entry);
		}
		return scenes;
	} else if(sub_uri.size() == 0 && method == "POST") {
		Json::Value v;
		v["id"] = data_source.takeSnapshot();
		return v;
	} else if(sub_uri.size() >= 1) {
		const std::string scene_id = sub_uri[0];
		const auto& cloud = data_source.getScene(scene_id);
		SceneAnalyzer analyzer(cloud);

		if(sub_uri.size() == 1 && method == "GET") {
			return handleScene(analyzer);
		} else if(sub_uri.size() == 2 && sub_uri[1] == "grabcut" && method == "POST") {
			return handleGrabcut(*analyzer.getBestBelief(), data);
		} else if(sub_uri.size() == 3 && sub_uri[1] == "belief" && method == "GET") {
			auto beliefs = analyzer.getAllBelief();
			const int belief_id = std::stoi(sub_uri[2]);
			if(belief_id < 0 || belief_id >= beliefs.size()) {
				return Response::notFound();
			}
			return handleBelief(*beliefs[belief_id]);
		}
	}
	return Response::notFound();
}

Response ReconServer::handleJobRequest(const std::vector<std::string> sub_uri,
	const std::string& method, const std::string& data) {

	if(sub_uri.size() == 0 && method == "GET") {
		Json::Value jobs;
		for(const auto& job_id : job_pool.listJobs()) {
			jobs.append(job_pool.getJobDescription(job_id));
		}
		return jobs;
	} else if(sub_uri.size() == 0 && method == "POST") {
		const std::string id = job_pool.createJob();
		Json::Value status;
		status["id"] = id;
		return Response(status);
	} else if(sub_uri.size() == 1 && method == "GET") {
		Json::Value desc = job_pool.getJobDescription(sub_uri[0]);
		return Response(desc);
	}
	return Response::notFound();
}

Response ReconServer::handleScene(SceneAnalyzer& analyzer) {
	auto belief_ptr = analyzer.getBestBelief();
	auto& belief = *belief_ptr;

	Json::Value scene;
	scene["camera"] = serializeCamera(belief);
	scene["points"] = serializePoints(belief);
	scene["voxels"] = serializeVoxels(belief, false);
	scene["voxels_empty"] = serializeVoxels(belief, true);
	scene["rgb"] = dataURLFromImage(belief.getRGBImage());
	scene["depth"] = dataURLFromImage(depthToRGB(belief.getDepthImage()));
	scene["objects"] = serializeObjects(belief);
	scene["planes"] = serializePlanes(belief);
	scene["peeling"] = serializePeeling(belief);
	scene["log"] = belief.getLog();

	for(int index : boost::irange(0, (int)analyzer.getAllBelief().size())) {
		scene["candidates"].append(index);
	}

	return Response(scene);
}

Response ReconServer::handleBelief(SceneBelief& belief) {
	Json::Value scene;
	scene["camera"] = serializeCamera(belief);
	scene["points"] = serializePoints(belief);
	scene["voxels"] = serializeVoxels(belief, false);
	scene["voxels_empty"] = serializeVoxels(belief, true);
	scene["rgb"] = dataURLFromImage(belief.getRGBImage());
	scene["depth"] = dataURLFromImage(depthToRGB(belief.getDepthImage()));
	scene["objects"] = serializeObjects(belief);
	scene["planes"] = serializePlanes(belief);
	scene["peeling"] = serializePeeling(belief);
	scene["log"] = belief.getLog();
	return Response(scene);
}

Json::Value ReconServer::serializeCamera(SceneBelief& belief) {
	const Eigen::Quaternionf rot(belief.getCameraLocalToWorld());

	Json::Value pose;
	pose["x"] = rot.x();
	pose["y"] = rot.y();
	pose["z"] = rot.z();
	pose["w"] = rot.w();
	return pose;
}

Json::Value ReconServer::serializePoints(SceneBelief& belief) {
	Json::Value vs;
	for(const auto& pt : belief.getCloud()->points) {
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

Json::Value ReconServer::serializeVoxels(SceneBelief& belief, bool extract_empty) {
	Json::Value root;
	for(const auto& pair : belief.getVoxelsDetailed()) {
		if(extract_empty) {
			if(pair.second.state != VoxelState::EMPTY) {
				continue;
			}
		} else {
			if(pair.second.state != VoxelState::OCCUPIED) {
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

Response ReconServer::handleGrabcut(SceneBelief& belief, const std::string& data) {
	Json::Value root;
	Json::Reader().parse(data, root);
	const cv::Mat mask = imageFromDataURL(root["image"].asString());
	if(!mask.data) {
		return Response(400, "Invalid image", "text/plain");
	}
	const cv::Mat image = belief.getRGBImage();

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

Json::Value ReconServer::serializeObjects(SceneBelief& belief) {
	Json::Value result;
	for(const OrientedBox& box : belief.getObjects()) {
		Json::Value object;
		object["pos"] = serialize(box.getPosition());
		object["ry"] = box.getRotationY();
		object["size"] = serialize(box.getSize());
		object["valid"] = box.getValid();
		object["r"] = box.getColor()(0);
		object["g"] = box.getColor()(1);
		object["b"] = box.getColor()(2);
		result.append(object);
	}
	return result;
}

Json::Value ReconServer::serializePlanes(SceneBelief& belief) {
	Json::Value result;
	for(const auto& plane : belief.getPlanes()) {
		Json::Value plane_s;
		plane_s["center"] = serialize(plane.center);
		plane_s["size"] = plane.size;
		plane_s["normal"] = serialize(plane.normal);
		plane_s["tex"] = dataURLFromImage(plane.texture);
		result.append(plane_s);
	}
	return result;
}

Json::Value ReconServer::serializePeeling(SceneBelief& belief) {
	Json::Value peeling;

	const cv::Mat target = belief.getRGBImage();
	const cv::Mat render = belief.renderRGBImage();

	cv::Mat delta;
	cv::absdiff(target, render, delta);

	peeling["target"] = dataURLFromImage(target);
	peeling["render"] = dataURLFromImage(render);
	peeling["delta"] = dataURLFromImage(delta);
	peeling["l1"] = SceneAnalyzer::getScore(belief);

	return peeling;
}

Json::Value ReconServer::serialize(const Eigen::Vector3f v) {
	Json::Value j;
	j["x"] = v.x();
	j["y"] = v.y();
	j["z"] = v.z();
	return j;
}

Json::Value ReconServer::serialize(Direction d) {
	switch(d) {
		case Direction::XP: return "x+";
		case Direction::XN: return "x-";
		case Direction::YP: return "y+";
		case Direction::YN: return "y-";
		case Direction::ZP: return "z+";
		case Direction::ZN: return "z-";
		default: assert(false);
	}
}

cv::Mat ReconServer::depthToRGB(const cv::Mat& depth) {
	assert(depth.type() == CV_32F);

	cv::Mat visible(depth.rows, depth.cols, CV_8UC3);
	for(int y : boost::irange(0, depth.rows)) {
		for(int x : boost::irange(0, depth.cols)) {
			const float d = depth.at<float>(y,x);
			const uint8_t v = std::min(255, static_cast<int>(d / 2.5 * 255.0));
			visible.at<cv::Vec3b>(y, x) = cv::Vec3b(v, v, v);
		}
	}
	return visible;
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

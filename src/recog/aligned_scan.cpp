#include "scene_recognizer.h"

#include <fstream>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/optional.hpp>
#include <boost/range/irange.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <math_util.h>
#include <program_proxy.h>
#include <recog/shape_fitter.h>
#include <third/ICP.h>
#include <visual/cloud_baker.h>
#include <visual/cloud_base.h>
#include <visual/film.h>
#include <visual/mapping.h>
#include <visual/mesh_intersecter.h>
#include <visual/texture_conversion.h>

namespace recon {

SingleScan::SingleScan(const std::string& scan_dir) {
	using boost::filesystem::path;
	INFO("Loading a scan from", scan_dir);
	std::ifstream f_input((path(scan_dir) / path("points.json")).string());
	if(!f_input.is_open()) {
		throw std::runtime_error("Cloudn't open points.json");
	}
	Json::Value cloud_json;
	Json::Reader().parse(f_input, cloud_json);

	// extract scan id (basename)
	std::vector<std::string> components;
	boost::split(components, scan_dir, boost::is_any_of("/"));
	if(components.empty() || components.back() == "") {
		throw std::runtime_error("Couldn't extract valid scan id");
	}
	scan_id = components.back();

	INFO("Parsing JSON");
	cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
	for(const auto& point : cloud_json) {
		pcl::PointXYZRGBNormal pt;
		pt.x = point["x"].asDouble();
		pt.y = point["y"].asDouble();
		pt.z = point["z"].asDouble();
		pt.r = point["r"].asDouble();
		pt.g = point["g"].asDouble();
		pt.b = point["b"].asDouble();
		pt.normal_x = point["nx"].asDouble();
		pt.normal_y = point["ny"].asDouble();
		pt.normal_z = point["nz"].asDouble();
		cloud->points.push_back(pt);
	}

	INFO("Loading equirectangular images");
	er_rgb = cv::imread((path(scan_dir) / path("rgb.png")).string());
	er_intensity = cv::imread((path(scan_dir) / path("intensity.png")).string(), CV_LOAD_IMAGE_ANYDEPTH);
	cv::Mat er_depth_mm = cv::imread((path(scan_dir) / path("depth.png")).string(), CV_LOAD_IMAGE_ANYDEPTH);

	if(er_rgb.size() == cv::Size(0, 0)) {
		WARN("Image was not loaded properly");
		throw std::runtime_error("Couldn't load ER RGB image");
	}
	if(er_rgb.size() != er_intensity.size() || er_rgb.size() != er_depth_mm.size()) {
		throw std::runtime_error("Inconsistent image sizes");
	}
	if(er_rgb.type() != CV_8UC3 || er_depth_mm.type() != CV_16U || er_intensity.type() != CV_16U) {
		INFO("correct types", CV_8UC3, CV_16U, CV_16U);
		INFO("found types", er_rgb.type(), er_depth_mm.type(), er_intensity.type());
		throw std::runtime_error("Invalid image type");
	}
	// Convert u16 depth(mm) back to float depth(m).
	er_depth_mm.convertTo(er_depth, CV_32F, 1e-3);
	assert(er_depth.type() == CV_32F);
}

std::string SingleScan::getScanId() const {
	return scan_id;
}


CorrectedSingleScan::CorrectedSingleScan(
		const SingleScan& raw_scan,
		const Eigen::Affine3f& local_to_world,
		const Eigen::Vector3f& color_multiplier) :
		raw_scan(raw_scan), local_to_world(local_to_world),
		color_multiplier(color_multiplier) {
}

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr CorrectedSingleScan::getCloudInWorld() const {
	return applyTransform(raw_scan.cloud, local_to_world);
}

AlignedScans::AlignedScans(SceneAssetBundle& bundle,
		const std::vector<SingleScan>& scans,
		const Json::Value& hint) {
	assert(!scans.empty());
	loadInitialPoses(hint["initial_pose"], scans);
	if(bundle.hasAlignmentCheckpoint()) {
		loadCheckpoint(bundle.getAlignmentCheckpoint());
	} else {
		finealignToTarget(hint["align_to"].asString());
		assert(scans.size() == scans_with_pose.size());
		bundle.setAlignmentCheckpoint(saveCheckpoint());
	}
	correctColor();
	applyLeveling();
}

Json::Value AlignedScans::saveCheckpoint() const {
	Json::Value cp;
	for(const auto& swp : scans_with_pose) {
		cp[std::get<0>(swp).getScanId()] = encodeAffine(std::get<1>(swp));
	}
	return cp;
}

void AlignedScans::loadCheckpoint(const Json::Value& cp) {
	for(auto& swp : scans_with_pose) {
		std::get<1>(swp) = decodeAffine(cp[std::get<0>(swp).getScanId()]);
	}
}

void AlignedScans::loadInitialPoses(const Json::Value& initial_pose, const std::vector<SingleScan>& scans) {
	// Load pose json (scan_id -> local_to_world)
	std::map<std::string, Eigen::Affine3f> poses;
	{
		for(auto& entry : initial_pose) {
			poses[entry["scan_id"].asString()] =
				decodeAffine(entry["affine"]);
		}
	}
	// Apply predefined pose.
	for(const auto& scan : scans) {
		const std::string scan_id = scan.getScanId();
		if(poses.find(scan_id) == poses.end()) {
			throw std::runtime_error("Pose undefined for scan_id=" + scan_id);
		}
		scans_with_pose.push_back(std::make_tuple(
			scan, poses[scan_id], Eigen::Vector3f(1, 1, 1)));
	}
}

Json::Value AlignedScans::encodeAffine(const Eigen::Affine3f& affine) {
	Eigen::Matrix4f m = affine.matrix();
	Json::Value json;
	for(int i : boost::irange(0, 4)) {
		Json::Value row;
		for(int j : boost::irange(0, 4)) {
			row.append(m(i, j));
		}
		json.append(row);
	}
	return json;
}

Eigen::Affine3f AlignedScans::decodeAffine(const Json::Value& json, bool require_rigid) {
	if(json.size() != 4) {
		throw std::runtime_error("Invalid affine transform (#rows !=4)");
	}
	Eigen::Matrix4f m;
	for(int i : boost::irange(0, 4)) {
		if(json[i].size() != 4) {
			throw std::runtime_error("Invalid affine transform (#cols != 4)");
		}
		for(int j : boost::irange(0, 4)) {
			m(i, j) = json[i][j].asDouble();
		}
	}
	Eigen::Affine3f trans(m);
	if(require_rigid) {
		if(std::abs(trans.linear().determinant() - 1) > 1e-3) {
			throw std::runtime_error(
				"Given affine transform is not a rigid transform");
		}
	}
	return trans;
}

void AlignedScans::finealignToTarget(const std::string& fine_align_target_id) {
	assert(!scans_with_pose.empty());
	// Find alignment target and conver it to world coordinate.
	boost::optional<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> fine_align_target;
	for(const auto& swp : scans_with_pose) {
		if(std::get<0>(swp).getScanId() != fine_align_target_id) {
			continue;
		}
		fine_align_target = applyTransform(
			std::get<0>(swp).cloud, std::get<1>(swp));
		break;
	}
	if(!fine_align_target) {
		throw std::runtime_error("Fine-alignment target not found. id=" + fine_align_target_id);
	}

	// fine-align to target.
	std::vector<std::tuple<SingleScan, Eigen::Affine3f, Eigen::Vector3f>> new_scans_with_pose;
	for(auto& scan_with_pose : scans_with_pose) {
		auto fine_align = Eigen::Affine3f::Identity();
		if(std::get<0>(scan_with_pose).getScanId() != fine_align_target_id) {
			INFO("Fine-aligning scan", std::get<0>(scan_with_pose).getScanId());
			fine_align = finealign(
				*fine_align_target,
				applyTransform(std::get<0>(scan_with_pose).cloud, std::get<1>(scan_with_pose)));
		}
		new_scans_with_pose.push_back(std::make_tuple(
			std::get<0>(scan_with_pose),
			fine_align * std::get<1>(scan_with_pose),
			Eigen::Vector3f(1, 1, 1)));
	}
	scans_with_pose = std::move(new_scans_with_pose);
}

void AlignedScans::createClosenessMatrix(SceneAssetBundle& bundle, const std::vector<SingleScan>& scans) const {
	const int n = scans.size();
	INFO("Calculating point cloud closeness matrix");
	Eigen::MatrixXf closeness(n, n);
	for(int i : boost::irange(0, n)) {
		for(int j : boost::irange(i, n)) {
			const auto pre_align = prealign(scans[j], scans[i]);

			const float dist = cloudDistance(
				applyTransform(scans[i].cloud, pre_align),
				scans[j].cloud);

			closeness(i, j) = dist;
			closeness(j, i) = dist;
		}
	}
	// serialize to json.
	Json::Value rows;
	for(int i : boost::irange(0, n)) {
		Json::Value row;
		for(int j : boost::irange(0, n)) {
			row.append(closeness(i, j));
		}
		rows.append(row);
	}
	DEBUG("closeness matrix", rows);

	// do hierarchical aggregation.
	// will be O(N^2) + small constant * O(N^3)
	// O(N^3) can be removed by some clever data structure,
	// but since N is small (<100) it's needless optimization.
	int new_id = n;
	std::map<int, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> remaining;
	std::map<std::pair<int, int>, float> sparse_dist;  // always i < j
	// node = (i, (j, k, trans))
	//   i
	// -----
	// |   |
	// j  trans * k
	std::map<int, std::tuple<int, int, Eigen::Affine3f>> nodes;

	// initialize.
	INFO("Begin hierarchical aggregation of point clouds");
	for(int i : boost::irange(0, n)) {
		remaining[i] = scans[i].cloud;
		for(int j : boost::irange(i + 1, n)) {
			sparse_dist[std::make_pair(i, j)] = closeness(i, j);
		}
	}
	// merge until only one node remains.
	int count_merge = 0;
	while(remaining.size() > 1) {
		const auto best_pair_org = std::min_element(sparse_dist.begin(), sparse_dist.end(),
			[](const std::pair<std::pair<int, int>, float>& a, const std::pair<std::pair<int, int>, float>& b) {
				return a.second < b.second;
			}
		);
		const auto best_dist = best_pair_org->second;
		const auto best_pair = best_pair_org->first;
		// Create new node(agg_id) from best_pair.
		const int agg_id = new_id++;
		INFO("Pair:", best_pair.first, best_pair.second, "->", agg_id, best_dist);

		// Align.
		const auto pre_align = prealign(remaining[best_pair.second], remaining[best_pair.first]);
		const auto fine_align = finealign(remaining[best_pair.second], applyTransform(remaining[best_pair.first], pre_align));
		const Eigen::Affine3f trans = fine_align * pre_align;
		nodes[agg_id] = std::make_tuple(best_pair.second, best_pair.first, trans);
		remaining[agg_id] = merge<pcl::PointXYZRGBNormal>(
			remaining[best_pair.second],
			applyTransform(remaining[best_pair.first], trans));
		remaining.erase(best_pair.first);
		remaining.erase(best_pair.second);

		// debug dump
		bundle.addDebugPointCloud(remaining[agg_id]);

		DEBUG("Updating distances");
		// Remove obsolete distances.
		// (pairs containing either first or second)
		for(const auto& pair : sparse_dist) {
			if(pair.first.first == best_pair.first || pair.first.second == best_pair.first ||
				pair.first.first == best_pair.second || pair.first.second == best_pair.second) {
				sparse_dist.erase(pair.first);
			}
		}
		// Add new distances.
		// agg_id - other
		for(const auto& key : remaining) {
			if(key.first == agg_id) {
				continue;
			}
			const auto new_pair = (key.first > agg_id) ?
				std::make_pair(agg_id, key.first) :
				std::make_pair(key.first, agg_id);
			const int i = new_pair.first;
			const int j = new_pair.second;
			assert(i < j);
			const auto pre_align = prealign(remaining[j], remaining[i]);
			const float dist = cloudDistance(
				applyTransform(remaining[i], pre_align),
				remaining[j]);
			sparse_dist[new_pair] = dist;
		}
	}
	assert(remaining.size() == 1);
}

void AlignedScans::hierarchicalMerge(const std::vector<SingleScan>& scans) {
}

void AlignedScans::correctColor() {
	// At multiple regions indexed by l,
	// calculate average color for each scan(i),
	// call this vector C(i, l).
	// Also, let N(i, l) be number of points at the region.
	// Let M(i) be color multiplier for scan i.
	INFO("Correcting color");
	// calculate avg color.
	// location_index -> scan_index -> (num_samples, avg_color)
	std::vector<std::vector<std::pair<int, Eigen::Vector3f>>> samples;
	// take seeds from each scan's point.
	for(const auto& s_w_p : scans_with_pose) {
		auto delta = applyTransform(std::get<0>(s_w_p).cloud, std::get<1>(s_w_p));
		const Eigen::Vector3f org = delta->points[0].getVector3fMap();
		const float radius = 0.5;
		DEBUG("Adding test location", org(0), org(1), org(2), radius);

		std::vector<std::pair<int, Eigen::Vector3f>> sample;
		DEBUG("Calculating avg color");
		for(const auto& s_w_p : scans_with_pose) {
			auto delta = applyTransform(std::get<0>(s_w_p).cloud, std::get<1>(s_w_p));
			int n = 0;
			Eigen::Vector3f color_accum = Eigen::Vector3f::Zero();
			for(const auto& pt : delta->points) {
				if((pt.getVector3fMap() - org).norm() > radius) {
					continue;
				}

				color_accum += Eigen::Vector3f(pt.r, pt.g, pt.b);
				n++;
			}
			if(n > 0) {
				color_accum /= n;
			}
			Json::Value info;
			info["n_sample"] = n;
			info["avg_col"].append(color_accum(0));
			info["avg_col"].append(color_accum(1));
			info["avg_col"].append(color_accum(2));
			DEBUG("Avg col", info);
			sample.emplace_back(n, color_accum);
		}
		samples.push_back(sample);
	}
	// Convert samples to json.
	assert(!samples.empty());
	Json::Value samples_json;
	for(const auto& sample : samples) {
		Json::Value sample_json;
		for(const auto& pair : sample) {
			Json::Value pair_json;
			Json::Value color;
			color.append(pair.second(0));
			color.append(pair.second(1));
			color.append(pair.second(2));
			pair_json.append(color);
			pair_json.append(pair.first);
			sample_json.append(pair_json);
		}
		samples_json.append(sample_json);
	}
	Json::Value v = call_external("extpy/color_correct.py", samples_json);
	DEBUG("colorCorrect:result", v);
	if(v.size() != scans_with_pose.size()) {
		throw std::runtime_error("Invalid color multipliers returned");
	}
	for(int i : boost::irange(0, (int)scans_with_pose.size())) {
		std::get<2>(scans_with_pose[i]) = Eigen::Vector3f(
			v[i][0].asDouble(), v[i][1].asDouble(), v[i][2].asDouble());
	}
}

void AlignedScans::applyLeveling() {
	// Extract upper points (possible and ceiling).
	const auto merged = getMergedPoints();
	const float margin = 0.5;
	float max_z = -1e3;
	for(const auto& pt : merged->points) {
		max_z = std::max(max_z, pt.z);
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr edges(new pcl::PointCloud<pcl::PointXYZRGB>);
	for(const auto& pt : merged->points) {
		if(pt.z > max_z - margin) {
			edges->points.push_back(pt);
		}
	}

	// Fit plane
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01);

	seg.setInputCloud(edges);
	seg.segment(*inliers, *coefficients);

	if(inliers->indices.size() == 0) {
		WARN ("Could not estimate a planar model for the given dataset.");
		return;
	}

	INFO("Leveling plane:",
		coefficients->values[0], coefficients->values[1],
		coefficients->values[2], coefficients->values[3]);
	const float angle_thresh = 10.0 / 180 * pi;
	auto target_normal = Eigen::Vector3f::UnitZ();
	auto normal = Eigen::Vector3f(
		coefficients->values[0], coefficients->values[1], coefficients->values[2]).normalized();
	if(normal.dot(target_normal) < 0) {
		normal *= -1;
	}
	// If it's ceiling, adjust it. Otherwise, ignore it.
	if(normal.dot(target_normal) < std::cos(angle_thresh)) {
		return;
	}
	INFO("Adjusting rotation");
	const auto rot = normal.cross(target_normal);
	if(rot.norm() < 1e-3) {
		INFO("Rotation too small; keeping as is");
		return;
	}
	const float rot_angle = std::asin(rot.norm());
	INFO("Rotating by", rot_angle);
	const Eigen::Affine3f trans(
		Eigen::AngleAxisf(rot_angle, rot.normalized()));
	std::vector<std::tuple<SingleScan, Eigen::Affine3f, Eigen::Vector3f>> new_scans_with_pose;
	for(auto& scan_w_pose : scans_with_pose) {
		new_scans_with_pose.push_back(std::make_tuple(
			std::get<0>(scan_w_pose),
			trans * std::get<1>(scan_w_pose),
			Eigen::Vector3f(1, 1, 1)));
	}
	scans_with_pose = std::move(new_scans_with_pose);
}

Eigen::Affine3f AlignedScans::prealign(const SingleScan& target, const SingleScan& source) {
	return prealign(target.cloud, source.cloud);
}

Eigen::Affine3f AlignedScans::prealign(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr target,
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr source) {
	assert(target);
	assert(source);
	std::vector<Eigen::Affine3f> seeds;
	const int n_rot = 60;
	for(int i : boost::irange(0, n_rot)) {
		seeds.push_back(
			Eigen::Affine3f(Eigen::AngleAxisf(
				(float)i / n_rot * 2 * pi,
				Eigen::Vector3f::UnitZ())));
	}

	auto extract_centroid = [](pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud) {
		std::vector<float> xs;
		std::vector<float> ys;
		for(auto& pt : cloud->points) {
			xs.push_back(pt.x);
			ys.push_back(pt.y);
		}
		return Eigen::Vector2f(
			mean(robustMinMax(xs, 0.01)),
			mean(robustMinMax(ys, 0.01)));
	};

	// source --seed--> source' --centroid-align--> source'' ==? target
	float best_dist = 1e6;
	Eigen::Affine3f best_trans;
	for(const auto& seed_trans : seeds) {
		const auto cloud_seeded = applyTransform(source, seed_trans);
		const auto dp = extract_centroid(target) - extract_centroid(cloud_seeded);
		const Eigen::Affine3f trans_centroid(Eigen::Translation3f(Eigen::Vector3f(dp(0), dp(1), 0)));

		const float dist = cloudDistance(applyTransform(cloud_seeded, trans_centroid), target);
		DEBUG("Dist(angle)", dist);
		if(dist < best_dist) {
			best_dist = dist;
			best_trans = trans_centroid * seed_trans;
		}
	}
	INFO("Pre-align: best dist=", best_dist);
	return best_trans;
}

Eigen::Affine3f AlignedScans::finealign(const SingleScan& target, const SingleScan& source) {
	return finealign(target.cloud, source.cloud);
}

Eigen::Affine3f AlignedScans::finealign(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr target,
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr source) {
	assert(target);
	assert(source);

	// 0: target
	// 1,2,..: source
	// return: (position matrix, normal matrix)
	auto to_matrix = [](pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud) {
		const auto ds_cloud = downsample<pcl::PointXYZRGBNormal>(cloud, 0.05);
		const int n = ds_cloud->points.size();
		Eigen::Matrix3Xd mat(3, n);
		Eigen::Matrix3Xd mat_n(3, n);
		for(int i : boost::irange(0, n)) {
			mat.col(i) = ds_cloud->points[i].getVector3fMap().cast<double>();
			mat_n.col(i) = ds_cloud->points[i].getNormalVector3fMap().cast<double>();
		}
		return std::make_pair(mat, mat_n);
	};

	auto to_cloud = [](pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud) {
		// return downsample<pcl::PointXYZRGBNormal>(cloud, 0.05);
		return cloud;
	};

	INFO("Running PCL ICP");
	pcl::IterativeClosestPoint<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp;
	icp.setMaxCorrespondenceDistance(0.3);  // need to be larger than pre-alignment error
	icp.setMaximumIterations(25);
	icp.setTransformationEpsilon(0);  // disable this criteria
	icp.setEuclideanFitnessEpsilon(0);  // disable this criteria
	icp.setInputCloud(to_cloud(source));
	icp.setInputTarget(to_cloud(target));
	pcl::PointCloud<pcl::PointXYZRGBNormal> final;
	icp.align(final);
	INFO("ICP converged", icp.hasConverged(), "score", icp.getFitnessScore());
	Eigen::Affine3f fine_align(icp.getFinalTransformation());
	INFO("Fine Affine |t|=", fine_align.translation().norm());
	return fine_align;
}


std::vector<CorrectedSingleScan> AlignedScans::getScansWithPose() const {
	std::vector<CorrectedSingleScan> result;
	for(const auto& s_w_p : scans_with_pose) {
		result.emplace_back(
			std::get<0>(s_w_p),
			std::get<1>(s_w_p),
			std::get<2>(s_w_p));
	}
	return result;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr AlignedScans::getMergedPoints() const {
	return cast<pcl::PointXYZRGBNormal, pcl::PointXYZRGB>(getMergedPointsNormal());
}

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr AlignedScans::getMergedPointsNormal() const {
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr merged(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
	for(const auto& s_w_p : scans_with_pose) {
		const auto multiplier = std::get<2>(s_w_p);
		auto delta = applyTransform(std::get<0>(s_w_p).cloud, std::get<1>(s_w_p));
		for(auto& pt : delta->points) {
			pt.r = std::min(pt.r * multiplier(0), 255.0f);
			pt.g = std::min(pt.g * multiplier(1), 255.0f);
			pt.b = std::min(pt.b * multiplier(2), 255.0f);
			merged->points.push_back(pt);
		}
	}
	return merged;
}

}  // namespace

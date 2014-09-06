#include "scene_recognizer.h"

#include <fstream>

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

#include <third/ICP.h>
#include <visual/cloud_baker.h>
#include <visual/cloud_base.h>
#include <visual/film.h>
#include <visual/mapping.h>
#include <visual/mesh_intersecter.h>
#include <visual/shape_fitter.h>
#include <visual/texture_conversion.h>

namespace visual {

const double pi = 3.14159265359;


SingleScan::SingleScan(Json::Value& cloud_json) :
		pre_rotation(0) {
	// Convert input coords. space to make Z+ up.
	// Only required for old dataset which was Y+ up.
	Eigen::Matrix3f m;
	m.col(0) = Eigen::Vector3f(0, 1, 0);
	m.col(1) = Eigen::Vector3f(0, 0, 1);
	m.col(2) = Eigen::Vector3f(1, 0, 0);

	cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
	for(const auto& point : cloud_json) {
		pcl::PointXYZRGB pt;
		pt.x = point["x"].asDouble();
		pt.y = point["y"].asDouble();
		pt.z = point["z"].asDouble();
		pt.r = point["r"].asDouble();
		pt.g = point["g"].asDouble();
		pt.b = point["b"].asDouble();

		pt.getVector3fMap() = m * pt.getVector3fMap();
		cloud->points.push_back(pt);
	}
	WARN("Since SingleScan Json constructor is no longer supported, program might crash when newer feature is used; consider migration");
}

SingleScan::SingleScan(const std::string& scan_dir, float pre_rotation) :
		pre_rotation(pre_rotation) {
	using boost::filesystem::path;
	INFO("Loading a scan from", scan_dir);
	std::ifstream f_input((path(scan_dir) / path("points.json")).string());
	if(!f_input.is_open()) {
		throw std::runtime_error("Cloudn't open points.json");
	}
	Json::Value cloud_json;
	Json::Reader().parse(f_input, cloud_json);

	cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
	for(const auto& point : cloud_json) {
		pcl::PointXYZRGB pt;
		pt.x = point["x"].asDouble();
		pt.y = point["y"].asDouble();
		pt.z = point["z"].asDouble();
		pt.r = point["r"].asDouble();
		pt.g = point["g"].asDouble();
		pt.b = point["b"].asDouble();
		cloud->points.push_back(pt);
	}
	cloud_w_normal.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
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
		cloud_w_normal->points.push_back(pt);
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


AlignedScans::AlignedScans(const std::vector<SingleScan>& scans) {
	assert(!scans.empty());

	createClosenessMatrix(scans);
	hierarchicalMerge(scans);

	scans_with_pose.push_back(std::make_pair(
		scans[0],
		Eigen::Affine3f::Identity()));

	// 0: target
	// 1,2,..: source
	// return: (position matrix, normal matrix)
	auto to_matrix = [](pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud) {
		const auto ds_cloud = cloud_base::downsample<pcl::PointXYZRGBNormal>(cloud, 0.05);
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
		return cloud_base::downsample<pcl::PointXYZRGBNormal>(cloud, 0.05);
	};

	bool use_sicp = false;

	// Merge others.
	for(int i : boost::irange(1, (int)scans.size())) {
		INFO("Calculating best pre-alignment between", 0, i);
		const auto pre_align = prealign(scans[0], scans[i]);
		INFO("Prealign Affine |t|=", pre_align.translation().norm());

		const auto source_pa = cloud_base::applyTransform(scans[i].cloud_w_normal, pre_align);
		const auto target = scans[0].cloud_w_normal;
		Eigen::Affine3f trans;
		if(false) {
			INFO("Running Sparse ICP", i);

			auto m_source_pa = to_matrix(source_pa);
			auto m_target = to_matrix(target);

			// See this youtube video for quick summary of good p value.
			// https://www.youtube.com/watch?v=ii2vHBwlmo8
			SICP::Parameters params;
			params.max_icp = 200;
			params.p = 0.6;
			//params.stop = 0;

			// The type signature do look like it allows any Scalar, but only
			// double will work in reality.
			Eigen::Matrix3Xd m_source_orig = m_source_pa.first;
			SICP::point_to_plane(m_source_pa.first, m_target.first, m_target.second, params);

			// Recover motion.
			const Eigen::Affine3f fine_align = RigidMotionEstimator::point_to_point(
				m_source_orig, m_source_pa.first).cast<float>();
			INFO("Fine Affine |t|=", fine_align.translation().norm());
			trans = fine_align * pre_align;
		}
		if(true) {
			INFO("Running PCL ICP");
			pcl::IterativeClosestPoint<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp;
			icp.setInputCloud(to_cloud(source_pa));
			icp.setInputTarget(to_cloud(target));
			pcl::PointCloud<pcl::PointXYZRGBNormal> final;
			icp.align(final);
			INFO("ICP converged", icp.hasConverged(), "score", icp.getFitnessScore());
			Eigen::Affine3f fine_align(icp.getFinalTransformation());
			INFO("Fine Affine |t|=", fine_align.translation().norm());
			trans = fine_align * pre_align;
		}

		scans_with_pose.push_back(std::make_pair(
			scans[i], trans));
	}

	applyLeveling();
}

void AlignedScans::createClosenessMatrix(const std::vector<SingleScan>& scans) const {
	const int n = scans.size();
	INFO("Calculating point cloud closeness matrix");
	Eigen::MatrixXf closeness(n, n);
	for(int i : boost::irange(0, n)) {
		for(int j : boost::irange(i, n)) {
			const auto pre_align = prealign(scans[j], scans[i]);

			const float dist = cloud_base::cloudDistance(
				cloud_base::applyTransform(scans[i].cloud_w_normal, pre_align),
				scans[j].cloud_w_normal);

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
		remaining[i] = scans[i].cloud_w_normal;
		for(int j : boost::irange(i + 1, n)) {
			sparse_dist[std::make_pair(i, j)] = closeness(i, j);
		}
	}
	// merge until only one node remains.
	while(remaining.size() > 1) {
		const auto best_pair = std::min_element(sparse_dist.begin(), sparse_dist.end(),
			[](const std::pair<std::pair<int, int>, float>& a, const std::pair<std::pair<int, int>, float>& b) {
				return a.second < b.second;
			}
		)->first;
		// Create new node(agg_id) from best_pair.
		const int agg_id = new_id++;
		DEBUG("Pair:", best_pair.first, best_pair.second, "->", agg_id);


		// Align.
		DEBUG("ALIGNING");
		const auto pre_align = prealign(remaining[best_pair.second], remaining[best_pair.first]);
		DEBUG("FALIGNING");
		const auto fine_align = finealign(remaining[best_pair.second], cloud_base::applyTransform(remaining[best_pair.first], pre_align));
		DEBUG("REGISTERING");
		const Eigen::Affine3f trans = fine_align * pre_align;
		nodes[agg_id] = std::make_tuple(best_pair.second, best_pair.first, trans);
		remaining[agg_id] = cloud_base::merge<pcl::PointXYZRGBNormal>(
			remaining[best_pair.second],
			cloud_base::applyTransform(remaining[best_pair.first], trans));
		remaining.erase(best_pair.first);
		remaining.erase(best_pair.second);

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
			const float dist = cloud_base::cloudDistance(
				cloud_base::applyTransform(remaining[i], pre_align),
				remaining[j]);
			sparse_dist[new_pair] = dist;
		}
	}
	assert(remaining.size() == 1);
}

void AlignedScans::hierarchicalMerge(const std::vector<SingleScan>& scans) {
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
	std::vector<std::pair<SingleScan, Eigen::Affine3f>> new_scans_with_pose;
	for(auto& scan_w_pose : scans_with_pose) {
		new_scans_with_pose.push_back(std::make_pair(
			scan_w_pose.first,
			trans * scan_w_pose.second));
	}
	scans_with_pose = std::move(new_scans_with_pose);
}

Eigen::Affine3f AlignedScans::prealign(const SingleScan& target, const SingleScan& source) {
	return prealign(target.cloud_w_normal, source.cloud_w_normal);
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
			shape_fitter::mean(shape_fitter::robustMinMax(xs, 0.01)),
			shape_fitter::mean(shape_fitter::robustMinMax(ys, 0.01)));
	};

	// source --seed--> source' --centroid-align--> source'' ==? target
	float best_dist = 1e6;
	Eigen::Affine3f best_trans;
	for(const auto& seed_trans : seeds) {
		const auto cloud_seeded = cloud_base::applyTransform(source, seed_trans);
		const auto dp = extract_centroid(target) - extract_centroid(cloud_seeded);
		const Eigen::Affine3f trans_centroid(Eigen::Translation3f(Eigen::Vector3f(dp(0), dp(1), 0)));

		const float dist = cloud_base::cloudDistance(cloud_base::applyTransform(cloud_seeded, trans_centroid), target);
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
	return finealign(target.cloud_w_normal, source.cloud_w_normal);
}

Eigen::Affine3f AlignedScans::finealign(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr target,
		const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr source) {
	assert(target);
	assert(source);

	// 0: target
	// 1,2,..: source
	// return: (position matrix, normal matrix)
	auto to_matrix = [](pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud) {
		const auto ds_cloud = cloud_base::downsample<pcl::PointXYZRGBNormal>(cloud, 0.05);
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
		return cloud_base::downsample<pcl::PointXYZRGBNormal>(cloud, 0.05);
	};

	/*
	bool use_sicp = false;

		const auto source = cloud_base::applyTransform(scans[i].cloud_w_normal, pre_align);
		const auto target = scans[0].cloud_w_normal;
		Eigen::Affine3f trans;
		if(false) {
			INFO("Running Sparse ICP", i);

			auto m_source_pa = to_matrix(source_pa);
			auto m_target = to_matrix(target);

			// See this youtube video for quick summary of good p value.
			// https://www.youtube.com/watch?v=ii2vHBwlmo8
			SICP::Parameters params;
			params.max_icp = 200;
			params.p = 0.6;
			//params.stop = 0;

			// The type signature do look like it allows any Scalar, but only
			// double will work in reality.
			Eigen::Matrix3Xd m_source_orig = m_source_pa.first;
			SICP::point_to_plane(m_source_pa.first, m_target.first, m_target.second, params);

			// Recover motion.
			const Eigen::Affine3f fine_align = RigidMotionEstimator::point_to_point(
				m_source_orig, m_source_pa.first).cast<float>();
			INFO("Fine Affine |t|=", fine_align.translation().norm());
			trans = fine_align * pre_align;
		}
	*/
	if(true) {
		INFO("Running PCL ICP");
		pcl::IterativeClosestPoint<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp;
		icp.setInputCloud(to_cloud(source));
		icp.setInputTarget(to_cloud(target));
		pcl::PointCloud<pcl::PointXYZRGBNormal> final;
		icp.align(final);
		INFO("ICP converged", icp.hasConverged(), "score", icp.getFitnessScore());
		Eigen::Affine3f fine_align(icp.getFinalTransformation());
		INFO("Fine Affine |t|=", fine_align.translation().norm());
		return fine_align;
	}
}


std::vector<std::pair<SingleScan, Eigen::Affine3f>> AlignedScans::getScansWithPose() const {
	return scans_with_pose;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr AlignedScans::getMergedPoints() const {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged(new pcl::PointCloud<pcl::PointXYZRGB>());
	for(const auto& s_w_p : scans_with_pose) {
		for(const auto& pt : s_w_p.first.cloud->points) {
			pcl::PointXYZRGB pt_new = pt;
			pt_new.getVector3fMap() = s_w_p.second * pt.getVector3fMap();
			merged->points.push_back(pt_new);
		}
	}
	return merged;
}

}  // namespace

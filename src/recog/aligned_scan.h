#pragma once

#include <map>
#include <tuple>

#include <boost/optional.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <jsoncpp/json/json.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <logging.h>
#include <recog/scene_asset_bundle.h>
#include <visual/textured_mesh.h>

namespace recon {

// Immutable data that holds a single scan.
class SingleScan {
public:
	SingleScan(const std::string& path);

	std::string getScanId() const;
public:
	// TODO: make this immutable.
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud;

	cv::Mat_<cv::Vec3b> er_rgb;
	cv::Mat_<float> er_depth;
	cv::Mat_<uint16_t> er_intensity;

private:
	std::string scan_id;
};


// An aligned, color-corrected, SingleScan.
class CorrectedSingleScan {
public:
	CorrectedSingleScan(
		const SingleScan& raw_scan,
		const Eigen::Affine3f& local_to_world,
		const Eigen::Vector3f& color_multiplier);
public:
	SingleScan raw_scan;
	Eigen::Affine3f local_to_world;
	Eigen::Vector3f color_multiplier;
};


// All scans succesfully aligned.
class AlignedScans {
public:
	// Create aligned scan from unaligned scans. (slow, takes a few minutes)
	AlignedScans(SceneAssetBundle& bundle, const std::vector<SingleScan>& scans);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getMergedPoints() const;
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr getMergedPointsNormal() const;

	// TODO: deprecate this!
	// External process shouldn't depend on individual scans,
	// and this interface collides with color-correcting alignment step.
	std::vector<CorrectedSingleScan> getScansWithPose() const;

	// pose (de)serialization for checkpointing.
	// This checkpoint doesn't include color correction or leveling.
	Json::Value saveCheckpoint() const;
	void loadCheckpoint(const Json::Value& cp);

	// (row-major, whole-matrix encoding).
	static Json::Value encodeAffine(const Eigen::Affine3f& affine);
	static Eigen::Affine3f decodeAffine(const Json::Value& json, bool require_rigid=false);
private:
	// Load external json that contains poses, and initialize internal scan structure using it.
	void loadInitialPoses(const std::string& path, const std::vector<SingleScan>& scans);

	// Do finealignment to target scan id. (pose target of will not change)
	void finealignToTarget(const std::string& fine_align_target_id);

	void createClosenessMatrix(SceneAssetBundle& bundle, const std::vector<SingleScan>& scans) const;
	void hierarchicalMerge(const std::vector<SingleScan>& scans);

	// Calculate a rough transform from source to target by heuristics.
	static Eigen::Affine3f prealign(const SingleScan& target, const SingleScan& source);
	static Eigen::Affine3f finealign(const SingleScan& target, const SingleScan& source);
	static Eigen::Affine3f prealign(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr target,
			const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr source);
	static Eigen::Affine3f finealign(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr target,
			const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr source);

	// Estimate color multipliers in scans_with_pose, so that
	// color seams in corrected point cloud is minimized.
	void correctColor();

	// Make large near-horizontal planar segment (most likely ceiling) completely level
	// by slight rotation.
	void applyLeveling();
private:
	// [(original scan, local_to_world, color multiplier)]
	std::vector<std::tuple<SingleScan, Eigen::Affine3f, Eigen::Vector3f>> scans_with_pose;
};

}  // namespace

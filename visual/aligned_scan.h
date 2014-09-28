#pragma once

#include <map>
#include <tuple>

#include <boost/optional.hpp>
#include <Eigen/Dense>
#include <jsoncpp/json/json.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <logging.h>
#include <visual/scene_asset_bundle.h>
#include <visual/textured_mesh.h>

namespace visual {

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


// All scans succesfully aligned.
class AlignedScans {
public:
	// Create aligned scan from unaligned scans. (slow, takes a few minutes)
	AlignedScans(SceneAssetBundle& bundle, const std::vector<SingleScan>& scans);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getMergedPoints() const;
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr getMergedPointsNormal() const;
	std::vector<std::pair<SingleScan, Eigen::Affine3f>> getScansWithPose() const;
private:
	// Use external json with pose for each scan.
	void predefinedMerge(std::string path, const std::vector<SingleScan>& scans);

	void createClosenessMatrix(SceneAssetBundle& bundle, const std::vector<SingleScan>& scans) const;
	void hierarchicalMerge(const std::vector<SingleScan>& scans);

	// Calculate a rough transform from source to target by heuristics.
	static Eigen::Affine3f prealign(const SingleScan& target, const SingleScan& source);
	static Eigen::Affine3f finealign(const SingleScan& target, const SingleScan& source);
	static Eigen::Affine3f prealign(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr target,
			const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr source);
	static Eigen::Affine3f finealign(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr target,
			const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr source);

	// Mkae large near-horizontal planar segment (most likely ceiling) completely level
	// by slight rotation.
	void applyLeveling();
private:
	// [(original scan, local_to_world)]
	std::vector<std::pair<SingleScan, Eigen::Affine3f>> scans_with_pose;
};

}  // namespace

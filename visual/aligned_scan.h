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
#include <visual/textured_mesh.h>

namespace visual {

// Immutable data that holds a single scan.
class SingleScan {
public:
	SingleScan(Json::Value& old_style);

	SingleScan(const std::string& path, float pre_rotation = 0);
public:
	// TODO: make this immutable.
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_w_normal;

	// Hack to make sparse ICP work.
	// TODO: guess this internally and remove this field,
	// or replace with magnetometer measurement.
	const float pre_rotation;

	cv::Mat_<cv::Vec3b> er_rgb;
	cv::Mat_<float> er_depth;
	cv::Mat_<uint16_t> er_intensity;
};


// All scans succesfully aligned.
class AlignedScans {
public:
	// Create aligned scan from unaligned scans. (slow, takes a few minutes)
	AlignedScans(const std::vector<SingleScan>& scans);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getMergedPoints() const;
	std::vector<std::pair<SingleScan, Eigen::Affine3f>> getScansWithPose() const;
private:
	void createClosenessMatrix(const std::vector<SingleScan>& scans) const;

	// Calculate a rough transform from source to target by heuristics.
	static Eigen::Affine3f prealign(const SingleScan& target, const SingleScan& source);

	// Mkae large near-horizontal planar segment (most likely ceiling) completely level
	// by slight rotation.
	void applyLeveling();
private:
	// [(original scan, local_to_world)]
	std::vector<std::pair<SingleScan, Eigen::Affine3f>> scans_with_pose;
};

}  // namespace

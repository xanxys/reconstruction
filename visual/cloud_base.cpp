#include "scene_recognizer.h"

#include <fstream>

#include <boost/optional.hpp>
#include <boost/range/irange.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <visual/mesh_intersecter.h>
#include <visual/shape_fitter.h>
#include <visual/texture_conversion.h>

namespace visual {
namespace cloud_base {

const double pi = 3.14159265359;

// Apply affine transform to given XYZ+RGB+Normal point cloud,
// and return new transformed cloud.
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr applyTransform(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, const Eigen::Affine3f& trans) {
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
	for(auto& pt : cloud->points) {
		pcl::PointXYZRGBNormal pt_new = pt;
		pt_new.getVector3fMap() = trans * pt.getVector3fMap();
		pt_new.getNormalVector3fMap() = trans * pt.getNormalVector3fMap();
		new_cloud->points.push_back(pt_new);
	}
	return new_cloud;
}

// This function will be called several times for pre-alignment:
// focus on speed rather than accurancy.
float cloudDistance(
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr c1,
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr c2) {
	// Collect single smaple per each bin.
	const float res = 0.3;
	const float weight_normal_cos = 5;
	const float weight_color_l2 = 0.01;
	// PosIndex -> (Normal, RGB)
	std::map<
		std::tuple<int, int, int>,
		std::pair<
			std::vector<std::tuple<Eigen::Vector3f, Eigen::Vector3f>>,
			std::vector<std::tuple<Eigen::Vector3f, Eigen::Vector3f>>>> cells;
	for(const auto& pt : c1->points) {
		const auto ix = (pt.getVector3fMap() / res).cast<int>();
		const auto tix = std::make_tuple(ix(0), ix(1), ix(2));
		cells[tix].first.push_back(std::make_tuple(
			pt.getNormalVector3fMap(),
			pt.getRGBVector3i().cast<float>()));
	}
	for(const auto& pt : c2->points) {
		const auto ix = (pt.getVector3fMap() / res).cast<int>();
		const auto tix = std::make_tuple(ix(0), ix(1), ix(2));
		cells[tix].second.push_back(std::make_tuple(
			pt.getNormalVector3fMap(),
			pt.getRGBVector3i().cast<float>()));
	}

	// Compare each bin's score.
	auto calc_stat = [](const std::vector<std::tuple<Eigen::Vector3f, Eigen::Vector3f>>& samples) {
		const int n = samples.size();
		if(n == 0) {
			return boost::optional<std::tuple<Eigen::Vector3f, Eigen::Vector3f>>();
		}
		Eigen::Vector3f accum_n = Eigen::Vector3f::Zero();
		Eigen::Vector3f accum_c = Eigen::Vector3f::Zero();
		for(const auto& sample : samples) {
			accum_n += std::get<0>(sample);
			accum_c += std::get<1>(sample);
		}
		return boost::optional<std::tuple<Eigen::Vector3f, Eigen::Vector3f>>(std::make_tuple(
			(accum_n / n).normalized(),
			accum_c / n));
	};
	int n_bins = 0;
	float accum_distance = 0;
	for(const auto& bin : cells) {
		const auto& samples = bin.second;
		const auto stat1 = calc_stat(samples.first);
		const auto stat2 = calc_stat(samples.second);
		if(!stat1 || !stat2) {
			continue;
		}

		const float dist =
			weight_normal_cos * (std::get<0>(*stat1) - std::get<0>(*stat2)).norm() +
			weight_color_l2 * (std::get<1>(*stat1) - std::get<1>(*stat2)).norm();
		accum_distance += dist;
		n_bins++;
	}
	if(n_bins == 0) {
		throw std::runtime_error("similarity() is not defined for empty point clouds");
	}
	return 1.0 / n_bins; //accum_distance / (n_bins * std::sqrt(n_bins));  // More bins == more similar
}

pcl::PointCloud<pcl::PointXYZ>::Ptr decolor(const pcl::PointCloud<pcl::PointXYZRGB>& cloud) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_colorless(new pcl::PointCloud<pcl::PointXYZ>());
	for(const auto& point : cloud) {
		pcl::PointXYZ pt;
		pt.getVector3fMap() = point.getVector3fMap();
		cloud_colorless->points.push_back(pt);
	}
	return cloud_colorless;
}

}  // namespace
}  // namespace

#include "frame_belief.h"

#include <map>
#include <cmath>
#include <exception>
#include <random>

#include <boost/range/irange.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "../renderer/renderer.h"
#include "voxel_traversal.h"

using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using ColorCloud = pcl::PointCloud<pcl::PointXYZRGBA>;
using RigidTrans3f = Eigen::Transform<float, 3, Eigen::AffineCompact>;

const double pi = 3.14159265359;

FrameBelief::FrameBelief(
	const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud) :
	cloud(cloud), camera_pos(0, 0, 0), camera_center(320, 240), camera_fl(585) {
}

cv::Mat FrameBelief::extractImageFromPointCloud(
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

cv::Mat FrameBelief::extractDepthImageFromPointCloud(
	const ColorCloud::ConstPtr& cloud) {
	if(cloud->points.size() != cloud->width * cloud->height) {
		throw std::runtime_error("Point cloud is not an image");
	}

	cv::Mat depth(cloud->height, cloud->width, CV_32F);
	for(int y : boost::irange(0, (int)cloud->height)) {
		for(int x : boost::irange(0, (int)cloud->width)) {
			const pcl::PointXYZRGBA& pt = cloud->points[y * cloud->width + x];

			depth.at<float>(y, x) = pt.z;
		}
	}
	return depth;
}

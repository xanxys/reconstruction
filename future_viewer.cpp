#include "future_viewer.h"

#include <cmath>
#include <iostream>
#include <sstream>
#include <random>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>

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


FutureViewer::FutureViewer() : visualizer("Time Lens2") {
	visualizer.addCoordinateSystem(1.0);
	visualizer.addPointCloud(ColorCloud::ConstPtr(new ColorCloud()));
}

// Called in grabber thread. DO NOT TOUCH visualizer!.
void FutureViewer::cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud) {
	ColorCloud::Ptr cloud_filtered(new ColorCloud());
	for(const auto& pt : cloud->points) {
		if(std::isfinite(pt.x)) {
			cloud_filtered->points.push_back(pt);
		}
	}

	ColorCloud::Ptr cloud_final(new ColorCloud());

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>());
	tree->setInputCloud(cloud_filtered);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
	ec.setClusterTolerance(0.05);
	ec.setMinClusterSize(100);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud_filtered);
	ec.extract(cluster_indices);

	std::cout << "#clusters: " << cluster_indices.size() << std::endl;

	// Detect 0 or 1 plane in each cluster.
	ColorSet colors;
	for(const auto& cluster : cluster_indices) {
		// Indices -> Points
		ColorCloud::Ptr cloud_cluster(new ColorCloud());
		for(const int index : cluster.indices) {
			cloud_cluster->points.push_back(cloud_filtered->points[index]);
		}

		// Setup segmentation params.
		pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
		seg.setOptimizeCoefficients (true);
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setDistanceThreshold (0.01);

		// Do segmentation.
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		seg.setInputCloud (cloud_cluster);
		seg.segment (*inliers, *coefficients);

		// Color cluster cloud.
		colors.next();
		for(const int index : inliers->indices) {
			colors.apply(cloud_cluster->points[index]);
		}

		if(inliers->indices.size() > 0) {
			// Calculate center.
			Eigen::Vector3f accum(Eigen::Vector3f::Zero());
			for(const int index : inliers->indices) {
				accum += cloud_cluster->points[index].getVector3fMap();
			}
			const auto center = accum / inliers->indices.size();

			// Calculate approx size.
			float max_len = 0;
			for(const int index : inliers->indices) {
				max_len = std::max(
					max_len,
				 	(cloud_cluster->points[index].getVector3fMap() - center).norm());
			}

			// Calculate basis with e0=normal, e1=up-like, e2=?
			const auto e0 = Eigen::Vector3f(
				coefficients->values[0],
				coefficients->values[1],
				coefficients->values[2]).normalized();

			const Eigen::Vector3f pre_e1(0, 1, 0);
			const auto e2 = e0.cross(pre_e1).normalized();
			const auto e1 = e2.cross(e0);

			Eigen::Matrix3f basis;
			basis.col(0) = e1;
			basis.col(1) = e2;
			basis.col(2) = e0;  // Z -> normal

			// Setup appropriate quad.
			RigidTrans3f trans;
			trans = Eigen::Translation<float, 3>(center) * basis;

			cloud_final = concat(cloud_final, generateQuad(max_len * 2, max_len * 2, trans));
		}

		cloud_final = concat(cloud_final, cloud_cluster);
	}

	{
		std::lock_guard<std::mutex> lock(latest_cloud_lock);
		latest_cloud = cloud_final;
	}
}

/*

std::vector<TrackingTarget> FutureViewer::findAllTargets(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud_color) {
	Cloud::Ptr cloud(new Cloud());
	pcl::copyPointCloud(*cloud_color, *cloud);

	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	Cloud::Ptr cloud_filtered(new Cloud());
	vg.setInputCloud(cloud);
	vg.setLeafSize(0.01f, 0.01f, 0.01f);
	vg.filter(*cloud_filtered);
	std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size()  << " data points." << std::endl;

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud_filtered);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(0.05);
	ec.setMinClusterSize(10);
	ec.setMaxClusterSize(100000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud_filtered);
	ec.extract(cluster_indices);

	// Pack colored points.
	ColorCloud::Ptr cloud_cluster(new ColorCloud());

	int cluster_ix = 0;
	cloud_cluster->width = 0;

	std::mt19937 gen;
	std::vector<TrackingTarget> targets;
	for(const auto& point_indices : cluster_indices) {
		std::cout << "Cluster size: " << point_indices.indices.size() << std::endl;

		Eigen::Vector3f accum(0, 0, 0);
		for(int index : point_indices.indices) {
			const pcl::PointXYZ& pt_xyz = cloud_filtered->points[index];

			pcl::PointXYZRGBA pt;
			pt.getVector3fMap() = pt_xyz.getVector3fMap();
			pt.r = cluster_ix * 25;
			pt.g = 0xff;
			pt.b = 255 - pt.r;
			pt.a = 0xff;

			cloud_cluster->points.push_back(pt);
			accum += Eigen::Vector3f(pt.x, pt.y, pt.z);
		}
		cloud_cluster->width += cloud_cluster->points.size();

		const Eigen::Vector3f center = accum / point_indices.indices.size();
		std::cout << "center=" << center << std::endl;

		// Ignore large objects.
		if(point_indices.indices.size() < 1000) {
			TrackingTarget target(center);
			target.r = std::uniform_int_distribution<>(0, 255)(gen);
			target.g = std::uniform_int_distribution<>(0, 255)(gen);
			target.b = std::uniform_int_distribution<>(0, 255)(gen);
			targets.push_back(target);
		}
	}
	cloud_cluster->height = 1;
	cloud_cluster->is_dense = true;

	return targets;
}

void FutureViewer::trackTarget(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud_color, TrackingTarget& target) {
	const float radius = 0.1;

	Eigen::Vector3f accum(0, 0, 0);
	int count = 0;
	for(const auto& point : cloud_color->points) {
		if((point.getVector3fMap() - target.position).norm() < radius) {
			accum += point.getVector3fMap();
			count += 1;
		}
	}

	if(count == 0) {
		std::cout << "Tracking lost" << std::endl;
		// TODO: Search again with larger radius.
		return;
	}

	const Eigen::Vector3f new_position = accum / count;
	const auto new_time = std::chrono::system_clock::now();
	target.velocity = (new_position - target.position) / (std::chrono::duration_cast<std::chrono::milliseconds>(new_time - target.time).count() * 1e-3);
	target.position = new_position;
	target.time = new_time;

	// stat
	target.position_avg10 *= 0.9;
	target.position_avg10 += target.position * 0.1;
	target.history.push_back(target.position);
}
*/

void FutureViewer::run() {
	pcl::Grabber* source = new pcl::OpenNIGrabber();

	boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
		boost::bind(&FutureViewer::cloud_cb_, this, _1);

	source->registerCallback(f);
	source->start();

	while(!visualizer.wasStopped()) {
		// No need to protect updatePointCloud, since the point cloud's data is const.
		// boost::shared_ptr internals are important here.
		ColorCloud::ConstPtr cloud;
		{
			std::lock_guard<std::mutex> lock(latest_cloud_lock);
			cloud = latest_cloud;
		}
		if(cloud) {
			visualizer.updatePointCloud(cloud);
		}
		
		// This call blocks (== cannot update point cloud) when user is rotating the view.
		visualizer.spinOnce(100, true);

		boost::this_thread::sleep(boost::posix_time::microseconds(20000));
	}
	source->stop();
}

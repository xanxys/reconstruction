#include "future_viewer.h"

#include <iostream>
#include <sstream>
#include <random>

#include <Eigen/Dense>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>

using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using ColorCloud = pcl::PointCloud<pcl::PointXYZRGBA>;

FutureViewer::FutureViewer() : viewer("Time Lens"), tracking_ok(false), target_position({0, 0, 0}) {
}

void FutureViewer::cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud) {
	if(viewer.wasStopped()) {
		return;
	}

	if(tracking_ok) {
		tracking_ok = trackTarget(cloud);
	} else {
		tracking_ok = findTarget(cloud);
	}

	// Add visualization to the original point cloud.
	ColorCloud::Ptr cloud_final(new ColorCloud());
	pcl::copyPointCloud(*cloud, *cloud_final);

	std::mt19937 gen;
	if(tracking_ok) {
		for(int i = 0; i < 200; i++) {
			const Eigen::Vector3d dp(
				std::normal_distribution<double>(0, 0.01)(gen),
				std::normal_distribution<double>(0, 0.01)(gen),
				std::normal_distribution<double>(0, 0.01)(gen));

			const Eigen::Vector3d p = target_position + dp;

			pcl::PointXYZRGBA pt;
			pt.x = p.x();
			pt.y = p.y();
			pt.z = p.z();
			pt.r = 255;
			pt.g = 0;
			pt.b = 0;
			pt.a = 255;
			cloud_final->points.push_back(pt);
		}
	}

	viewer.showCloud(cloud_final);
}

bool FutureViewer::findTarget(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud_color) {
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
	for(const auto& point_indices : cluster_indices) {
		for(int index : point_indices.indices) {
			const pcl::PointXYZ& pt_xyz = cloud_filtered->points[index];

			pcl::PointXYZRGBA pt;
			pt.getVector3fMap() = pt_xyz.getVector3fMap();
			pt.r = cluster_ix * 25;
			pt.g = 0xff;
			pt.b = 255 - pt.r;
			pt.a = 0xff;

			cloud_cluster->points.push_back(pt);
		}
		cloud_cluster->width += cloud_cluster->points.size();
		cluster_ix++;
	}
	cloud_cluster->height = 1;
	cloud_cluster->is_dense = true;

	return true;
}

bool FutureViewer::trackTarget(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud_color) {
	target_position += Eigen::Vector3d(0, 0, 0.01);
	return true;
}

void FutureViewer::run() {
	pcl::Grabber* interface = new pcl::OpenNIGrabber();

	boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
		boost::bind(&FutureViewer::cloud_cb_, this, _1);

	interface->registerCallback(f);

	interface->start();

	while(!viewer.wasStopped()) {
		sleep(1);
	}

	interface->stop();
}

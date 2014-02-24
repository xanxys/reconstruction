#define linux 1
#define __x86_64__ 1

#include <iostream>
#include <sstream>

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


class SimpleOpenNIViewer {
public:
	SimpleOpenNIViewer();

	void cloud_cb_(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);
	void run();

	pcl::visualization::CloudViewer viewer;
};

SimpleOpenNIViewer::SimpleOpenNIViewer() : viewer("Time Lens") {
}

void SimpleOpenNIViewer::cloud_cb_(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud) {
	if(viewer.wasStopped()) {
		return;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>());

	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	vg.setInputCloud(cloud);
	vg.setLeafSize(0.01f, 0.01f, 0.01f);
	vg.filter(*cloud_filtered);
	std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size()  << " data points." << std::endl; //*

	// Create the segmentation object for the planar model and set all the parameters
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PCDWriter writer;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(0.02);

	int i=0, nr_points =(int) cloud_filtered->points.size();
	while(cloud_filtered->points.size() > 0.3 * nr_points) {
	
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud(cloud_filtered);
		seg.segment(*inliers, *coefficients);
		if(inliers->indices.size() == 0)
		{
			std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		// Extract the planar inliers from the input cloud
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud_filtered);
		extract.setIndices(inliers);
		extract.setNegative(false);

		// Get the points associated with the planar surface
		extract.filter(*cloud_plane);
		std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;

		// Remove the planar inliers, extract the rest
		extract.setNegative(true);
		extract.filter(*cloud_f);
		*cloud_filtered = *cloud_f;
	}

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud_filtered);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(0.05);
	ec.setMinClusterSize(100);
	ec.setMaxClusterSize(25000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud_filtered);
	ec.extract(cluster_indices);

	// Pack colored points.
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster(
		new pcl::PointCloud<pcl::PointXYZRGBA>());

	int cluster_ix = 0;
	cloud_cluster->width = 0;
	for(const auto& point_indices : cluster_indices) {
		for(int index : point_indices.indices) {
			const pcl::PointXYZ& pt_xyz = cloud_filtered->points[index];

			pcl::PointXYZRGBA pt;
			pt.getVector3fMap() = pt_xyz.getVector3fMap();
			pt.r = cluster_ix * 10;
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

	viewer.showCloud(cloud_cluster);
}

void SimpleOpenNIViewer::run() {
	pcl::Grabber* interface = new pcl::OpenNIGrabber();

	boost::function<void(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
		boost::bind(&SimpleOpenNIViewer::cloud_cb_, this, _1);

	interface->registerCallback(f);

	interface->start();

	while(!viewer.wasStopped()) {
		sleep(1);
	}

	interface->stop();
}

int main() {
	SimpleOpenNIViewer viewer;
	viewer.run();
	return 0;
}

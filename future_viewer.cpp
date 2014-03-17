#include "future_viewer.h"

#include <map>
#include <cmath>
#include <iostream>
#include <sstream>
#include <random>

#include <boost/range/irange.hpp>
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
// #include <pcl/visualization/cloud_viewer.h>

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


/*
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

	for(int step : boost::irange(0, 3)) {
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
		ColorCloud::Ptr next_cloud(new ColorCloud());
		for(const auto& cluster : cluster_indices) {
			// Indices -> Points
			ColorCloud::Ptr cloud_cluster(new ColorCloud());
			for(const int index : cluster.indices) {
				cloud_cluster->points.push_back(cloud_filtered->points[index]);
			}

			if(cloud_cluster->points.size() < 100) {
				next_cloud = concat(next_cloud, cloud_cluster);
				continue;
			}

			// Setup segmentation params.
			pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
			seg.setOptimizeCoefficients(true);
			seg.setModelType(pcl::SACMODEL_PLANE);
			seg.setMethodType(pcl::SAC_RANSAC);
			seg.setDistanceThreshold(0.01);

			// Do segmentation.
			pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
			pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
			seg.setInputCloud (cloud_cluster);
			seg.segment (*inliers, *coefficients);

			if(inliers->indices.size() == 0) {
				next_cloud = concat(next_cloud, cloud_cluster);
				continue;
			}

			// Color cluster cloud.
			colors.next();
			for(const int index : inliers->indices) {
				colors.apply(cloud_cluster->points[index]);
				cloud_final->points.push_back(cloud_cluster->points[index]);
			}


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

			// Subtract.
			ColorCloud::Ptr remaining(new ColorCloud());
			pcl::ExtractIndices<pcl::PointXYZRGBA> remover;
			remover.setNegative(true);
			remover.setIndices(inliers);

			remover.setInputCloud(cloud_cluster);
			remover.filter(*remaining);

			next_cloud = concat(next_cloud, remaining);

			// Add remaining point.
			cloud_final = concat(cloud_final, cloud_cluster);
		}

		cloud_filtered = next_cloud;
	}
	

	{
		std::lock_guard<std::mutex> lock(latest_cloud_lock);
		latest_cloud = cloud_final;
	}
}


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
*/

VoxelTraversal::VoxelTraversal(float size, Eigen::Vector3f org, Eigen::Vector3f dir) :
	org(org / size), dir(dir) {
	index = Eigen::Vector3i(
		std::floor(org(0)),
		std::floor(org(1)),
		std::floor(org(2)));

	frac = org - index.cast<float>();
}

std::tuple<int, int, int> VoxelTraversal::next() {
	const auto key = std::make_tuple(index(0), index(1), index(2));

	const auto remaining = Eigen::Vector3f(
		(dir(0) < 0) ? -frac(0) : 1 - frac(0),
		(dir(1) < 0) ? -frac(1) : 1 - frac(1),
		(dir(2) < 0) ? -frac(2) : 1 - frac(2));

	const auto dt_xyz = remaining.cwiseQuotient(dir);

	// Select the direction with smallest dt. (tie-breaker: prefer X>Y>Z)
	if(dt_xyz(0) <= dt_xyz(1) && dt_xyz(0) <= dt_xyz(2)) {
		const int dix = (dir(0) < 0) ? -1 : 1;
		index(0) += dix;
		frac += dt_xyz(0) * dir;
		frac(0) -= dix;
	} else if(dt_xyz(1) <= dt_xyz(2)) {
		const int dix = (dir(1) < 0) ? -1 : 1;
		index(1) += dix;
		frac += dt_xyz(1) * dir;
		frac(1) -= dix;
	} else {
		const int dix = (dir(2) < 0) ? -1 : 1;
		index(2) += dix;
		frac += dt_xyz(2) * dir;
		frac(2) -= dix;
	}

	return key;
}

ReconServer::ReconServer() {
	pcl::Grabber* source = new pcl::OpenNIGrabber();

	boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
	boost::bind(&ReconServer::cloudCallback, this, _1);

	source->registerCallback(f);
	source->start();
}

void ReconServer::cloudCallback(const ColorCloud::ConstPtr& cloud) {
	{
		std::lock_guard<std::mutex> lock(latest_cloud_lock);
		latest_cloud = cloud;
	}
}

Response ReconServer::handleRequest(std::vector<std::string> uri,
	std::string method, std::string data) {

	if(uri.size() == 0) {
		return sendStaticFile("/index.html", "text/html");
	} else if(uri.size() == 2 && uri[0] == "static") {
		return sendStaticFile(uri[1]);
	} else if(uri.size() == 1 && uri[0] == "points") {
		return handlePoints();
	} else if(uri.size() == 1 && uri[0] == "voxels") {
		return handleVoxels();
	}
	return Response::notFound();
}

Response ReconServer::handlePoints() {
	Json::Value vs;
	{
		std::lock_guard<std::mutex> lock(latest_cloud_lock);
		if(!latest_cloud) {
			return Response::notFound();
		}

		for(const auto& pt : latest_cloud->points) {
			if(!std::isfinite(pt.x)) {
				continue;
			}
			
			Json::Value p;
			p["x"] = pt.x;
			p["y"] = pt.y;
			p["z"] = pt.z;
			p["r"] = pt.r / 255.0;
			p["g"] = pt.g / 255.0;
			p["b"] = pt.b / 255.0;
			vs.append(p);
		}
	}
	return Response(vs);
}

Response ReconServer::handleVoxels() {
	Json::Value root;
	{
		std::lock_guard<std::mutex> lock(latest_cloud_lock);
		if(!latest_cloud) {
			return Response::notFound();
		}

		const float size = 0.1;

		// known to be filled
		std::map<std::tuple<int, int, int>, bool> voxels;
		for(const auto& pt : latest_cloud->points) {
			if(!std::isfinite(pt.x)) {
				continue;
			}

			auto ix = pt.getVector3fMap() / size;
			auto key = std::make_tuple(
				static_cast<int>(std::floor(ix.x())),
				static_cast<int>(std::floor(ix.y())),
				static_cast<int>(std::floor(ix.z())));
			voxels[key] = true;
		}

		const auto& camera_origin = Eigen::Vector3f::Zero();

		std::map<std::tuple<int, int, int>, bool> voxels_empty;
		for(const auto& pair_filled : voxels) {
			// cast ray from camera
			const auto pos = Eigen::Vector3f(
				std::get<0>(pair_filled.first),
				std::get<1>(pair_filled.first),
				std::get<2>(pair_filled.first)) * size;

			const auto dir = (pos - camera_origin).normalized();

			// traverse until hit.
			VoxelTraversal traversal(size, camera_origin, dir);
			for(int i : boost::irange(0, 150)) {
				const auto key = traversal.next();

				// Hit wall.
				if(voxels.find(key) != voxels.end()) {
					break;
				}

				voxels_empty[key] = true;
			}
		}

		for(const auto& pair : voxels_empty) {
			Json::Value vx;
			vx["x"] = std::get<0>(pair.first);
			vx["y"] = std::get<1>(pair.first);
			vx["z"] = std::get<2>(pair.first);
			root.append(vx);
		}
	}
	return Response(root);
}

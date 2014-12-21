#pragma once

#include <tuple>
#include <vector>

#include <boost/optional.hpp>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <Eigen/Dense>
#include <jsoncpp/json/json.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <geom/util.h>
#include <logging.h>
#include <recog/aligned_scan.h>
#include <recog/indoor.h>
#include <recog/scene_asset_bundle.h>
#include <recog/shape_fitter.h>
#include <visual/textured_mesh.h>

namespace recon {

std::pair<TexturedMesh, cv::Mat> bakeBoundaryTexture(
	const AlignedScans& scans,
	const TriangleMesh<std::nullptr_t>& shape,
	float accept_dist = 0.1);

// deprecated. Use recognizeBoundary.
std::vector<Eigen::Vector3f> recognizeLights(
	SceneAssetBundle& bundle,
	const RoomFrame& rframe,
	const AlignedScans& scans,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

// points
// |-inside
// | |-exterior
// | |-interior
// |-outside (either mirror or window)
std::pair<
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr,
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> splitInOut(
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr points_merged,
		const std::vector<Eigen::Vector2f>& room_polygon);

class MiniCluster {
public:
	MiniCluster(
		CorrectedSingleScan* c_scan,
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);

	static AABB3f calculateAABB(
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);
public:
	// original info
	CorrectedSingleScan* c_scan;  // borrowed
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud;

public:  // linking info
	AABB3f aabb;
	Eigen::Vector3f grav_center;

	// bottom part
	bool is_supported;
	float support_z;
	std::vector<Eigen::Vector2f> support_polygon;

	// derived info
	bool stable;
};


// MCLinker receives (and then owns) MiniClusters,
// and link them into groups.
// After that, you can access grouping results
// with some useful analysis functionality.
//
// We want to have:
// physically-stable, maximally separated objects.
// = minimize #links while making objects stable.
//
// Dirty truth is we also need to reject outlier clusters
// (patch of walls etc.) && hallucinate hidden surface.
// (esp. desks)
//
// Process basically starts from floor-touching MiniClusters,
// and we go up connecting.
class MCLinker {
public:
	MCLinker(
		SceneAssetBundle& bundle,
		const RoomFrame& rframe,
		const std::vector<MiniCluster>& mcs);

	std::vector<std::set<int>> getResult();
	const std::vector<MiniCluster>& getMiniClusters() const;

	// Return z value of supporting surface if there's any,
	// none otherwise.
	boost::optional<float> getSupportLevel(int cl) const;
private:
	using K = CGAL::Exact_predicates_inexact_constructions_kernel;
	using Kex = CGAL::Exact_predicates_exact_constructions_kernel;
	using Point_2 = K::Point_2;
	using Polygon_2 = CGAL::Polygon_2<K>;
	using MCId = int;

	float distanceBetween(MCId cl0, MCId cl1) const;
	float distanceBetween(
		const std::set<MCId>& cl0,
		const std::set<MCId>& cl1) const;

	// Group level queries.
	Eigen::Vector3f getCG(const std::set<MCId>& cl) const;
	boost::optional<Polygon_2> getSupportPolygon(
		const std::set<MCId>& cl) const;

	bool isStable(const std::set<MCId>& cls,
		float disturbance_radius) const;

	// Simple util.
	static std::set<MCId> join(
		const std::set<MCId>& a, const std::set<MCId>& b);
private:
	const RoomFrame& rframe;
	SceneAssetBundle& bundle;

	std::vector<MiniCluster> mcs;

	// Minimum point-point distance between clusters.
	Eigen::MatrixXf cluster_dist;
};


// Label each point as part of room boundary, inside, or outside.
// Small error between RoomFrame and aligned coordinate will be
// compensated, but CorrectedSingleScan must not have ghosting.
std::vector<MiniCluster> splitEachScan(
	SceneAssetBundle& bundle, CorrectedSingleScan& ccs, RoomFrame& rframe);


// Takes several scans of a single room as input (in unordered way),
// and populate given SceneAsssetBundle.
void recognizeScene(
	SceneAssetBundle& bundle, const std::vector<SingleScan>& scans,
	const Json::Value& hint);

// Create an InteriorObject complete with visual proxy
// and collisions from given subset of MiniClusters.
// Created object will NOT be added to bundle.
InteriorObject createInteriorObject(
	SceneAssetBundle& bundle,
	const MCLinker& linker,
	const std::set<int>& mc_ixs,
	const std::string& debug_id);

// Return textured boundary mesh and light location.
std::pair<TexturedMesh, std::vector<Eigen::Vector3f>>
	recognizeBoundary(
		SceneAssetBundle& bundle,
		const RoomFrame& rframe,
		const AlignedScans& scans,
		const ExtrudedPolygonMesh& ext_mesh,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_inside);


}  // namespace

#pragma once

#include <tuple>
#include <vector>

#include <boost/optional.hpp>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <logging.h>
#include <recog/aligned_scan.h>
#include <recog/indoor.h>
#include <recog/scene_asset_bundle.h>
#include <recog/shape_fitter.h>
#include <visual/textured_mesh.h>

namespace recon {

TexturedMesh bakeTexture(
	const AlignedScans& scans,
	const TriangleMesh<std::nullptr_t>& shape,
	float accept_dist = 0.1);


TexturedMesh bakeTextureSingleExterior(
	const AlignedScans& scans,
	const TriangleMesh<std::nullptr_t>& shape,
	float accept_dist = 0.1);

// deprecated. Use recognizeExterior.
std::vector<Eigen::Vector3f> recognize_lights(
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

// Return visually coherent planar objects from given
// mostly planar point cloud. This code uses grabcut.
//
// center, normal: avg. normal and avg. pos of cluster for local planar projection
// cluster_name: cluster name for debugging purpose
//
// WARNING: currently, extractVisualGroups returns only 0 or 1 group.
std::vector<TexturedMesh> extractVisualGroups(
	SceneAssetBundle& bundle, CorrectedSingleScan& c_scan,
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cluster,
	const Eigen::Vector3f& center,
	const Eigen::Vector3f& normal,
	const std::string& cluster_name);

class MiniCluster {
public:
	CorrectedSingleScan* c_scan;  // borrowed
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud;
};


// Label each point as part of room boundary, inside, or outside.
// Small error between RoomFrame and aligned coordinate will be
// compensated, but CorrectedSingleScan must not have ghosting.
std::vector<MiniCluster> splitEachScan(
	SceneAssetBundle& bundle, CorrectedSingleScan& ccs, RoomFrame& rframe);

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
void linkMiniClusters(
	SceneAssetBundle& bundle,
	const RoomFrame& rframe, const std::vector<MiniCluster>& mcs);

// Takes several scans of a single room as input (in unordered way),
// and populate given SceneAsssetBundle.
void recognizeScene(SceneAssetBundle& bundle, const std::vector<SingleScan>& scans);

// Return textured exterior mesh and light location.
std::pair<TexturedMesh, std::vector<Eigen::Vector3f>>
	recognizeExterior(
		SceneAssetBundle& bundle,
		const RoomFrame& rframe,
		const AlignedScans& scans,
		const ExtrudedPolygonMesh& ext_mesh,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_inside);


}  // namespace

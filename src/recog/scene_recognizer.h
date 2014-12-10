#pragma once

#include <tuple>
#include <vector>

#include <boost/optional.hpp>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <logging.h>
#include <recog/aligned_scan.h>
#include <recog/scene_asset_bundle.h>
#include <visual/textured_mesh.h>

namespace recon {

TexturedMesh bakeTexture(
	const AlignedScans& scans,
	const TriangleMesh<std::nullptr_t>& shape,
	float accept_dist = 0.1);


TexturedMesh bakeTextureSingle(
	const AlignedScans& scans,
	const TriangleMesh<std::nullptr_t>& shape,
	float accept_dist = 0.1);

// Represents Manhattan indoor room coordinates.
// But RoomFrame doesn't hold raw scans.
// (aligned scans and RoomFrame are implicitly tied by having
// shared world coordinate system)
//
// Main purpose of RoomFrame is information compression.
// (put another way, it's a very strong prior)
// Do NOT put fine-detailed contour here.
// New model (such as arcs) should be implemented in such cases.
//
// To make it obvious, do not store large objects here.
// (i.e. PointCloud, TexturedMesh, cv::Mat, etc.)
//
// RoomFrames defines not just entire 3-d coordinates, but
// * boundary of room (maybe fuzzy around windows)
// * wall / ceiling / floor coodinates
//
// Z+ is assumed to be up, but Z offset and other Manhattan axes
// are arbitrary.
//
// features:
// Mahnatta-aligned, rectangular pit or protrusion.
// One of edge must be substantial (>50%) of base surface.
// Depth must be small (<1m).
// (It should be called "wall" if it's very deep)
//
// Very thin or small "feature" should be called fixed object instead.
// (e.g. AC, lights)
class RoomFrame {
public:
	RoomFrame();

	// Set Z range that corresponds to min and max inside the room.
	// (ill-defined when there's holes in ceiling etc.)
	void setHRange(float z0, float z1);

	std::pair<float, float> getHRange() const;

	// Get wall contour after Manhattan enforcement as CCW ordered
	// vertices.
	// Warning: result will not align well with world X, Y axes.
	std::vector<Eigen::Vector2f> getSimplifiedContour() const;
public:
	std::vector<Eigen::Vector2f> wall_polygon;
private:
	// up and principle directions.
	Eigen::Vector3f up;
	Eigen::Vector3f prin0;
	Eigen::Vector3f prin1;

	// z range
	float z0;
	float z1;
};

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

// Label each point as part of room boundary, inside, or outside.
// Small error between RoomFrame and aligned coordinate will be
// compensated, but CorrectedSingleScan must not have ghosting.
void splitEachScan(
	SceneAssetBundle& bundle, CorrectedSingleScan& ccs, RoomFrame& rframe);

// Takes several scans of a single room as input (in unordered way),
// and populate given SceneAsssetBundle.
void recognizeScene(SceneAssetBundle& bundle, const std::vector<SingleScan>& scans);

// Return textured exterior mesh and light location.
std::pair<TexturedMesh, std::vector<Eigen::Vector3f>>
	recognizeExterior(
		SceneAssetBundle& bundle,
		const RoomFrame& rframe,
		const AlignedScans& scans,
		const TriangleMesh<std::nullptr_t>& shape,
		const std::vector<int>& ceiling_ixs,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_inside);

void splitObjects(
	SceneAssetBundle& bundle,
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_org,
	const AlignedScans& scans);

}  // namespace

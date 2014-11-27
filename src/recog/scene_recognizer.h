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

// deprecated. Use recognizeExterior.
std::vector<Eigen::Vector3f> recognize_lights(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

TexturedMesh bakeTexture(
	const AlignedScans& scans,
	const TriangleMesh<std::nullptr_t>& shape,
	float accept_dist = 0.1);

// Return nearest power of 2 number >= x.
// ceilToPowerOf2(x) = 1 for x <=0.
int ceilToPowerOf2(int x);

// Represents Manhattan indoor room coordinates.
// But RoomFrame doesn't hold raw scans.
// (aligned scans and RoomFrame are implicitly tied by having
// shared world coordinate system)
//
// RoomFrames defines not just entire 3-d coordinates, but
// * boundary of room (maybe fuzzy around windows)
// * wall / ceiling / floor coodinates
//
// Z+ is assumed to be up, but Z offset and other Manhattan axes
// are arbitrary.
class RoomFrame {
public:
	RoomFrame();
public:
	std::vector<Eigen::Vector2f> wall_polygon;
private:
	// up and principle directions.
	Eigen::Vector3f up;
	Eigen::Vector3f prin0;
	Eigen::Vector3f prin1;
};

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
		const AlignedScans& scans,
		const TriangleMesh<std::nullptr_t>& shape,
		const std::vector<int>& ceiling_ixs,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_inside);

void splitObjects(
	SceneAssetBundle& bundle,
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_org,
	const AlignedScans& scans);

}  // namespace

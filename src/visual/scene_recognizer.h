#pragma once

#include <tuple>
#include <vector>

#include <boost/optional.hpp>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <logging.h>
#include <visual/aligned_scan.h>
#include <visual/scene_asset_bundle.h>
#include <visual/textured_mesh.h>

namespace visual {

// deprecated. Use recognizeExterior.
std::vector<Eigen::Vector3f> recognize_lights(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

// Try avoiding classes for this kind of complex, pure operation.
namespace scene_recognizer {

TexturedMesh bakeTexture(
	const AlignedScans& scans,
	const TriangleMesh<std::nullptr_t>& shape,
	float accept_dist = 0.1);

// Return nearest power of 2 number >= x.
// ceilToPowerOf2(x) = 1 for x <=0.
int ceilToPowerOf2(int x);

void splitEachScan(SceneAssetBundle& bundle, CorrectedSingleScan& ccs);

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

}  // namespace

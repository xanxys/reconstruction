#pragma once

#include <memory>
#include <tuple>
#include <vector>

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <recog//scene_asset_bundle.h>
#include <visual/triangle_mesh.h>

namespace recon {

// Fit an Z-extruded concave, inward-facing shape.
// WARNING:
// Since this code use robust estimate, instead of min/max,
// some points will NOT be inside of the returned mesh.
std::tuple<
	std::vector<Eigen::Vector2f>,
	std::pair<float, float>
	> fitExtrudedPolygon(
		SceneAssetBundle& bundle,
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);


class ExtrudedPolygonMesh {
public:
	ExtrudedPolygonMesh(
		const std::vector<Eigen::Vector2f>& poly,
		const std::pair<float, float>& h_range);

	//std::vector<int> getFloorTriangles() const;
	//std::vector<int> getCeilingTriangles() const;
	std::vector<int> getFloorPolygonVertices() const;

	const TriangleMesh<std::nullptr_t>& getMesh() const;
private:
	std::vector<int> floor_poly_verts;
	std::vector<int> floor_tri_ixs;
	std::vector<int> ceiling_tri_ixs;
	TriangleMesh<std::nullptr_t> mesh;
};


float mean(const std::pair<float, float>& pair);
float half(const std::pair<float, float>& pair);

// The list will be sorted.
std::pair<float, float> robustMinMax(std::vector<float>& values, float tile=0.01);

// Check if given polygon is CCW.
// Results is undefined when given polygon is degenerate.
// runtime: O(N)
bool isPolygonCCW(const std::vector<Eigen::Vector2f>& points);

// Check if given polygon is "sane".
// 1. No too short segments.
// 2. No self intersection.
// 3. Has enough # of vertices.
// runtime: O(N^2)
bool isSaneSimplePolygon(const std::vector<Eigen::Vector2f>& points, const float eps = 1e-3);

// Extract XY 2D concave polygon. (CCW)
std::vector<Eigen::Vector2f> extractPolygon2D(
	SceneAssetBundle& bundle,
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

// Extract Z range
std::pair<float, float> extractHeightRange(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);


// Implementation of k-nearest neighbor based approach
// http://repositorium.sdum.uminho.pt/xmlui/bitstream/handle/1822/6429/ConcaveHull_ACM_MYS.pdf?sequence=1
// points: 2d points in shape [N, 2]
// return: CCW list of points
std::vector<Eigen::Vector2f> calculateConcaveHull(
	const std::vector<Eigen::Vector2f>& points, int k_min);

bool intersectSegments(
	std::pair<Eigen::Vector2f, Eigen::Vector2f> a0,
	std::pair<Eigen::Vector2f, Eigen::Vector2f> b0);

}  // namespace

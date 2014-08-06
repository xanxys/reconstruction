#pragma once

#include <memory>
#include <tuple>
#include <vector>

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <visual/triangle_mesh.h>

namespace visual {

// internal functions. maybe useful elsewhere, but not well documented nor stable.
namespace shape_fitter {

// Fit an Z-extruded concave, inward-facing shape.
//
// WARNING:
// Since this code use robust estimate, instead of min/max,
// some points will NOT be inside of the returned mesh.
std::pair<TriangleMesh<std::nullptr_t>, std::vector<Eigen::Vector2f>> fitExtrusion(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

// Fit an orinted bounding box (with Y-axis-only rotation) to
// given point cloud, and return (6) planes of the OBB.
//
// WARNING:
// Since this code use robust estimate, instead of min/max,
// some points will NOT be inside of the returned OBB.
//
// Return a box with 12 triangles.
TriangleMesh<std::nullptr_t> fitOBB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);


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

// Triangulate a CCW simple polygon (no self intersecting edges, no holes)
// into CCW triangles. N-vertex polygon always results in N-2 triangles.
//
// Useful course slide:
// https://www.cs.ucsb.edu/~suri/cs235/Triangulation.pdf
std::vector<std::array<int, 3>> triangulatePolygon(const std::vector<Eigen::Vector2f>& points);

// Extract XY 2D concave polygon. (CCW)
std::vector<Eigen::Vector2f> extractPolygon2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

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

TriangleMesh<std::nullptr_t> createBox(
Eigen::Vector3f center,Eigen::Vector3f half_dx,
	Eigen::Vector3f half_dy, Eigen::Vector3f half_dz);

}  // namespace
}  // namespace

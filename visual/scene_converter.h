#pragma once

#include <memory>
#include <tuple>
#include <vector>

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <visual/triangle_mesh.h>

namespace visual {

float mean(const std::pair<float, float>& pair);
float half(const std::pair<float, float>& pair);

// The list will be sorted.
std::pair<float, float> robustMinMax(std::vector<float>& values);

// Fit an Z-extruded concave, inward-facing shape.
//
// WARNING:
// Since this code use robust estimate, instead of min/max,
// some points will NOT be inside of the returned mesh.
class ExtrusionFitter {
public:
	ExtrusionFitter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

	// Return a box with 12 triangles.
	TriangleMesh<std::nullptr_t> extract() const;
private:
	// Extract XY 2D concave polygon. (CCW)
	static std::vector<Eigen::Vector2f> extractPolygon2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	// Extract Z range
	static std::pair<float, float> extractHeightRange(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

	// points: 2d points in shape [N, 2]
	// return: CCW list of points

	// Implementation of k-nearest neighbor based approach
	// http://repositorium.sdum.uminho.pt/xmlui/bitstream/handle/1822/6429/ConcaveHull_ACM_MYS.pdf?sequence=1
	static std::vector<Eigen::Vector2f> calculateConcaveHull(
		const std::vector<Eigen::Vector2f>& points, int k_min);

	static bool intersectSegments(
		std::pair<Eigen::Vector2f, Eigen::Vector2f> a0,
		std::pair<Eigen::Vector2f, Eigen::Vector2f> b0);
private:
	TriangleMesh<std::nullptr_t> mesh;
};


// Fit an orinted bounding box (with Y-axis-only rotation) to
// given point cloud, and return (6) planes of the OBB.
//
// WARNING:
// Since this code use robust estimate, instead of min/max,
// some points will NOT be inside of the returned OBB.
class OBBFitter {
public:
	OBBFitter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

	// Return a box with 12 triangles.
	TriangleMesh<std::nullptr_t> extract() const;
private:
	static TriangleMesh<std::nullptr_t> createBox(
		Eigen::Vector3f center,Eigen::Vector3f half_dx,
		Eigen::Vector3f half_dy, Eigen::Vector3f half_dz);
private:
	TriangleMesh<std::nullptr_t> mesh;
};

}  // namespace

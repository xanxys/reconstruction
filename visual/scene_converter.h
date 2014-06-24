#pragma once

#include <memory>
#include <tuple>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <visual/triangle_mesh.h>

namespace visual {

// Fit an orinted bounding box (with Y-axis-only rotation) to
// given point cloud, and return (6) planes of the OBB.
class OBBFitter {
public:
	OBBFitter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

	// Return a box with 12 triangles.
	TriangleMesh<std::nullptr_t> extract() const;
private:
	static float mean(const std::pair<float, float>& pair);
	static float half(const std::pair<float, float>& pair);

	static TriangleMesh<std::nullptr_t> createBox(
		Eigen::Vector3f center,Eigen::Vector3f half_dx,
		Eigen::Vector3f half_dy, Eigen::Vector3f half_dz);
	// The list will be sorted.
	static std::pair<float, float> robustMinMax(std::vector<float>& values);
private:
	TriangleMesh<std::nullptr_t> mesh;
};

}  // namespace

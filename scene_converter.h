#pragma once

#include <memory>
#include <tuple>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// A (oriented) plane described by ax + by + cz = d.
// which means normal = (a,b,c) (normalized).
class PlaneGeometry {
public:
	PlaneGeometry(float a, float b, float c, float d);
	PlaneGeometry(Eigen::Vector3f normal, float d);
	float a, b, c, d;
};


// Fit an orinted bounding box (with Y-axis-only rotation) to
// given point cloud, and return (6) planes of the OBB.
class OBBFitter {
public:
	OBBFitter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

	// Return all planes included in the given point cloud.
	std::vector<PlaneGeometry> extract();
private:
	static PlaneGeometry planeUpper(Eigen::Vector3f axis, float v);
	static PlaneGeometry planeLower(Eigen::Vector3f axis, float v);
	// The list will be sorted.
	static std::pair<float, float> robustMinMax(std::vector<float>& values);
private:
	std::vector<PlaneGeometry> planes;
};

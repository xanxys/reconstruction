// Colored point cloud -> 2D texture conversion
#pragma once

#include <array>
#include <iostream>
#include <limits>

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "triangle_mesh.h"

namespace visual {

// Get barycentric coordinate of the narest point on triangle surface.
Eigen::Vector2f nearestBarycentricApprox(
	const Eigen::Vector3f& v0, const Eigen::Vector3f& v1, const Eigen::Vector3f& v2, const Eigen::Vector3f& p);

// Get linear-interpolate coordinate of the surfact point, which is nearest to the given point.
// return: (distance, UV coords)
std::pair<float, Eigen::Vector2f> nearestCoordinate(
	const TriangleMesh<Eigen::Vector2f>& mesh, const Eigen::Vector3f p);

}  // namespace

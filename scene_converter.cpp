#include "scene_converter.h"

#include <algorithm>
#include <iostream>
#include <stdexcept>

#include <boost/math/constants/constants.hpp>
#include <boost/range/irange.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/point_types.h>

#include "logging.h"

typedef pcl::PointCloud<pcl::PointXYZ> ColorCloud;

using pcl::PointIndices;
using pcl::PointXYZ;

OBBFitter::OBBFitter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	const float angle_step = 0.05;

	// We call two horizontal axes in OBB (s,t) to avoid confusion.
	// when angle==0, (x,y,z) == (t,y,s)

	// Find smallest 2D OBB (st projection).
	INFO("Finding Y-rotation angle");
	float best_area = 1e3;
	float best_angle = -1;
	std::pair<float, float> best_s_range, best_t_range;
	for(float angle = 0; angle < boost::math::constants::pi<float>() / 2; angle += angle_step) {
		// Create z and x histogram of rotated cloud.
		std::vector<float> ss, ts;
		Eigen::Matrix2f rot_world_to_local = Eigen::Rotation2D<float>(angle).matrix();
		for(auto it = cloud->points.cbegin(); it != cloud->points.cend(); it++) {
		 	const Eigen::Vector2f pt_zx(it->z, it->x);
			const auto pt_st = rot_world_to_local * pt_zx;
			ss.push_back(pt_st(0));
			ts.push_back(pt_st(1));
		}

		// Create local AABB.
		const auto s_range = robustMinMax(ss);
		const auto t_range = robustMinMax(ts);
		const float area =
			(std::get<1>(s_range) - std::get<0>(s_range)) *
			(std::get<1>(t_range) - std::get<0>(t_range));

		DEBUG("@angle", angle, " area=", area);

		if(area < best_area) {
			best_area = area;
			best_angle = angle;
			best_s_range = s_range;
			best_t_range = t_range;
		}
	}
	assert(best_angle >= 0);

	// Find Y range (1D AABB).
	std::vector<float> ys;
	for(auto it = cloud->points.cbegin(); it != cloud->points.cend(); it++) {
		ys.push_back(it->y);
	}
	const auto y_range = robustMinMax(ys);

	// Construct 6 faces of the best OBB.
	const Eigen::Matrix2f local_to_world = Eigen::Rotation2D<float>(-best_angle).matrix();
	const auto splus_zx = local_to_world * Eigen::Vector2f(1, 0);
	const auto tplus_zx = local_to_world * Eigen::Vector2f(0, 1);

	Eigen::Vector3f splus(splus_zx(1), 0, splus_zx(0));
	Eigen::Vector3f yplus(0, 1, 0);
	Eigen::Vector3f tplus(tplus_zx(1), 0, tplus_zx(0));

	// Create box from Y,S,T axes.
	mesh = createBox(
		tplus * mean(best_t_range) +
		yplus * mean(y_range) +
		splus * mean(best_s_range),
		tplus * half(best_t_range),
		yplus * half(y_range),
		splus * half(best_s_range));
}

float OBBFitter::mean(const std::pair<float, float>& pair) {
	return (pair.first + pair.second) / 2;
}

float OBBFitter::half(const std::pair<float, float>& pair) {
	return (pair.second - pair.first) / 2;
}

TriangleMesh<std::nullptr_t> OBBFitter::createBox(
		Eigen::Vector3f center,Eigen::Vector3f half_dx,
		Eigen::Vector3f half_dy, Eigen::Vector3f half_dz) {
	TriangleMesh<std::nullptr_t> box;
	for(int i : boost::irange(0, 8)) {
		const Eigen::Vector3f vertex_pos = center +
			((i & 0b001) ? 1 : -1) * half_dx +
			((i & 0b010) ? 1 : -1) * half_dy +
			((i & 0b100) ? 1 : -1) * half_dz;
		box.vertices.push_back(std::make_pair(vertex_pos, nullptr));
	}

	// Create inward-facing triangles. (CCW is positive direction)
	// Draw a cube with 000-111 to understand this.
	box.triangles = {
		// X-
		std::make_tuple(0, 2, 4),
		std::make_tuple(6, 4, 2),
		// X+
		std::make_tuple(5, 7, 1),
		std::make_tuple(3, 1, 7),
		// Y-
		std::make_tuple(0, 4, 1),
		std::make_tuple(5, 1, 4),
		// Y+
		std::make_tuple(6, 2, 7),
		std::make_tuple(3, 7, 2),
		// Z-
		std::make_tuple(0, 1, 2),
		std::make_tuple(3, 2, 1),
		// Z+
		std::make_tuple(6, 7, 4),
		std::make_tuple(5, 4, 7)
	};

	return box;
}

std::pair<float, float> OBBFitter::robustMinMax(std::vector<float>& values) {
	std::sort(values.begin(), values.end());
	return std::make_pair(
		values[int(values.size() * 0.01)],
		values[int(values.size() * 0.99)]);
}

TriangleMesh<std::nullptr_t> OBBFitter::extract() const {
	return mesh;
}

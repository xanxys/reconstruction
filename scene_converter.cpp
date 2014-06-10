#include "scene_converter.h"

#include <algorithm>
#include <iostream>
#include <stdexcept>

#include <boost/math/constants/constants.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/point_types.h>

#include "logging.h"

typedef pcl::PointCloud<pcl::PointXYZ> ColorCloud;

using pcl::PointIndices;
using pcl::PointXYZ;

PlaneGeometry::PlaneGeometry(float a, float b, float c, float d) :
	a(a), b(b), c(c), d(d) {
}

PlaneGeometry::PlaneGeometry(Eigen::Vector3f normal, float d) :
	a(normal(0)), b(normal(1)), c(normal(2)), d(d) {
}


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

		INFO("@angle", angle, " area=", area);

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

	// floor & ceiling
	planes.push_back(planeLower(yplus, std::get<0>(y_range)));
	planes.push_back(planeUpper(yplus, std::get<1>(y_range)));

	// S
	planes.push_back(planeLower(splus, std::get<0>(best_s_range)));
	planes.push_back(planeUpper(splus, std::get<1>(best_s_range)));

	// T
	planes.push_back(planeLower(tplus, std::get<0>(best_t_range)));
	planes.push_back(planeUpper(tplus, std::get<1>(best_t_range)));
}

PlaneGeometry OBBFitter::planeUpper(Eigen::Vector3f axis, float v) {
	return PlaneGeometry(-axis, -v);
}

PlaneGeometry OBBFitter::planeLower(Eigen::Vector3f axis, float v) {
	return PlaneGeometry(axis, v);
}

std::pair<float, float> OBBFitter::robustMinMax(std::vector<float>& values) {
	std::sort(values.begin(), values.end());
	return std::make_pair(
		values[int(values.size() * 0.01)],
		values[int(values.size() * 0.99)]);
}

std::vector<PlaneGeometry> OBBFitter::extract() {
	return planes;
}


#include "floor_belief.h"

#include <map>
#include <cmath>
#include <exception>
#include <random>

#include <boost/range/irange.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using ColorCloud = pcl::PointCloud<pcl::PointXYZRGBA>;
using RigidTrans3f = Eigen::Transform<float, 3, Eigen::AffineCompact>;

const double pi = 3.14159265359;


FloorBelief::FloorBelief(const FloorBelief& that) :
	log(that.log.str()), manhattan(that.manhattan), index(that.index) {
}

FloorBelief::FloorBelief(const ManhattanBelief& manhattan, int index) :
	manhattan(manhattan), index(index) {
}

std::vector<std::shared_ptr<FloorBelief>> FloorBelief::expand(const ManhattanBelief& manhattan) {
	std::vector<std::shared_ptr<FloorBelief>> results;

	int iy_floor = 0;
	for(const auto& pair : manhattan.getVoxelsDetailed()) {
		if(pair.second.state == VoxelState::OCCUPIED) {
			iy_floor = std::max(iy_floor, std::get<1>(pair.first));
		}
	}

	results.push_back(std::make_shared<FloorBelief>(
		manhattan, iy_floor));

	results.push_back(std::make_shared<FloorBelief>(
		manhattan, iy_floor - 1));

	return results;
}

#include "indoor.h"

namespace recon {

RoomFrame::RoomFrame() {
	up = Eigen::Vector3f(0, 0, 1);
}

void RoomFrame::setHRange(float z0, float z1) {
	assert(z0 < z1);
	this->z0 = z0;
	this->z1 = z1;
}

std::pair<float, float> RoomFrame::getHRange() const {
	return std::make_pair(z0, z1);
}

std::vector<Eigen::Vector2f> RoomFrame::getSimplifiedContour() const {
	assert(wall_polygon.size() >= 3);
	return wall_polygon;
}

}  // namespace

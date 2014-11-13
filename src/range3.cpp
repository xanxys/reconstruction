#include "range3.h"

range3::range3(Eigen::Vector3i max) :
	x0(0), y0(0), z0(0),
	x1(max.x()), y1(max.y()), z1(max.z()) {
}

range3::range3(Eigen::Vector3i min, Eigen::Vector3i max) :
	x0(min.x()), y0(min.y()), z0(min.z()),
	x1(max.x()), y1(max.y()), z1(max.z()) {
}

range3::iterator range3::begin() {
	return iterator(x0, y0, z0, x0, y0, x1, y1);
}

range3::iterator range3::end() {
	return iterator(x0, y0, z1, x0, y0, x1, y1);
}


range3::iterator::iterator(
	int x, int y, int z, int x0, int y0, int x1, int y1) :
	x(x), y(y), z(z), x0(x0), y0(y0), x1(x1), y1(y1) {
}

range3::iterator& range3::iterator::operator++() {
	x++;
	if(x == x1) {
		x = x0;
		y++;
	}
	if(y == y1) {
		y = y0;
		z++;
	}
	return *this;
}

Eigen::Vector3i range3::iterator::operator*() {
	return Eigen::Vector3i(x, y, z);
}

bool range3::iterator::operator!=(const iterator& that) {
	return (x != that.x) || (y != that.y) || (z != that.z);
}

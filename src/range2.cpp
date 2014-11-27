#include "range2.h"

namespace recon {

range2::range2(Eigen::Vector2i vmax) :
	x0(0), y0(0),
	x1(vmax.x()), y1(vmax.y()) {
}

range2::range2(int xmax, int ymax) :
	x0(0), y0(0), x1(xmax), y1(ymax) {
}

range2::range2(Eigen::Vector2i vmin, Eigen::Vector2i vmax) :
	x0(vmin.x()), y0(vmin.y()) ,
	x1(vmax.x()), y1(vmax.y()) {
}

range2::range2(int x0, int y0, int x1, int y1) :
	x0(x0), y0(y0), x1(x1), y1(y1) {
}


range2::iterator range2::begin() {
	return iterator(x0, y0, x0, x1);
}

range2::iterator range2::end() {
	return iterator(x0, y1, x0, x1);
}


range2::iterator::iterator(
	int x, int y, int x0, int x1) :
	x(x), y(y), x0(x0), x1(x1) {
}

range2::iterator& range2::iterator::operator++() {
	x++;
	if(x == x1) {
		x = x0;
		y++;
	}
	return *this;
}

Eigen::Vector2i range2::iterator::operator*() {
	return Eigen::Vector2i(x, y);
}

bool range2::iterator::operator!=(const iterator& that) {
	return (x != that.x) || (y != that.y);
}

}  // namespace

#pragma once

#include <Eigen/Dense>

// Convenient class to iterate rectangular 2D region.
// range2(vmin, vmax) scans [vmin, vmax).
// e.g. range2([0,0], [2,2])
// -> (0,0), (1,0), (0,1), (1,1)
class range2 {
public:
	// This is needed to support increment.
	class iterator {
	public:
		iterator(int x, int y, int x0, int x1);
		iterator& operator++();
		Eigen::Vector2i operator*();
		bool operator!=(const iterator& that);
	private:
		int x, y;
		int x0, x1;
	};

	// [0, max)
	range2(Eigen::Vector2i vmax);
	range2(int xmax, int ymax);

	// [min, max)
	range2(Eigen::Vector2i vmin, Eigen::Vector2i vmax);
	range2(int x0, int y0, int x1, int y1);

	iterator begin();
	iterator end();
private:
	int x0, y0;
	int x1, y1;
};

#pragma once

#include <Eigen/Dense>

namespace recon {

// Convenient class to iterate rectangular 3D region.
// range3(vmin, vmax) scans [vmin, vmax).
// e.g. range3([0,0,0], [2,2,2])
// -> (0,0,0), (1,0,0), (0,1,0), ..., (1,1,1)
class range3 {
public:
	// This is needed to support increment.
	class iterator {
	public:
		iterator(int x, int y, int z, int x0, int y0, int x1, int y1);
		iterator& operator++();
		Eigen::Vector3i operator*();
		bool operator!=(const iterator& that);
	private:
		int x, y, z;
		int x0, y0;
		int x1, y1;
	};

	// [0, max)
	range3(Eigen::Vector3i max);

	// [min, max)
	range3(Eigen::Vector3i min, Eigen::Vector3i max);

	iterator begin();
	iterator end();
private:
	int x0, y0, z0;
	int x1, y1, z1;
};

}  // namespace

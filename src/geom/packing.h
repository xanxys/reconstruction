#pragma once

#include <array>
#include <vector>

#include <Eigen/Dense>

namespace recon {

// Try to find packing of rectangles in a (minimum) square region.
// Obviously, it's NP-hard problem, so don't expect near-optimal result.
// This function is optimized for horizontally-long rectangles.
//
// rectangles: size of each rectangle ((0,0) is assumed to be origins)
// return: (size of square, offset of rectangle)
// returned offset is such that all rectangles will fit in [0,square]^2
// without any overlaps.
std::pair<float, std::vector<Eigen::Vector2f>>
	packRectangles(std::vector<Eigen::Vector2f> rectangles);

}  // namespace

// 3D -> 2D texture conversion
#pragma once

#include <array>
#include <iostream>
#include <cmath>
#include <vector>

#include <boost/range/irange.hpp>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include <logging.h>
#include <visual/mapping.h>
#include <visual/marching_cubes.h>

namespace recon {

Eigen::Vector2f swapY(const Eigen::Vector2f& v);

cv::Point2i eigenToCV(const Eigen::Vector2f& v);

cv::Mat visualizeUVMap(const TriangleMesh<Eigen::Vector2f>& mesh);


// color: [0-1]^3, RGB
// output, uint8*3 image (in BGR, which is opencv standard)
cv::Mat bake3DTexture(
	const TriangleMesh<Eigen::Vector2f>& mesh,
	std::function<Eigen::Vector3f(Eigen::Vector3f)> colorField
	);

}  // namespace

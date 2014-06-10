#pragma once

#include <array>
#include <iostream>
#include <cmath>
#include <vector>

#include <boost/range/irange.hpp>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include "logging.h"
#include "marching_cubes.h"
#include "mapping.h"

Eigen::Vector2f swapY(const Eigen::Vector2f& v);

cv::Point2i eigenToCV(const Eigen::Vector2f& v);

cv::Mat visualizeUVMap(const TriangleMesh<std::pair<Eigen::Vector3f, Eigen::Vector2f>>& mesh);


// color: [0-1]^3, RGB
// output, uint8*3 image (in BGR, which is opencv standard)
cv::Mat bake3DTexture(
	const TriangleMesh<Eigen::Vector2f>& mesh,
	std::function<Eigen::Vector3f(Eigen::Vector3f)> colorField
	);


TriangleMesh<Eigen::Vector2f> dropNormal(const TriangleMesh<std::pair<Eigen::Vector3f, Eigen::Vector2f>>& mesh);


void writeObjMaterial(std::ostream& output);

// 3D -> 2D texture conversion
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

cv::Mat visualizeUVMap(const TriangleMesh<Eigen::Vector2f>& mesh);


// color: [0-1]^3, RGB
// output, uint8*3 image (in BGR, which is opencv standard)
cv::Mat bake3DTexture(
	const TriangleMesh<Eigen::Vector2f>& mesh,
	std::function<Eigen::Vector3f(Eigen::Vector3f)> colorField
	);


template<typename TypeFirst, typename TypeSecond>
TriangleMesh<TypeSecond> mapSecond(const TriangleMesh<std::pair<TypeFirst, TypeSecond>>& mesh) {
	TriangleMesh<Eigen::Vector2f> mesh_snd;
	mesh_snd.triangles = mesh.triangles;
	mesh_snd.vertices.resize(mesh.vertices.size());
	std::transform(mesh.vertices.begin(), mesh.vertices.end(), mesh_snd.vertices.begin(),
		[](const std::pair<Eigen::Vector3f, std::pair<TypeFirst, TypeSecond>>& vertex) {
			return std::make_pair(vertex.first, vertex.second.second);
		});
	return mesh_snd;
}

void writeObjMaterial(std::ostream& output, std::string texture_path);

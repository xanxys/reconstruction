#include <array>
#include <iostream>
#include <fstream>

#include "recon_server.h"

#include <cmath>
#include "logging.h"
#include "marching_cubes.h"
#include "mapping.h"

cv::Point2i eigenToCV(const Eigen::Vector2f& v) {
	return cv::Point2i(v(0), v(1));
}

cv::Mat visualizeUVMap(const TriangleMesh<std::pair<Eigen::Vector3f, Eigen::Vector2f>>& mesh) {
	const int image_size = 2048;
	cv::Mat image(image_size, image_size, CV_8UC3);
	image = cv::Scalar(0, 0, 0);
	const cv::Scalar color(0, 0, 255);
	for(const auto& tri : mesh.triangles) {
		const std::array<Eigen::Vector2f, 3> uvs = {
			mesh.vertices[std::get<0>(tri)].second.second,
			mesh.vertices[std::get<1>(tri)].second.second,
			mesh.vertices[std::get<2>(tri)].second.second
		};

		for(int i : boost::irange(0, 3)) {
			cv::line(image,
				eigenToCV(uvs[i] * image_size),
				eigenToCV(uvs[(i + 1) % 3] * image_size),
				color);
		}
	}
	return image;
}

TriangleMesh<Eigen::Vector2f> dropNormal(const TriangleMesh<std::pair<Eigen::Vector3f, Eigen::Vector2f>>& mesh) {
	TriangleMesh<Eigen::Vector2f> mesh_uv;
	mesh_uv.triangles = mesh.triangles;
	mesh_uv.vertices.resize(mesh.vertices.size());
	std::transform(mesh.vertices.begin(), mesh.vertices.end(), mesh_uv.vertices.begin(),
		[](const std::pair<Eigen::Vector3f, std::pair<Eigen::Vector3f, Eigen::Vector2f>>& vertex) {
			return std::make_pair(vertex.first, vertex.second.second);
		});
	return mesh_uv;
}

int main() {
	INFO("creating metaball");
	const auto mesh = extractIsosurface(
		3, [](Eigen::Vector3f p) {
			return (
				1 / (0.01 + p.norm()) +
				1 / (0.01 + (p - Eigen::Vector3f(0.9,0.9,0.9)).norm())
				);
		}, std::make_pair(
			Eigen::Vector3f(-3, -3, -3),
			Eigen::Vector3f(3, 3, 3)),
		0.1);
	std::ofstream test("test.ply");
	mesh.serializePLY(test);

	const auto mesh_uv = assignUV(mesh);
	cv::imwrite("uv.png", visualizeUVMap(mesh_uv));
	std::ofstream test_uv("test_uv.obj");

	dropNormal(mesh_uv).serializeObjWithUv(test_uv);

	INFO("Launching HTTP server");
	ReconServer server;
	server.launch();
	while(true) {
		std::string dummy;
		std::cin >> dummy;
	}
	return 0;
}

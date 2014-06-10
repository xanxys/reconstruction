#include <array>
#include <iostream>
#include <fstream>
#include <limits>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "logging.h"
#include "mapping.h"
#include "marching_cubes.h"
#include "recon_server.h"
#include "scene_converter.h"
#include "texture_conversion.h"

void testMeshIO() {
	INFO("creating metaball");
	const auto mesh_n = extractIsosurface(
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
	mesh_n.serializePLY(test);

	const auto mesh_n_uv = assignUV(mesh_n);
	const auto mesh_uv = mapSecond(mesh_n_uv);
	cv::imwrite("uv.png", visualizeUVMap(mesh_uv));
	cv::imwrite("uv_3d.png", bake3DTexture(mesh_uv,
		[](Eigen::Vector3f p) {
			return p;
		}));

	std::ofstream test_uv("test_uv.obj");
	std::ofstream test_mat("test_uv.mtl");
	mesh_uv.serializeObjWithUv(test_uv, "test_uv.mtl");
	writeObjMaterial(test_mat);
}

// Get barycentric coordinate of the narest point on triangle surface.
Eigen::Vector2f nearestBarycentricApprox(
	const Eigen::Vector3f& v0, const Eigen::Vector3f& v1, const Eigen::Vector3f& v2, const Eigen::Vector3f& p) {
	const auto d1 = v1 - v0;
	const auto d2 = v2 - v0;
	Eigen::Matrix3f m;
	m.col(0) = d1;
	m.col(1) = d2;
	m.col(2) = d1.cross(d2);

	const Eigen::Vector3f coord = m.inverse() * (p - v0);
	// approximate barycentric coordinates.
	float bary0 = std::max(0.0f, coord(0));
	float bary1 = std::max(0.0f, coord(1));
	if(bary1 + bary0 > 1) {
		const float scale = 1 / (bary0 + bary1);
		bary0 *= scale;
		bary1 *= scale;
	}
	return Eigen::Vector2f(bary0, bary1);
}

// Get linear-interpolate coordinate of the surfact point, which is nearest to the given point.
Eigen::Vector2f nearestCoordinate(
	const TriangleMesh<Eigen::Vector2f>& mesh, const Eigen::Vector3f p) {
	float min_dist = std::numeric_limits<float>::max();
	Eigen::Vector2f coord;
	for(const auto& tri : mesh.triangles) {
		const Eigen::Vector3f v0 = mesh.vertices[std::get<0>(tri)].first;
		const Eigen::Vector3f v1 = mesh.vertices[std::get<1>(tri)].first;
		const Eigen::Vector3f v2 = mesh.vertices[std::get<2>(tri)].first;
		const auto bary = nearestBarycentricApprox(v0, v1, v2, p);

		const Eigen::Vector3f pt_nearest = v0 + (v1 - v0) * bary(0) + (v2 - v0) * bary(1);
		const float dist = (p - pt_nearest).norm();
		if(dist < min_dist) {
			min_dist = dist;

			const Eigen::Vector2f c0 = mesh.vertices[std::get<0>(tri)].second;
			const Eigen::Vector2f c1 = mesh.vertices[std::get<1>(tri)].second;
			const Eigen::Vector2f c2 = mesh.vertices[std::get<2>(tri)].second;
			coord = c0 + (c1 - c0) * bary(0) + (c2 - c0) * bary(1);
		}
	}
	return coord;
}

void testPointCloudMeshing() {
	INFO("Creating textured mesh from point cloud");
	std::ifstream test("ocha_points.json");
	Json::Value cloud;
	Json::Reader().parse(test, cloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>());
	for(const auto& point : cloud) {
		pcl::PointXYZ pt;
		pt.x = point["x"].asDouble();
		pt.y = point["y"].asDouble();
		pt.z = point["z"].asDouble();
		cloud_pcl->points.push_back(pt);
	}

	TriangleMesh<std::nullptr_t> mesh = OBBFitter(cloud_pcl).extract();
	const auto mesh_uv = mapSecond(assignUV(mesh));
	std::ofstream room_box_uv("room_box_uv.obj");
	mesh_uv.serializeObjWithUv(room_box_uv, "uv_debug.mtl");

	cv::imwrite("uv_box.png", visualizeUVMap(mesh_uv));

	// Project points to the surface and draw circle onto texture.
	const int tex_size = 1024;
	cv::Mat texture(tex_size, tex_size, CV_8UC3);
	texture = cv::Scalar(0, 0, 0);
	for(const auto& point : cloud) {
		const Eigen::Vector3f pos(
			point["x"].asDouble(),
			point["y"].asDouble(),
			point["z"].asDouble());
		const auto uv = nearestCoordinate(mesh_uv, pos);
		// DEBUG("nearestUV", uv(0), uv(1));

		const cv::Scalar color(
			point["b"].asDouble(),
			point["g"].asDouble(),
			point["r"].asDouble());

		cv::circle(
			texture, eigenToCV(swapY(uv) * tex_size), 3,
			color, -1);
	}
	cv::imwrite("uv_pt_baked.png", texture);
}

int main() {
	testMeshIO();
	testPointCloudMeshing();

	INFO("Launching HTTP server");
	ReconServer server;
	server.launch();
	while(true) {
		std::string dummy;
		std::cin >> dummy;
	}
	return 0;
}

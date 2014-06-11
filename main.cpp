#include <array>
#include <iostream>
#include <fstream>
#include <limits>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cloud_baker.h>
#include <cloud_conversion.h>
#include <logging.h>
#include <mapping.h>
#include <marching_cubes.h>
#include <recon_server.h>
#include <scene_converter.h>
#include <texture_conversion.h>

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
	writeObjMaterial(test_mat, "uv_3d.png");
}


void testPointCloudMeshing() {
	INFO("Creating textured mesh from point cloud");
	std::ifstream test("ocha_points.json");
	Json::Value cloud;
	Json::Reader().parse(test, cloud);

	CloudBaker(cloud).writeWavefrontObject();

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

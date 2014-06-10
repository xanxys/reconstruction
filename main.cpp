#include <array>
#include <iostream>
#include <fstream>

#include "recon_server.h"

#include "logging.h"
#include "mapping.h"
#include "marching_cubes.h"
#include "texture_conversion.h"

void testMeshIO() {
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
	cv::imwrite("uv_3d.png", bake3DTexture(dropNormal(mesh_uv),
		[](Eigen::Vector3f p) {
			return p;
		}));

	std::ofstream test_uv("test_uv.obj");
	std::ofstream test_mat("test_uv.mtl");
	dropNormal(mesh_uv).serializeObjWithUv(test_uv, "test_uv.mtl");
	writeObjMaterial(test_mat);
}

int main() {
	testMeshIO();

	INFO("Launching HTTP server");
	ReconServer server;
	server.launch();
	while(true) {
		std::string dummy;
		std::cin >> dummy;
	}
	return 0;
}

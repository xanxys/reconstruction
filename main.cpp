#include <iostream>
#include <fstream>

#include "recon_server.h"

#include <cmath>
#include "marching_cubes.h"

int main() {
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
	return 0;

	ReconServer server;
	server.launch();
	while(true) {
		std::string dummy;
		std::cin >> dummy;
	}
	return 0;
}

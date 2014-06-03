#include <iostream>
#include <fstream>

#include "recon_server.h"

#include <cmath>
#include "marching_cubes.h"

int main() {
	const auto mesh = extractIsosurface(
		0.5, [](Eigen::Vector3f p) {
			return std::max(0.0f, 1 - p.norm());
		}, std::make_pair(
			Eigen::Vector3f(-2, -2, -2),
			Eigen::Vector3f(2, 2, 2)),
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

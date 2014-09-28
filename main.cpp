#include <array>
#include <iostream>
#include <fstream>
#include <limits>
#include <map>
#include <random>

#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>

#include <logging.h>
#include <server/recon_server.h>
#include <visual/cloud_baker.h>
#include <visual/cloud_conversion.h>
#include <visual/mapping.h>
#include <visual/marching_cubes.h>
#include <visual/scene_recognizer.h>
#include <visual/shape_fitter.h>
#include <visual/texture_conversion.h>

/* Bunch of examples on how to use codes */

void testMeshIO() {
	INFO("creating metaball");
	const auto mesh_n = visual::extractIsosurface(
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

	const auto mesh_n_uv = visual::assignUV(mesh_n);
	const auto mesh_uv = visual::mapSecond(mesh_n_uv);
	cv::imwrite("uv.png", visual::visualizeUVMap(mesh_uv));
	cv::imwrite("uv_3d.png", visual::bake3DTexture(mesh_uv,
		[](Eigen::Vector3f p) {
			return p;
		}));

	std::ofstream test_uv("test_uv.obj");
	std::ofstream test_mat("test_uv.mtl");
	mesh_uv.serializeObjWithUv(test_uv, "test_uv.mtl");
	visual::writeObjMaterial(test_mat, "uv_3d.png");
}

std::string guessSceneName(const std::string& scan_path) {
	std::vector<std::string> components;
	boost::split(components, scan_path, boost::is_any_of("/"));
	if(components.size() >= 1 && components.back() != "") {
		return components.back();
	} else if(components.size() >= 2 && components[components.size() - 2] != "") {
		return components[components.size() - 2];
	} else {
		return "result_bundle";
	}
}


int main(int argc, char** argv) {
	using boost::program_options::notify;
	using boost::program_options::options_description;
	using boost::program_options::parse_command_line;
	using boost::program_options::store;
	using boost::program_options::value;
	using boost::program_options::variables_map;

	options_description desc("Reconstruct 3D scene from scans.");
	desc.add_options()
		("help", "show this message")
		("test", "do experimental stuff")
		("second", value<std::string>(), "do second pass")
		("convert", value<std::vector<std::string>>()->multitoken(), "convert given scans");

	variables_map vars;
	store(parse_command_line(argc, argv, desc), vars);
	notify(vars);

	if(vars.count("help") > 0) {
		std::cout << desc << std::endl;
		return 0;
	} else if(vars.count("test") > 0) {
		testMeshIO();
		return 0;
	} else if(vars.count("convert") > 0) {
		const auto dir_paths = vars["convert"].as<std::vector<std::string>>();
		if(dir_paths.empty()) {
			ERROR("Need one or more scans to proceed");
			return 1;
		}
		INFO("Loading scans, #scans=", (int)dir_paths.size());
		std::vector<visual::SingleScan> scans;
		for(const auto& dir_path : dir_paths) {
			scans.emplace_back(dir_path);
		}

		INFO("Converting to a scene");
		visual::SceneAssetBundle bundle(guessSceneName(dir_paths.front()));
		visual::scene_recognizer::recognizeScene(bundle, scans);
		return 0;
	} else if(vars.count("second") > 0) {
		const std::string dir_path = vars["second"].as<std::string>();
		INFO("Opening existing SceneAssetBundle");
		visual::SceneAssetBundle bundle(guessSceneName(dir_path), 1000);
		visual::scene_recognizer::recognizeScene2(bundle);
		return 0;
	} else {
		INFO("Launching HTTP server");
		server::ReconServer server;
		server.launch();
		while(true) {
			std::string dummy;
			std::cin >> dummy;
		}
		return 0;
	}
}

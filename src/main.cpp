#include <fstream>
#include <iostream>

#include <boost/program_options.hpp>
#include <jsoncpp/json/json.h>

#include <logging.h>
#include <recog/scene_asset_bundle.h>
#include <recog/scene_recognizer.h>

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
		("debug", "output debug / visualization data (slow)")
		("toy", "Generate toy scene")
		("convert", value<std::vector<std::string>>()->multitoken(), "convert given scans")
		("sound", value<std::string>(), "Path to a directory containing processed collision sound wave files.")
		("hint", value<std::string>(), "Alignment hint (json path)")
		("output", value<std::string>(), "scene asset output path");

	variables_map vars;
	store(parse_command_line(argc, argv, desc), vars);
	notify(vars);

	// Do specified action.
	if(vars.count("help") > 0) {
		std::cout << desc << std::endl;
		return 0;
	} else if(vars.count("convert") > 0) {
		// Argument sanity check.
		if(vars.count("output") == 0) {
			std::cerr << "--output is now required" << std::endl;
			return -1;
		}
		if(vars.count("hint") == 0) {
			std::cerr << "--hint is required" << std::endl;
			return -1;
		}
		if(vars.count("sound") == 0) {
			WARN("--sound not specified, collision sound will NOT be generated");
		}

		// Load scans.
		const auto dir_paths = vars["convert"].as<std::vector<std::string>>();
		INFO("Loading scans, #scans=", (int)dir_paths.size());
		std::vector<recon::SingleScan> scans;
		for(const auto& dir_path : dir_paths) {
			scans.emplace_back(dir_path);
		}
		// Load hint.
		Json::Value hint;
		{
			Json::Reader reader;
			std::ifstream test(vars["hint"].as<std::string>());
			const bool success = reader.parse(test, hint, false);
			if(!success) {
				WARN("couldn't parse scan hint json file");
				throw std::runtime_error(reader.getFormatedErrorMessages());
			}
		}
		// Recognize.
		INFO("Converting to a scene");
		const bool debug = vars.count("debug");
		recon::SceneAssetBundle bundle(
			vars["output"].as<std::string>(), debug);
		recon::recognizeScene(bundle, scans, hint);
		if(vars.count("sound") > 0) {
			bundle.addCollisionSoundFromDir(vars["sound"].as<std::string>());
		}
		return 0;
	} else if(vars.count("toy") > 0) {
		// Argument sanity check.
		if(vars.count("output") == 0) {
			std::cerr << "--output is now required" << std::endl;
			return -1;
		}
		if(vars.count("sound") == 0) {
			WARN("--sound not specified, collision sound will NOT be generated");
		}

		// Generate.
		const bool debug = vars.count("debug");
		recon::SceneAssetBundle bundle(
			vars["output"].as<std::string>(), debug);
		recon::populateToyScene(bundle);
		if(vars.count("sound") > 0) {
			bundle.addCollisionSoundFromDir(vars["sound"].as<std::string>());
		}
		return 0;
	} else {
		std::cout << desc << std::endl;
		return -1;
	}
}

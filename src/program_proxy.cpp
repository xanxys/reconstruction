#include "program_proxy.h"

#include <stdlib.h>

#include <fstream>
#include <iostream>
#include <random>

#include <boost/filesystem.hpp>

#include <logging.h>


Json::Value call_external(const std::string& prog_path, Json::Value arg) {
	std::random_device rd;
	std::uniform_int_distribution<uint32_t> dist(0, 0xffffffff);
	const std::string id = std::to_string(dist(rd));
	const std::string path_in = "/tmp/recon-prog_proxy-in-" + id;
	const std::string path_out = "/tmp/recon-prog_proxy-out" + id;

	std::ofstream of(path_in);
	of << Json::FastWriter().write(arg) << std::endl;
	of.close();

	INFO("Calling external program", prog_path);
	const int status = system(("cat " + path_in + " | " + prog_path + " > " + path_out).c_str());
	if(status != 0) {
		throw std::runtime_error("call_external: system() failed with " + std::to_string(status));
	}

	std::ifstream in_f(path_out);
	Json::Value result;
	Json::Reader().parse(in_f, result, false);
	return result;
}

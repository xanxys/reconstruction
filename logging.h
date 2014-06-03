#pragma once

#include <iostream>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <jsoncpp/json/json.h>


#define INFO(msg) logJson("INFO", __FILE__, __LINE__, msg)

template<typename MessageType>
void logJson(std::string level, std::string path, int line, MessageType msg) {
	Json::Value entry;
	entry["msg"] = msg;
	entry["date"] = boost::posix_time::to_iso_extended_string(boost::posix_time::microsec_clock::universal_time());
	entry["level"] = level;
	entry["loc"]["path"] = path;
	entry["loc"]["line"] = line;
	std::cout << Json::FastWriter().write(entry) << std::endl;
}

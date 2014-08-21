#include "logging.h"

void logJsonRaw(std::string level, std::string path, int line, Json::Value& msg) {
	Json::Value entry;
	entry["msg"] = msg;
	entry["date"] = boost::posix_time::to_iso_extended_string(boost::posix_time::microsec_clock::universal_time());
	entry["level"] = level;
	entry["loc"]["path"] = path;
	entry["loc"]["line"] = line;
	std::cout << Json::FastWriter().write(entry);
}

Json::Value packMessageReversed() {
	return Json::arrayValue;
}

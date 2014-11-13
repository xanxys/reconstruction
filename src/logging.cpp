#include "logging.h"

#include <limits>
#include <random>

void logJsonRaw(std::string level, std::string path, int line, Json::Value& msg) {
	static std::string session_id = "";
	if(session_id == "") {
		std::random_device rd;
		using SessionId = uint64_t;
		const SessionId v = std::uniform_int_distribution<SessionId>(0, std::numeric_limits<SessionId>::max())(rd);
		session_id = std::to_string(v);
	}
	Json::Value entry;
	entry["session_id"] = session_id;
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

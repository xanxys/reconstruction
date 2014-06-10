#pragma once

#include <iostream>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <jsoncpp/json/json.h>


#define INFO(...) logJson("INFO", __FILE__, __LINE__, __VA_ARGS__)

template<typename MessageType>
void logJson(std::string level, std::string path, int line, MessageType msg) {
	Json::Value entry;
	entry["msg"] = msg;
	entry["date"] = boost::posix_time::to_iso_extended_string(boost::posix_time::microsec_clock::universal_time());
	entry["level"] = level;
	entry["loc"]["path"] = path;
	entry["loc"]["line"] = line;
	std::cout << Json::FastWriter().write(entry);
}


template<typename MessageType1, typename MessageType2>
void logJson(std::string level, std::string path, int line,
	MessageType1 msg1, MessageType2 msg2) {
	Json::Value entry;
	entry["msg"].append(msg1);
	entry["msg"].append(msg2);
	entry["date"] = boost::posix_time::to_iso_extended_string(boost::posix_time::microsec_clock::universal_time());
	entry["level"] = level;
	entry["loc"]["path"] = path;
	entry["loc"]["line"] = line;
	std::cout << Json::FastWriter().write(entry);
}

template<typename MessageType1, typename MessageType2, typename MessageType3>
void logJson(std::string level, std::string path, int line,
	MessageType1 msg1, MessageType2 msg2, MessageType3 msg3) {
	Json::Value entry;
	entry["msg"].append(msg1);
	entry["msg"].append(msg2);
	entry["msg"].append(msg3);
	entry["date"] = boost::posix_time::to_iso_extended_string(boost::posix_time::microsec_clock::universal_time());
	entry["level"] = level;
	entry["loc"]["path"] = path;
	entry["loc"]["line"] = line;
	std::cout << Json::FastWriter().write(entry);
}

template<typename MessageType1, typename MessageType2, typename MessageType3, typename MessageType4>
void logJson(std::string level, std::string path, int line,
	MessageType1 msg1, MessageType2 msg2, MessageType3 msg3, MessageType4 msg4) {
	Json::Value entry;
	entry["msg"].append(msg1);
	entry["msg"].append(msg2);
	entry["msg"].append(msg3);
	entry["msg"].append(msg4);
	entry["date"] = boost::posix_time::to_iso_extended_string(boost::posix_time::microsec_clock::universal_time());
	entry["level"] = level;
	entry["loc"]["path"] = path;
	entry["loc"]["line"] = line;
	std::cout << Json::FastWriter().write(entry);
}

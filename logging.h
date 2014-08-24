#pragma once

#include <iostream>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <jsoncpp/json/json.h>


#define INFO(...) logJson("INFO", __FILE__, __LINE__, __VA_ARGS__)
#define DEBUG(...) logJson("DEBUG", __FILE__, __LINE__, __VA_ARGS__)
#define WARN(...) logJson("WARN", __FILE__, __LINE__, __VA_ARGS__)
#define ERROR(...) logJson("ERROR", __FILE__, __LINE__, __VA_ARGS__)

void logJsonRaw(std::string level, std::string path, int line, Json::Value& msg);

Json::Value packMessageReversed();

template<typename MessageType0, typename... MessageTypes>
Json::Value packMessageReversed(MessageType0 msg_head, MessageTypes... msg_tail) {
	Json::Value rest = packMessageReversed(msg_tail...);
	rest.append(msg_head);
	return rest;
}


template<typename... MessageType>
void logJson(std::string level, std::string path, int line, MessageType... msgs) {
	Json::Value message_rev = packMessageReversed(msgs...);
	if(message_rev.size() == 1) {
		logJsonRaw(level, path, line, message_rev[0]);
	} else {
		Json::Value array;
		for(int i = message_rev.size() - 1; i >= 0; i--) {
			array.append(message_rev[i]);
		}
		logJsonRaw(level, path, line, array);
	}
}

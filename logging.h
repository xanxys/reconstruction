#pragma once

#include <iostream>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <jsoncpp/json/json.h>


#define INFO(...) logJson("INFO", __FILE__, __LINE__, __VA_ARGS__)
#define DEBUG(...) logJson("DEBUG", __FILE__, __LINE__, __VA_ARGS__)
#define WARN(...) logJson("WARN", __FILE__, __LINE__, __VA_ARGS__)
#define ERROR(...) logJson("ERROR", __FILE__, __LINE__, __VA_ARGS__)

void logJsonRaw(std::string level, std::string path, int line, Json::Value& msg);

template<typename MessageType>
void logJson(std::string level, std::string path, int line, MessageType msg) {
	Json::Value message = msg;
	logJsonRaw(level, path, line, message);
}

template<typename MessageType1, typename MessageType2>
void logJson(std::string level, std::string path, int line,
	MessageType1 msg1, MessageType2 msg2) {
	Json::Value message;
	message.append(msg1);
	message.append(msg2);
	logJsonRaw(level, path, line, message);
}

template<typename MessageType1, typename MessageType2, typename MessageType3>
void logJson(std::string level, std::string path, int line,
	MessageType1 msg1, MessageType2 msg2, MessageType3 msg3) {
	Json::Value message;
	message.append(msg1);
	message.append(msg2);
	message.append(msg3);
	logJsonRaw(level, path, line, message);
}

template<typename MessageType1, typename MessageType2, typename MessageType3, typename MessageType4>
void logJson(std::string level, std::string path, int line,
	MessageType1 msg1, MessageType2 msg2, MessageType3 msg3, MessageType4 msg4) {
	Json::Value message;
	message.append(msg1);
	message.append(msg2);
	message.append(msg3);
	message.append(msg4);
	logJsonRaw(level, path, line, message);
}

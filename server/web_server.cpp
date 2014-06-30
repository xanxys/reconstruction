#include "web_server.h"

#include <iostream>
#include <fstream>

#include <boost/algorithm/string.hpp>

#include <logging.h>
#include <server/mongoose.h>

namespace server {

Response::Response(Json::Value& value) :
	Response(Json::FastWriter().write(value), "application/json") {
}

Response::Response(std::string data, std::string mime) :
	Response(200, data, mime) {
}

Response::Response(int status_code, std::string data, std::string mime) :
	status_code(status_code), data(data), mime(mime) {
}

Response Response::notFound() {
	return Response(404, "File not found", "text/plain");
}

std::string Response::serialize() {
	std::map<int, std::string> code_names = {
		{200, "OK"},
		{403, "Forbidden"},
		{404, "Not Found"},
	};

	return
		"HTTP/1.1 " + std::to_string(status_code) + " " + code_names[status_code] + "\r\n"
		"Content-Type: " + mime + "\r\n"
		"Content-Length: " + std::to_string(static_cast<int64_t>(data.size())) + "\r\n"
		"\r\n"
		 + data;
}


void WebServer::launch() {
	const char* options[] = {"listening_ports", "8080", NULL};
	ctx = ::mg_start(options, &eventHandler, this);
}

Response WebServer::sendStaticFile(std::string path) {
	std::string mime(::mg_get_builtin_mime_type(path.c_str()));
	return sendStaticFile(path, mime);
}

Response WebServer::sendStaticFile(
	std::string path, std::string mime) {

	const std::string file_path = "http_static/" + path;
	std::ifstream ifs(file_path);

	std::string content(
		(std::istreambuf_iterator<char>(ifs)),
		std::istreambuf_iterator<char>());

	return Response(content, mime);
}


int WebServer::eventHandler(struct ::mg_event* event) {
	// Ignore non-request events.
	if(event->type != MG_REQUEST_BEGIN) {
		return 0;
	}

	// Parse request "/hoge/abc" -> ["hoge", "abc"], "/" -> []
	const std::string uri(event->request_info->uri);

	std::vector<std::string> req_parsed;
	if(uri != "/") {
		boost::split(req_parsed, uri, boost::is_any_of("/"));
		req_parsed.erase(req_parsed.begin());
	}

	// Get method
	const std::string method(event->request_info->request_method);

	// Get data
	std::string data;
	const char* data_size = ::mg_get_header(event->conn, "Content-Length");
	if(data_size != nullptr) {
		const int size = std::stoi(data_size);
		if(0 < size && size < 100 * 1000 * 1000) {
			std::vector<uint8_t> buffer(size);
			mg_read(event->conn, buffer.data(), buffer.size());
			data = std::string(buffer.begin(), buffer.end());
		}
	}

	if(data_size) {
		INFO(method + " " + uri + " : " + std::to_string(std::stoi(data_size)) + "B");
	} else {
		INFO(method + " " + uri);
	}

	try {
		auto response = reinterpret_cast<WebServer*>(event->user_data)
			->handleRequest(req_parsed, method, data);
		const std::string response_str = response.serialize();
		::mg_write(event->conn, response_str.data(), response_str.size());
	} catch(std::exception& exc) {
		const std::string response_str = Response(500, exc.what(), "text/plain").serialize();
		::mg_write(event->conn, response_str.data(), response_str.size());
	} catch(...) {
		const std::string response_str = Response(500, "Unknown exception", "text/plain").serialize();
		::mg_write(event->conn, response_str.data(), response_str.size());
	}

	return 1;
}

}  // namespace

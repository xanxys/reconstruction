#pragma once

#include <string>
#include <vector>

#include <jsoncpp/json/json.h>

struct mg_event;
struct mg_context;

namespace server {

class Response {
public:
	// Success Response
	Response(Json::Value& value);
	Response(std::string data, std::string mime);

	// Fully generic Response
	Response(int status_code, std::string data, std::string mime);

	static Response notFound();

	// Get HTTP Response as string
	std::string serialize();
private:
	const int status_code;
	const std::string mime;
	const std::string data;
};

class WebServer {
public:
	// Start the server asynchronously.
	void launch();

	// TODO: when we add another parameter (e.g. query string),
	// wrap them in Request class.
	virtual Response handleRequest(std::vector<std::string> uri,
		std::string method, std::string data) = 0;
protected:
	Response sendStaticFile(std::string path);
	Response sendStaticFile(std::string path, std::string mime);
private:
	static int eventHandler(struct mg_event* event);
private:
	struct mg_context* ctx;
};

}  // namespace

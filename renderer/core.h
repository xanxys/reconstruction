#pragma once

#include <memory>
#include <string>
#include <vector>

#include <boost/optional.hpp>
#include <eigen3/Eigen/Dense>
#include <GL/glew.h>
#include <glfw3.h>
#include <opencv2/opencv.hpp>

#include "gl.h"

namespace construct {

class Core {
public:
	Core(bool windowed = true);

	cv::Mat render(std::shared_ptr<Geometry> geom);

	// Blocking call to run event loop.
	void run() [[deprecated]];
protected:
	enum DisplayMode {
		WINDOW,
	};

	void enableExtensions();
	void init(DisplayMode mode);

	

	void usePreBuffer();
	void useBackBuffer();
private:
	// avatar things.
	float max_luminance;

	GLuint FramebufferName;

	GLFWwindow* window;

	int screen_width;
	int screen_height;

	int buffer_width;
	int buffer_height;

	// shaders
	std::shared_ptr<Shader> standard_shader;
	std::shared_ptr<Shader> texture_shader;
	std::shared_ptr<Shader> warp_shader;
	std::shared_ptr<Texture> pre_buffer;

	double t_last_update;
};

}  // namespace

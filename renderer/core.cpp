#include "core.h"

#include <array>
#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <random>

#include <json/json.h>

#include "util.h"

namespace construct {


Core::Core(bool windowed) :
	max_luminance(150),
	t_last_update(0) {

	init(DisplayMode::WINDOW);
}

void Core::enableExtensions() {
	GLenum err = glewInit();
	if(GLEW_OK != err) {
		throw glewGetErrorString(err);
	}
}

void Core::usePreBuffer() {
	// Set attachment 0.
	glBindFramebuffer(GL_FRAMEBUFFER, FramebufferName);

	// Use attachment 0.
	std::array<GLenum, 1> buffers = {GL_COLOR_ATTACHMENT0};
	glDrawBuffers(buffers.size(), buffers.data());

	if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
		throw "Failed to set OpenGL frame buffer";
	}
}

void Core::useBackBuffer() {
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glDrawBuffer(GL_BACK);
}

void Core::init(DisplayMode mode) {
	bool use_true_fullscreen = false;

	if(!glfwInit()) {
		throw "Failed to initialize GLFW";
	}

	if(mode == DisplayMode::WINDOW) {
		screen_width = 640;
		screen_height = 480;

		glfwWindowHint(GLFW_VISIBLE, false);
		window = glfwCreateWindow(screen_width, screen_height, "recon-renderer", nullptr, nullptr);
		glfwHideWindow(window);
	}
	buffer_width = screen_width * 2;
	buffer_height  = screen_height * 2;	
	
	if(!window) {
		glfwTerminate();
		throw "Failed to create GLFW window";
	}

	glfwMakeContextCurrent(window);



	enableExtensions();

	// OpenGL things
	FramebufferName = 0;
	glGenFramebuffers(1, &FramebufferName);
	glBindFramebuffer(GL_FRAMEBUFFER, FramebufferName);

	pre_buffer = Texture::create(buffer_width, buffer_height, true);

	// The depth buffer
	GLuint depthrenderbuffer;
	glGenRenderbuffers(1, &depthrenderbuffer);
	glBindRenderbuffer(GL_RENDERBUFFER, depthrenderbuffer);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, buffer_width, buffer_height);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthrenderbuffer);

	// Set "renderedTexture" as our colour attachement #0
	glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, pre_buffer->unsafeGetId(), 0);

	//
	warp_shader = Shader::create("gpu/warp.vs", "gpu/warp.fs");
	standard_shader = Shader::create("gpu/base.vs", "gpu/base.fs");
	texture_shader = Shader::create("gpu/tex.vs", "gpu/tex.fs");
}

cv::Mat Core::render(std::shared_ptr<Geometry> geom) {
	// Erase all
	usePreBuffer();

	glClearColor(0, 0, 0, 1);
	glClearDepth(1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	
	const int width = screen_width;
	const int height = screen_height;

	glViewport(0, 0, width / 2, height);
	Eigen::Matrix4f projection;
	geom->render();


	cv::Mat image(height, width, CV_8UC3);
	glReadPixels(0, 0, width, height, GL_BGR, GL_UNSIGNED_BYTE, image.data);
	return image;
}

void Core::run() {
	/*
	try {
		while(!glfwWindowShouldClose(window)) {
			const double step_t0 = glfwGetTime();
			// step();
			
			render();
			auto error = glGetError();
			if(error != GL_NO_ERROR) {
				std::cout << "error: OpenGL error: " << error << std::endl;
			}

			glfwSwapBuffers(window);
			const double t = glfwGetTime();
			

			glfwPollEvents();
		}
		
	} catch(char* exc) {
		std::cout << "Exception: " << exc << std::endl;
	} catch(std::string exc) {
		std::cout << "Exception: " << exc << std::endl;
	} catch(std::exception& exc) {
		std::cout << "Exception: " << exc.what() << std::endl;
	} catch(...) {
		std::cout << "Unknown exception" << std::endl;
	}
	
	glfwTerminate();
	*/
}

}  // namespace

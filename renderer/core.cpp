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


Core::Core() :
	max_luminance(150),
	t_last_update(0) {

	init();

	/*
	glfwPollEvents();
	glfwPollEvents();
	glfwPollEvents();
	glfwPollEvents();
	*/
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

void Core::init() {
	bool use_true_fullscreen = false;

	if(!glfwInit()) {
		throw "Failed to initialize GLFW";
	}

	screen_width = 640;
	screen_height = 480;

	glfwWindowHint(GLFW_VISIBLE, false);
	window = glfwCreateWindow(screen_width, screen_height, "recon-renderer", nullptr, nullptr);
	glfwHideWindow(window);

	buffer_width = 640;
	buffer_height  = 480;
	
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
	warp_shader = Shader::create("renderer/warp.vs", "renderer/warp.fs");
	standard_shader = Shader::create("renderer/base.vs", "renderer/base.fs");
	texture_shader = Shader::create("renderer/tex.vs", "renderer/tex.fs");
}

cv::Mat Core::render(float fov_h,
	std::shared_ptr<Texture> tex, std::shared_ptr<Geometry> geom) {

	// Erase all
	usePreBuffer();

	glClearColor(0, 1, 0, 1);
	glClearDepth(1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glDisable(GL_DEPTH_TEST);
	
	const int width = screen_width;
	const int height = screen_height;

	const float near = 0.05;
	const float far = 50;
	const float r = std::tan(fov_h / 2);
	const float t = static_cast<float>(height) / static_cast<float>(width) * r;

	glViewport(0, 0, width, height);
	Eigen::Matrix<float, 4, 4, Eigen::RowMajor> projection(Eigen::Matrix4f::Identity());

	projection(0, 0) = near / r;
	projection(1, 1) = near / t;
	projection(2, 2) = - (far + near) / (far - near);
	projection(2, 3) = - 2 * far * near / (far - near);
	projection(3, 2) = -1;

	glCullFace(GL_FRONT);


	Eigen::Matrix<float, 4, 4, Eigen::RowMajor> trans(Eigen::Matrix4f::Identity());
	tex->useIn(0);
	texture_shader->use();
	texture_shader->setUniform("texture", 0);
	texture_shader->setUniform("luminance", 1.0f);
	texture_shader->setUniformMat4("world_to_screen", projection.data());
	texture_shader->setUniformMat4("local_to_world", trans.data());
	geom->render();

	glFinish();


	cv::Mat image(height, width, CV_8UC3);
	glReadPixels(0, 0, width, height, GL_BGR, GL_UNSIGNED_BYTE, image.data);
	return image;
}

}  // namespace

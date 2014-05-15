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
}

void Core::enableExtensions() {
	glewExperimental = GL_TRUE;
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
	// warp_shader = Shader::create("renderer/warp.vs", "renderer/warp.fs");
	standard_shader = Shader::create("renderer/base.vs", "renderer/base.fs");
	texture_shader = Shader::create("renderer/tex.vs", "renderer/tex.fs");
}

cv::Mat Core::render(
	const Camera& camera,
	std::shared_ptr<Texture> tex, std::shared_ptr<Geometry> geom) {

	const int width = camera.getWidth();
	const int height = camera.getHeight();

	assert(camera.getWidth() == screen_width);
	assert(camera.getHeight() == screen_height);

	// Erase all
	usePreBuffer();

	glClearColor(0, 0, 0, 1);
	glClearDepth(1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glEnable(GL_DEPTH_TEST);
	glCullFace(GL_FRONT);
	

	glViewport(0, 0, width, height);

	Eigen::Matrix<float, 4, 4, Eigen::RowMajor> trans(Eigen::Matrix4f::Identity());
	Eigen::Matrix<float, 4, 4, Eigen::RowMajor> proj_view = camera.getMatrix();

	tex->useIn(0);
	texture_shader->use();
	texture_shader->setUniform("tex", 0);
	texture_shader->setUniform("luminance", 1.0f);
	texture_shader->setUniformMat4("world_to_screen", proj_view.data());
	texture_shader->setUniformMat4("local_to_world", trans.data());
	geom->render();

	glFinish();

	cv::Mat image(height, width, CV_8UC3);
	glReadPixels(0, 0, width, height, GL_BGR, GL_UNSIGNED_BYTE, image.data);

	// Flip Y axis because:
	// * OpenGL window coord: down to up
	// * OpenCV image coord: up to down
	cv::flip(image, image, 0);
	return image;
}

}  // namespace

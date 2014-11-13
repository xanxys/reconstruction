#include "renderer.h"

#include <boost/range/irange.hpp>

Scene::Scene(Camera camera) : camera(camera) {
}

Renderer& Renderer::getInstance() {
	static Renderer renderer;
	return renderer;
}

Renderer::Renderer() : comm_input(nullptr) {
	std::thread(&Renderer::rendererWorker, this).detach();
}

cv::Mat Renderer::render(const Scene& scene) {
	// send command
	{
		std::lock_guard<std::mutex> lock(comm_mutex);

		// clear result so this thread don't wake up accidentally
		comm_result = cv::Mat();
		comm_input = &scene;
	}
	comm_signal.notify_one();

	// wait for result
	cv::Mat result;
	{
		std::unique_lock<std::mutex> lock(comm_mutex);
		comm_signal.wait(lock, [this]{
			return comm_result.data != nullptr;
		});

		result = comm_result;
	}
	assert(result.data);
	return result;
}

void Renderer::rendererWorker() {
	construct::Core core;

	while(true) {
		// Wait until input Scene is available.
		std::unique_lock<std::mutex> lock(comm_mutex);
		comm_signal.wait(lock, [this]{
			return comm_input != nullptr;
		});

		assert(comm_input);
		comm_result = renderInternal(core, *comm_input);
		assert(comm_result.data);
		comm_input = nullptr;

		// notify result's availability.
		lock.unlock();
		comm_signal.notify_one();
	}
}

cv::Mat Renderer::renderInternal(construct::Core& core, const Scene& scene) {
	assert(&core);
	assert(&scene);

	std::vector<float> data;
	data.reserve(3 * scene.triangles.size() * 5);

	cv::Mat tex_image;
	for(const auto& tri : scene.triangles) {
		for(int i : boost::irange(0, 3)) {
			const auto& pos = tri.pos[i];
			data.push_back(pos.x());
			data.push_back(pos.y());
			data.push_back(pos.z());

			const auto& uv = tri.uv[i];
			data.push_back(uv.x());
			data.push_back(uv.y());
		}
		tex_image = tri.texture;
	}
	assert(tex_image.data);
	assert(data.size() == 3 * scene.triangles.size() * 5);

	auto tex = construct::Texture::create(tex_image.cols, tex_image.rows, false);


	tex->useIn();
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, tex_image.cols, tex_image.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, tex_image.data);
	
	return core.render(
		scene.camera,
		tex,
		construct::Geometry::createPosUV(
			3 * scene.triangles.size(), data.data()));
}

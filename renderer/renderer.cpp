#include "renderer.h"

#include <boost/range/irange.hpp>

Renderer::Renderer() {
}

Renderer& Renderer::getInstance() {
	static Renderer renderer;
	return renderer;
}

cv::Mat Renderer::render(const Scene& scene) {
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

	auto tex = construct::Texture::create(tex_image.cols, tex_image.rows, false);


	tex->useIn();
	glTexImage2D(GL_TEXTURE_2D, 0, GL_BGR, tex_image.cols, tex_image.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, tex_image.data);
	
	return core.render(
		scene.camera.fov_h,
		tex,
		construct::Geometry::createPosUV(
			3 * scene.triangles.size(), data.data()));
}

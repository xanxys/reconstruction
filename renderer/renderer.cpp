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
	data.resize(3 * scene.triangles.size() * 5);
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
	}
	
	return core.render(construct::Geometry::createPosUV(
		3 * scene.triangles.size(), data.data()));
}

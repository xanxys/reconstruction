#include "texture_conversion.h"

Eigen::Vector2f swapY(const Eigen::Vector2f& v) {
	return Eigen::Vector2f(v(0), 1 - v(1));
}

cv::Point2i eigenToCV(const Eigen::Vector2f& v) {
	return cv::Point2i(v(0), v(1));
}

cv::Mat visualizeUVMap(const TriangleMesh<std::pair<Eigen::Vector3f, Eigen::Vector2f>>& mesh) {
	const int image_size = 2048;
	cv::Mat image(image_size, image_size, CV_8UC3);
	image = cv::Scalar(0, 0, 0);
	const cv::Scalar color(0, 0, 255);
	for(const auto& tri : mesh.triangles) {
		const std::array<Eigen::Vector2f, 3> uvs = {
			mesh.vertices[std::get<0>(tri)].second.second,
			mesh.vertices[std::get<1>(tri)].second.second,
			mesh.vertices[std::get<2>(tri)].second.second
		};

		for(int i : boost::irange(0, 3)) {
			cv::line(image,
				eigenToCV(swapY(uvs[i]) * image_size),
				eigenToCV(swapY(uvs[(i + 1) % 3]) * image_size),
				color);
		}
	}
	return image;
}

// color: [0-1]^3, RGB
// output, uint8*3 image (in BGR, which is opencv standard)
cv::Mat bake3DTexture(
	const TriangleMesh<Eigen::Vector2f>& mesh,
	std::function<Eigen::Vector3f(Eigen::Vector3f)> colorField
	) {
	const int texture_size = 512;
	cv::Mat texture(texture_size, texture_size, CV_8UC3);
	texture = cv::Scalar(0, 0, 0);
	for(const auto& triangle : mesh.triangles) {
		const auto& v0 = mesh.vertices[std::get<0>(triangle)];
		const auto& v1 = mesh.vertices[std::get<1>(triangle)];
		const auto& v2 = mesh.vertices[std::get<2>(triangle)];

		const Eigen::Vector2f p_tex0 = swapY(v0.second) * texture_size;
		const Eigen::Vector2f p_tex1 = swapY(v1.second) * texture_size;
		const Eigen::Vector2f p_tex2 = swapY(v2.second) * texture_size;

		// Create integer AABB [pmin, pmax) that contains the triangle completely.
		// Note that pixel center for pixel (x,y) is (x+0.5, y+0.5).
		const Eigen::Vector2i pmin = p_tex0.cwiseMin(p_tex1).cwiseMin(p_tex2).cast<int>();
		const Eigen::Vector2i pmax = p_tex0.cwiseMax(p_tex1).cwiseMax(p_tex2).cast<int>() + Eigen::Vector2i(2, 2);

		Eigen::Matrix2f to_barycentric;
		to_barycentric.col(0) = p_tex1 - p_tex0;
		to_barycentric.col(1) = p_tex2 - p_tex0;
		to_barycentric = to_barycentric.inverse().eval();
		for(int y : boost::irange(pmin(1), pmax(1))) {
			for(int x : boost::irange(pmin(0), pmax(0))) {
				const Eigen::Vector2f center_tex(x + 0.5, y + 0.5);
				const Eigen::Vector2f barycentric = to_barycentric * (center_tex - p_tex0);

				// Do nothing if current pixel lies outside of the triangle.
				if(barycentric.minCoeff() < 0 || barycentric(0) + barycentric(1) > 1) {
					continue;
				}

				const Eigen::Vector3f v = v0.first + barycentric(0) * (v1.first - v0.first) + barycentric(1) * (v2.first - v0.first);
				const Eigen::Vector3i color_int = (colorField(v) * 255).cast<int>();
				texture.at<cv::Vec3b>(y, x) = cv::Vec3b(color_int(2), color_int(1), color_int(0));
			}
		}
	}
	return texture;
}


TriangleMesh<Eigen::Vector2f> dropNormal(const TriangleMesh<std::pair<Eigen::Vector3f, Eigen::Vector2f>>& mesh) {
	TriangleMesh<Eigen::Vector2f> mesh_uv;
	mesh_uv.triangles = mesh.triangles;
	mesh_uv.vertices.resize(mesh.vertices.size());
	std::transform(mesh.vertices.begin(), mesh.vertices.end(), mesh_uv.vertices.begin(),
		[](const std::pair<Eigen::Vector3f, std::pair<Eigen::Vector3f, Eigen::Vector2f>>& vertex) {
			return std::make_pair(vertex.first, vertex.second.second);
		});
	return mesh_uv;
}

void writeObjMaterial(std::ostream& output) {
	output << "newmtl obj_uv" << std::endl;
	output << "Ka 1.0 1.0 1.0" << std::endl;
	output << "Kd 1.0 1.0 1.0" << std::endl;
	output << "Ks 0.0 0.0 0.0" << std::endl;
	output << "map_Kd uv.png" << std::endl;
}

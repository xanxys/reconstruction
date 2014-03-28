#include "camera.h"

Camera::Camera(int width, int height,
	float fov_h, Eigen::Transform<float, 3, Eigen::Affine> local_to_world) :
	width(width), height(height), fov_h(fov_h), local_to_world(local_to_world) {
}

Eigen::Matrix4f Camera::getMatrix() const {
	return getProjectionMatrix() * local_to_world.inverse().matrix();
}

Eigen::Matrix4f Camera::getProjectionMatrix() const {
	const float near = 0.05;
	const float far = 25;
	const float r = near * std::tan(fov_h / 2);
	const float t = static_cast<float>(height) / static_cast<float>(width) * r;

	Eigen::Matrix4f projection = Eigen::Matrix4f::Zero();
	projection(0, 0) = near / r;
	projection(1, 1) = -near / t;
	projection(2, 2) = (far + near) / (far - near);
	projection(2, 3) = - 2 * far * near / (far - near);
	projection(3, 2) = 1;
	return projection;
}

int Camera::getWidth() const {
	return width;
}

int Camera::getHeight() const {
	return height;
}

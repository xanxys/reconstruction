#pragma once

#include <array>
#include <memory>
#include <vector>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <opencv2/opencv.hpp>

#include "core.h"

class Triangle {
public:
	std::array<Eigen::Vector3f, 3> pos;
	std::array<Eigen::Vector2f, 3> uv;
	cv::Mat texture;
};


class Camera {
public:
	Eigen::Transform<float, 3, Eigen::Affine> local_to_world;
	float fov_h;
	
	int width;
	int height;
};


class Scene {
public:
	Camera camera;
	std::vector<Triangle> triangles;
};


class Renderer {
public:
	// OpenGL is initialized first time this method is caled.
	static Renderer& getInstance();

	cv::Mat render(const Scene& scene);
private:
	Renderer();
private:
	construct::Core core;
};

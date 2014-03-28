#pragma once

#include <array>
#include <condition_variable>
#include <thread>
#include <memory>
#include <mutex>
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

// Internally, this uses singleton to lazily create a single,
// OpenGL worker thread.
// Calls are further serialized by using bi-directional signaling
// with the worker.
class Renderer {
public:
	// OpenGL is initialized first time this method is caled.
	static Renderer& getInstance();

	cv::Mat render(const Scene& scene);
private:
	Renderer();

	// Worker thread.
	void rendererWorker();
	// Actual rendering method that uses OpenGL.
	// ONLY CALL FROM THE THREAD THAT OWNS CORE.
	static cv::Mat renderInternal(construct::Core& core, const Scene& scene);
private:
	// caller - worker communication
	// availability of input / output is determined as follows:
	//
	// input: comm_input is non-null
	// output: comm_result.data is non-null
	//
	// comm_signal can wake up threads even when no one notify'd.
	// (spurious wakeup). So we need to check these conditions
	// before proceeding to rendering / result fetch.
	std::mutex comm_mutex;
	std::condition_variable comm_signal;
	const Scene* comm_input;
	cv::Mat comm_result;
};

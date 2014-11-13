#pragma once

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

// Camera coordinates:
// x+: right
// y+: down
// z+: forward
class Camera {
public:
	Camera(int width, int height,
		float fov, Eigen::Transform<float, 3, Eigen::Affine> local_to_world);

	// Get the matrix that transforms world coordinates to
	// normalized device coodinates ([-1,1]^3).
	// OpenGL NDC:
	// x+: right
	// y+: up
	// z+: forward
	Eigen::Matrix4f getMatrix() const;
	Eigen::Matrix4f getProjectionMatrix() const;

	int getWidth() const;
	int getHeight() const;

private:
	const Eigen::Transform<float, 3, Eigen::Affine> local_to_world;
	const float fov_h;
	
	const int width;
	const int height;
};

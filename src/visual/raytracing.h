// General purpose raytracing module w/ path-tracing.
// Use SI unit.
#pragma once

#include <memory>
#include <random>

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

namespace recon {

// Linear sRGB color.
// It can be reflectance, radiance, irradiance, or anything.
typedef Eigen::Vector3f Color;

// Sample a vector from surface of sphere
Eigen::Vector3f sample_hemisphere(std::mt19937& randomness, const Eigen::Vector3f& n);

class Ray {
public:
	// Construct a Ray. dir must be normalized.
	Ray(Eigen::Vector3f org, Eigen::Vector3f dir);

	// Return point on the ray.
	Eigen::Vector3f at(float t) const;

	// Shift origin by given distance. Useful for adjusting by epsilon.
	Ray shiftForward(float dt) const;
public:
	// don't feel like creating setter/getter for these, so making them public.
	const Eigen::Vector3f org;
	const Eigen::Vector3f dir;
};

// Lambert + self emission
class Material {
public:
	Material();
	Material(Color _reflectance, Color _emission);

	// DEPRECATED: only works for Lambert surface.
	Color brdf();

	// in and out is both pointing "outward".
	Color brdf(Eigen::Vector3f normal, Eigen::Vector3f in, Eigen::Vector3f out);

	Color getEmission();
private:
	Color emission;
	Color reflectance;
};

// Visible object with uniform material.
class Object {
public:
	Object(Material _material);
	virtual bool intersect(const Ray& ray, float& t);
	virtual bool intersect(const Ray& ray, float& t, Eigen::Vector3f& normal);
public:
	Material material;
};

class Sphere : public Object {
public:
	Sphere(Eigen::Vector3f _center, float _radius, Material _material);

	bool intersect(const Ray& ray, float& t, Eigen::Vector3f& normal);
private:
	Eigen::Vector3f center;
	float radius;
};

// A plane described by {p | dot(p, normal) = dist}.
class Plane : public Object {
public:
	Plane(Eigen::Vector3f _normal, float _dist, Material _material);
	bool intersect(const Ray& ray, float& t);
	bool intersect(const Ray& ray, float& t, Eigen::Vector3f& _normal);
private:
	Eigen::Vector3f normal;
	float dist;
};

// Both-sided triangle.
class Triangle : public Object {
public:
	Triangle(Eigen::Vector3f _v0 , Eigen::Vector3f _v1, Eigen::Vector3f _v2 , Material _material);
	bool intersect(const Ray& ray, float& t, Eigen::Vector3f& normal) override;
	bool intersect(const Ray& ray, float& t, Eigen::Vector3f& normal, Eigen::Vector2f& bary);
private:
	Eigen::Vector3f v0,v1,v2;
	Eigen::Vector3f d1,d2;
	Eigen::Vector3f _normal;
};

// Base class for all (accelerated) lights.
class Light {
public:
	// Approximate light by (multiple) point lights (pos and emission).
	// emission is radiant intensity at the direction.
	virtual std::vector<std::pair<Eigen::Vector3f, Color>> emittingTo(Eigen::Vector3f to);
};

class Scene {
public:
	Scene(
		std::vector<std::shared_ptr<Object>> objects,
		std::vector<std::shared_ptr<Light>> lights);

	// Returns radiance.
	Color trace(Ray ray, int depth = 0);

	bool intersect(const Ray& ray, float& t, Eigen::Vector3f& normal, Material& material) const;
	bool intersect(const Ray& ray, float& t) const;

private:
	// shadow rayを飛ばして可視判定
	bool isVisible(Eigen::Vector3f from, Eigen::Vector3f to) const;
private:
	std::vector<std::shared_ptr<Object>> objects;
	std::vector<std::shared_ptr<Light>> lights;  // "fast" light (compared to emissive materials)
	std::mt19937 randomness;
};


/*
// Image coordinates directly corresponds to camera coordinates.
// * X+<Camera>: right
// * Y+<Camera>: down
// * Z+<Camera>: forward
class Camera {
public:
	// latest interface
	Camera(Eigen::Vector3f origin, cv::Mat camera_to_world, const CameraParameters& params);

	// Convenience method for renderHDR + tonemap
	cv::Mat render(Scene& scene);

	// Convert sRGB radiance image to LDR, [0,255], gamma-corrected image.
	// Also flip RGB -> BGR.
	cv::Mat_<cv::Vec3b> tonemap(cv::Mat_<cv::Vec3f> mat);

	cv::Mat_<cv::Vec3f> untonemap(cv::Mat_<cv::Vec3b> ldr);

	// Render linear sRGB image.
	cv::Mat_<cv::Vec3f> renderHDR(Scene& scene);
private:
	void initFOV(const CameraParameters& params);
private:
	// extrisinc (pose)
	const Eigen::Vector3f origin;
	cv::Mat camera_to_world;

	// intrinsic (size, fov)
	const int width, height;
	float fov_x, fov_y;  // TODO: should be made const, really

	// sampler
	std::mt19937 randomness;
	const int samples_per_pixel;
};
*/

}  // namespace

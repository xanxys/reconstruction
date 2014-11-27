#include "raytracing.h"

#include <fstream>
#include <iostream>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

namespace recon {

using std::shared_ptr;
using std::vector;

// Sample a vector from surface of sphere
Eigen::Vector3f sample_hemisphere(std::mt19937& randomness, const Eigen::Vector3f& n) {
	while(true){
		Eigen::Vector3f v = Eigen::Vector3f(
			std::uniform_real_distribution<float>(-1, 1)(randomness),
			std::uniform_real_distribution<float>(-1, 1)(randomness),
			std::uniform_real_distribution<float>(-1, 1)(randomness));

		const float length = v.norm();
		if(length <= 1) {
			if(v.dot(n) > 0) {
				v /= length;
			} else {
				v /= -length;
			}
			return v;
		}
	}
}

Ray::Ray(Eigen::Vector3f org, Eigen::Vector3f dir) : org(org), dir(dir) {
}

Eigen::Vector3f Ray::at(float t) const {
	return org + dir * t;
}

Ray Ray::shiftForward(float dt) const {
	return Ray(org + dir * dt, dir);
}


Material::Material() { //reflectance 0 or 1
}

Material::Material(Color _reflectance, Color _emission) : reflectance(_reflectance), emission(_emission){
}

Color Material::brdf() {
	return reflectance / M_PI;
}

Color Material::brdf(Eigen::Vector3f normal, Eigen::Vector3f in, Eigen::Vector3f out) {
	return reflectance / M_PI;
}

Color Material::getEmission() {
	return emission;
}


Object::Object(Material _material) : material(_material){
}

bool Object::intersect(const Ray& ray, float& t){
	return false;
}

bool Object::intersect(const Ray& ray, float& t, Eigen::Vector3f& normal){
	return false;
}


Sphere::Sphere(Eigen::Vector3f _center, float _radius, Material _material) : center(_center), radius(_radius), Object(_material){
}

bool Sphere::intersect(const Ray& ray, float& t, Eigen::Vector3f& normal){
	bool collided = false;

	const float a = ray.dir.norm();  // TODO: what's this??
	const float b = 2 * ray.dir.dot(ray.org - center);
	const float c = std::pow((ray.org - center).norm(), 2) - std::pow(radius, 2);

	const float det = std::pow(b, 2) - 4*a*c;
	if(det>=0){
		const float t0 = (-b - std::sqrt(det))/(2*a);
		const float t1 = (-b + std::sqrt(det))/(2*a);

		if(t0>0){
			t = t0;
			collided = true;
		}
		else if(t1>0){
			t = t1;
			collided = true;
		}

		if(collided){
			Ray r = ray;
			normal = (r.at(t) - center).normalized();
		}
	}

	return collided;
}


Plane::Plane(Eigen::Vector3f _normal, float _dist, Material _material) : normal(_normal), dist(_dist), Object(_material){
}

bool Plane::intersect(const Ray& ray, float& t){
	const float div = normal.dot(ray.dir);
	t = dist-normal.dot(ray.org);
	if((div<0 && t>=0) || (div>0 && t<=0) || (div==0)) return false;
	t /= div;
	return true;
}

bool Plane::intersect(const Ray& ray, float& t, Eigen::Vector3f& _normal){
	const float div = normal.dot(ray.dir);

	t = dist-normal.dot(ray.org);

	if(div<0) {
		if(t>=0) return false;
		_normal = normal; // front
	} else {
		if(t<=0) return false;
		_normal = -normal; // back
	}
	if(div==0)
		return false;
	t /= div;
	return true;
}


Triangle::Triangle(Eigen::Vector3f _v0 , Eigen::Vector3f _v1, Eigen::Vector3f _v2 , Material _material) : v0(_v0),v1(_v1),v2(_v2),Object(_material){
	d1 = v1 - v0;
	d2 = v2 - v0;
	_normal = (d1.cross(d2)).normalized(); // TODO: swap _normal & normal
}

bool Triangle::intersect(const Ray& ray, float& t, Eigen::Vector3f& normal) {
	//bool collided = false;
	Eigen::Vector3f s1 = ray.dir.cross(d2);//dirとd2の外積　opencvより
	float divisor = s1.dot(d1);//s1とd1の内積
	if(divisor ==0.){
		return false;
	}
	float invDivisor = 1.0/divisor;

	Eigen::Vector3f s  = ray.org - v0;
	float a = s.dot(s1) * invDivisor;
	if(a<0 || a>1)// if(!(0<=a && a<=1))
		return false;

	Eigen::Vector3f s2 = s.cross(d1);
	float b = ray.dir.dot(s2) * invDivisor;

	if(b<0 || a+b>1)
		return false;

	t = d2.dot(s2) * invDivisor;
	if(t < 0)
		return false;

	if(divisor < 0)
		normal = _normal; // front
	else
		normal = -_normal; // back

	return true;
}

bool Triangle::intersect(const Ray& ray, float& t, Eigen::Vector3f& normal, Eigen::Vector2f& bary) {
	Eigen::Vector3f s1 = ray.dir.cross(d2);
	float divisor = s1.dot(d1);
	if(divisor ==0.){
		return false;
	}
	float invDivisor = 1.0/divisor;

	Eigen::Vector3f s  = ray.org - v0;
	float a = s.dot(s1) * invDivisor;
	if(a<0 || a>1)// if(!(0<=a && a<=1))
		return false;

	Eigen::Vector3f s2 = s.cross(d1);
	float b = ray.dir.dot(s2) * invDivisor;

	if(b<0 || a+b>1)
		return false;

	t = d2.dot(s2) * invDivisor;
	if(t < 0)
		return false;

	if(divisor < 0)
		normal = _normal; // front
	else
		normal = -_normal; // back

	bary = Eigen::Vector2f(a, b);
	return true;
}


std::vector<std::pair<Eigen::Vector3f, Color>> Light::emittingTo(Eigen::Vector3f to) {
	return std::vector<std::pair<Eigen::Vector3f, Color>>();
}


Scene::Scene(vector<shared_ptr<Object>> objects, vector<shared_ptr<Light>> lights) :
	objects(objects), lights(lights) {
}

Color Scene::trace(Ray ray, int depth) {
	// Recursion cutoff; increase this value for glossy scenes.
	if(depth > 2) {
		return Eigen::Vector3f(0, 0, 0);
	}

	// Random-sample from rendering equation.
	Eigen::Vector3f normal;
	float t;
	Material material;
	bool collided = intersect(ray, t, normal, material);

	// Hit background
	if(!collided) {
		return Eigen::Vector3f(0, 0, 0);
	}

	// Hit something: collect emission, direct light, and one sample of random direction.
	Ray new_ray(ray.at(t), sample_hemisphere(randomness, normal));
	new_ray.shiftForward(1e-6);  // avoid self-collision

	Eigen::Vector3f light_radiance;
	for(auto it = lights.begin(); it != lights.end(); it++) {
		auto micro_lights = (*it)->emittingTo(new_ray.org);

		for(auto it_micro = micro_lights.begin(); it_micro != micro_lights.end(); it_micro++) {
			const Eigen::Vector3f ml_pos = it_micro->first;

			if(!isVisible(new_ray.org, ml_pos)) {
				continue;
			}

			const Eigen::Vector3f delta_ml = ml_pos - new_ray.org;
			const Color ml_radiance = it_micro->second / std::pow((delta_ml).norm(), 2);

			light_radiance += material.brdf().cwiseProduct(ml_radiance * normal.dot(delta_ml.normalized()));
		}
	}

	// 2 * pi comes from area(measure) of the domain (hemisphere)
	const Eigen::Vector3f other_radiance = 2 * M_PI * (material.brdf().cwiseProduct(trace(new_ray, depth + 1) * normal.dot(new_ray.dir)));
	return material.getEmission() + other_radiance + light_radiance;
}

bool Scene::intersect(const Ray& ray, float& t) const {
	bool collided = false;

	for(auto it = objects.begin(); it!=objects.end(); it++){
		float t_curr;
		bool collided_curr;

		collided_curr = (*it)->intersect(ray, t_curr);
		if(collided_curr && (!collided || (t_curr<t))){
			t = t_curr;
			collided = collided_curr;
		}
	}

	return collided;
}

bool Scene::intersect(const Ray& ray, float& t, Eigen::Vector3f& normal, Material& material) const {
	bool collided = false;

	for(auto it = objects.begin(); it!=objects.end(); it++){
		float t_curr;
		Eigen::Vector3f normal_curr;
		bool collided_curr;

		collided_curr = (*it)->intersect(ray, t_curr, normal_curr);
		if(collided_curr && (!collided || (t_curr<t))){
			t = t_curr;
			normal = normal_curr;
			collided = collided_curr;

			material = (*it)->material;
		}
	}

	return collided;
}

bool Scene::isVisible(Eigen::Vector3f from, Eigen::Vector3f to) const {
	float t;
	const Eigen::Vector3f tf = to - from;
	if(intersect(Ray(from, tf.normalized()), t))
		return (tf.norm() < t);
	else
		return true;
}

/*
Camera::Camera(Eigen::Vector3f origin, Mat camera_to_world, const CameraParameters& params) :
	origin(origin), camera_to_world(camera_to_world),
	width(params.getWidth()), height(params.getHeight()),
	samples_per_pixel(256) {

	initFOV(params);
}

void Camera::initFOV(const CameraParameters& params) {
	fov_x = params.getXFoV();
	fov_y = params.getYFoV();
}

cv::Mat Camera::render(Scene& scene) {
	return tonemap(renderHDR(scene));
}

cv::Mat_<cv::Vec3f> Camera::renderHDR(Scene& scene) {
	Mat_<cv::Vec3f> img = Mat::zeros(height, width, CV_32FC3);

	#pragma omp parallel for
	for(int iy = 0; iy < height; iy++) {
		for(int ix = 0; ix < width; ix++) {
			Eigen::Vector3f accum;
			for(int n=0; n < samples_per_pixel; n++){
				const float x = ix + std::uniform_real_distribution<float>(0, 1)(randomness);
				const float y = iy + std::uniform_real_distribution<float>(0, 1)(randomness);

				// Construct perspective ray in world coordinates.
				Eigen::Vector3f dir_camera = normalize(Eigen::Vector3f(
					(x / width - 0.5) * 2 * (std::tan(fov_x / 2)),
					(y / height - 0.5) * 2 * (std::tan(fov_y / 2)),
					1));
				Eigen::Vector3f dir_world = static_cast<cv::Mat>(camera_to_world * static_cast<cv::Mat>(dir_camera));
				Ray ray(origin, dir_world);

				accum += scene.trace(ray);
			}
			img.at<Vec3f>(iy, ix) = accum / samples_per_pixel;
		}
	}
	return img;
}

cv::Mat_<cv::Vec3b> Camera::tonemap(cv::Mat_<cv::Vec3f> hdr) {
	cv::Mat_<cv::Vec3b> ldr_rgb;
	hdr.convertTo(ldr_rgb, CV_8UC3);

	cv::Mat_<cv::Vec3b> ldr_bgr;
	cv::cvtColor(ldr_rgb, ldr_bgr, CV_BGR2RGB);  // actually RGB2BGR

	return ldr_bgr;
}

cv::Mat_<cv::Vec3f> Camera::untonemap(cv::Mat_<cv::Vec3b> ldr) {
	cv::Mat_<cv::Vec3b> ldr_rgb;
	cv::cvtColor(ldr, ldr_rgb, CV_BGR2RGB);

	cv::Mat_<cv::Vec3f> hdr;
	ldr_rgb.convertTo(hdr, CV_32FC3);

	return hdr;
}
*/

}  // namespace

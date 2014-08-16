#pragma once

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

namespace visual {

// An RGB film that reconstruct image from samples.
class FilmRGB8U {
public:
	// Initialize with given size, and set filter to gaussian(sigma).
	FilmRGB8U(int width, int height, float sigma);

	// Put a sample.
	void record(Eigen::Vector2f pos, cv::Vec3b value);

	// return CV_8UC3 interpolated image.
	cv::Mat extract() const;
private:
	const int width;
	const int height;
	const float sigma;
	int kernel_margin;

	cv::Mat_<cv::Vec3f> accum;
	cv::Mat_<float> weight;
};

}

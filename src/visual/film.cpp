#include "film.h"

#include <cmath>

#include <boost/range/irange.hpp>

#include <logging.h>
#include <math_util.h>
#include <range2.h>

namespace recon {

FilmRGB8U::FilmRGB8U(int width, int height, float sigma) :
	width(width), height(height), sigma(sigma) {
	assert(width > 0 && height > 0);
	assert(sigma > 0);
	INFO("Creating film of size", width, height);

	kernel_margin = static_cast<int>((5 * sigma) / 2);
	INFO("Calculated kernel margin=", kernel_margin);

	const int kernel_size = 2 * kernel_margin + 1;
	accum = cv::Mat(height + kernel_size, width + kernel_size, CV_32FC3);
	weight = cv::Mat(height + kernel_size, width + kernel_size, CV_32F);
	for(int y : boost::irange(0, (int)accum.rows)) {
		for(int x : boost::irange(0, (int)accum.cols)) {
			accum(y, x) = cv::Vec3f(0, 0, 0);
			weight(y, x) = 1e-3;  // avoid 0-division by having slight bias
		}
	}
}

void FilmRGB8U::record(Eigen::Vector2f pos, cv::Vec3f value) {
	const Eigen::Vector2i ipos = pos.cast<int>();
	const Eigen::Vector2i imin =
		ipos - Eigen::Vector2i(kernel_margin, kernel_margin);
	const Eigen::Vector2i imax =
		ipos + Eigen::Vector2i(kernel_margin + 2, kernel_margin + 2);

	const Eigen::Vector2i bmin(0, 0);
	const Eigen::Vector2i bmax(width, height);

	for(const auto i : range2(imin.cwiseMax(bmin), imax.cwiseMin(bmax))) {
		const float rsq = std::pow((i.cast<float>() - pos).norm(), 2);
		const float w = (1 / std::sqrt(2 * pi) / sigma) * std::exp(-rsq / (2 * sigma * sigma));
		accum(i(1), i(0)) += value * w;
		weight(i(1), i(0)) += w;
	}
}

cv::Mat FilmRGB8U::extract() const {
	cv::Mat result(height, width, CV_8UC3);
	for(int y : boost::irange(0, height)) {
		for(int x : boost::irange(0, width)) {
			const cv::Vec3f color =
				accum(y + kernel_margin, x + kernel_margin) /
				weight(y + kernel_margin, x + kernel_margin);
			result.at<cv::Vec3b>(y, x) = color;
		}
	}
	return result;
}

}

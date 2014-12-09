#include "field.h"

#include <boost/range/irange.hpp>

#include <logging.h>

namespace recon {

// TODO: this kind of thing should be done by matplotlib(python).
// Also, DO NOT DUMP VISUALIZATION WITHOUT RAW DATA!!!
// You'll NEED RAW DATA for last-minute correction of diagrams
// in paper / thesis.
//
// Assign color to 2-d scalar field.
// red:small green:mid blue:large
// NaN -> whole visualization will be undefined
cv::Mat visualize_field2(const cv::Mat& field) {
	assert(field.type() == CV_32F);
	double v_min, v_max;
	cv::minMaxLoc(field, &v_min, &v_max);
	INFO("range", v_min, v_max);

	auto t_to_color = [](float t) {
		// BGR channels.
		if(t < 0.5) {
			// R -> G
			const float t_sub = t / 0.5;
			return cv::Vec3b(
				0,
				static_cast<int>(t_sub * 255),
				255 - static_cast<int>(t_sub * 255));
		} else {
			// G -> B
			const float t_sub = (t - 0.5) / (1 - 0.5);
			return cv::Vec3b(
				static_cast<int>(t_sub * 255),
				255 - static_cast<int>(t_sub * 255),
				0);
		}
	};

	cv::Mat vis(field.rows, field.cols, CV_8UC3);
	for(const int i : boost::irange(0, field.rows)) {
		for(const int j : boost::irange(0, field.cols)) {
			const float v = field.at<float>(i, j);
			const float t = (v - v_min) / (v_max - v_min);
			vis.at<cv::Vec3b>(i, j) = t_to_color(t);
		}
	}
	return vis;
}

}  // namespace

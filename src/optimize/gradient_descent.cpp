#include "gradient_descent.h"

#include <boost/range/irange.hpp>

namespace recon {

std::pair<Eigen::VectorXf, float> minimize_gradient_descent(
		std::function<
			std::pair<float, Eigen::VectorXf>(const Eigen::VectorXf&)> target,
		const Eigen::VectorXf& initial_param,
		const int max_iter,
		const float epsilon) {
	assert(max_iter > 0);
	assert(epsilon > 0);
	Eigen::VectorXf param = initial_param;
	float current_value;
	for(const int step : boost::irange(0, max_iter)) {
		auto result = target(param);
		current_value = result.first;
		param -= result.second * epsilon;
	}
	return std::make_pair(param, target(param).first);
}

}  // namespace

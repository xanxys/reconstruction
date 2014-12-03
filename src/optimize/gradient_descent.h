// This file implements Gradient descent optimization algorithm.
// target: R^n -> R
//
// pros
// * relatively fast and simple
// cons
// * requires jacobian
#pragma once

#include <functional>

#include <Eigen/Dense>

namespace recon {

// Calculate param such that target(param) is (local) minimum.
// target must return (value, dvalue/dparam)
std::pair<Eigen::VectorXf, float> minimize_gradient_descent(
	std::function<
		std::pair<float, Eigen::VectorXf>(const Eigen::VectorXf&)> target,
	const Eigen::VectorXf& initial_param,
	int max_iter,
	float epsilon=0.1);

}  // namespace

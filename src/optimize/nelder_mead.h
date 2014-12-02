// This file implements Nelder-Mead (simplex) optimization algorithm.
// target: R^n -> R
//
// pros
// * no need for jacobian, hessian
// cons
// * slower than Newton, conjugate-gradient etc.
#pragma once

#include <functional>

#include <Eigen/Dense>

namespace recon {

// Calculate param such that target(param) is (local) minimum.
// size: initial size of simplex
std::pair<Eigen::VectorXf, float> minimize_nelder_mead(
	std::function<float(const Eigen::VectorXf&)> target,
	const Eigen::VectorXf& initial_param,
	int max_iter);

}  // namespace

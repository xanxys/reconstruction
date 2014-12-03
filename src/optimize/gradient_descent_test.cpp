#include "gradient_descent.h"

#include <cmath>

#include <boost/range/irange.hpp>
#include <gtest/gtest.h>

TEST(GradientDescent, Quadratic1Dim) {
	// f(x) = (x-1)^2
	auto fn = [](const Eigen::VectorXf& v) {
		EXPECT_EQ(1, v.size());
		Eigen::VectorXf grad(1);
		grad << 2 * (v(0) - 1);
		return
			std::make_pair(
				std::pow(v(0) - 1, 2),
				grad);
	};

	// Start from 0.
	Eigen::VectorXf x(1);
	x(0) = 0;
	const auto result = recon::minimize_gradient_descent(fn, x, 100);

	// Result position should be near convergence.
	Eigen::VectorXf minimum(1);
	minimum(0) = 1;

	EXPECT_EQ(minimum.size(), result.first.size());
	EXPECT_GT(1e-3, (result.first - minimum).norm());
	EXPECT_FLOAT_EQ(fn(result.first).first, result.second);
}

TEST(GradientDescent, Quadratic100Dim) {
	Eigen::VectorXf minimum(100);
	for(const int i : boost::irange(0, 100)) {
		minimum(i) = i * 0.01;
	}

	// f(x, y) = (x0-0)^2 + (x1-1)^2 + ...
	auto fn = [&minimum](const Eigen::VectorXf& v) {
		EXPECT_EQ(100, v.size());
		Eigen::VectorXf grad = 2 * (v - minimum);
		return std::make_pair(
			std::pow((v - minimum).norm(), 2),
			grad);
	};

	// Start from (0, 0).
	Eigen::VectorXf x(100);
	x.setConstant(0);
	const auto result = recon::minimize_gradient_descent(fn, x, 100);

	// Result position should be near convergence.
	EXPECT_EQ(minimum.size(), result.first.size());
	EXPECT_GT(1e-2, (result.first - minimum).norm());
	EXPECT_FLOAT_EQ(fn(result.first).first, result.second);
}

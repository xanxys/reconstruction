#include "nelder_mead.h"

#include <cmath>

#include <boost/range/irange.hpp>
#include <gtest/gtest.h>

TEST(NelderMeadTest, Quadratic1Dim) {
	// f(x) = (x-1)^2
	auto fn = [](const Eigen::VectorXf& v) {
		EXPECT_EQ(1, v.size());
		return std::pow(v(0) - 1, 2);
	};

	// Start from 0.
	Eigen::VectorXf x(1);
	x(0) = 0;
	const auto result = recon::minimize_nelder_mead(fn, x, 20);

	// Result position should be near convergence.
	Eigen::VectorXf minimum(1);
	minimum(0) = 1;

	EXPECT_EQ(minimum.size(), result.first.size());
	EXPECT_GT(1e-3, (result.first - minimum).norm());
	EXPECT_FLOAT_EQ(fn(result.first), result.second);
}

TEST(NelderMeadTest, Quadratic100Dim) {
	Eigen::VectorXf minimum(100);
	for(const int i : boost::irange(0, 100)) {
		minimum(i) = i * 0.01;
	}

	// f(x, y) = (x0-0)^2 + (x1-1)^2 + ...
	auto fn = [&minimum](const Eigen::VectorXf& v) {
		EXPECT_EQ(100, v.size());
		return std::pow((v - minimum).norm(), 2);
	};

	// Start from (0, 0).
	Eigen::VectorXf x(100);
	x.setConstant(0);
	const auto result = recon::minimize_nelder_mead(fn, x, 20000);

	// Result position should be near convergence.
	EXPECT_EQ(minimum.size(), result.first.size());
	EXPECT_GT(1e-2, (result.first - minimum).norm());
	EXPECT_FLOAT_EQ(fn(result.first), result.second);
}

TEST(NelderMeadTest, Rosenbrock) {
	// Rosenbrock is harder to converge, so give it more iterations
	// and relaxed check.
	const float a = 1;
	const float b = 100;
	auto rosenbrock = [a, b](const Eigen::VectorXf& v) {
		EXPECT_EQ(2, v.size());
		return std::pow(a - v(0), 2) + b * std::pow(v(1) - v(0) * v(0), 2);
	};

	// Start from (0, 0).
	Eigen::VectorXf x(2);
	x << 0, 0;
	const auto result = recon::minimize_nelder_mead(rosenbrock, x, 200);

	// Result position should be near convergence.
	Eigen::VectorXf minimum(2);
	minimum << a, a * a;

	EXPECT_EQ(minimum.size(), result.first.size());
	EXPECT_GT(1e-2, (result.first - minimum).norm());
	EXPECT_FLOAT_EQ(rosenbrock(result.first), result.second);
}
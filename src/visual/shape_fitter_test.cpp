#include "shape_fitter.h"

#include <vector>

#include <boost/range/irange.hpp>
#include <Eigen/Dense>
#include <gtest/gtest.h>

TEST(triangulatePolygonTest, TrivialCase) {
	std::vector<Eigen::Vector2f> points = {
		{0, 0}, {1, 0}, {0, 1}};

	const auto tris = visual::shape_fitter::triangulatePolygon(points);
	EXPECT_EQ(1, tris.size());

	// Circular shift of {0, 1, 2}
	const auto tri = tris[0];
	EXPECT_EQ((tri[0] + 1) % 3, tri[1]);
	EXPECT_EQ((tri[0] + 2) % 3, tri[2]);
}

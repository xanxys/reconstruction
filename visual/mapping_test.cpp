#pragma once
#include "mapping.h"

#include <vector>

#include <boost/range/irange.hpp>
#include <Eigen/Dense>
#include <gtest/gtest.h>

std::vector<Eigen::Vector2f> arbRectangles() {
	return std::vector<Eigen::Vector2f>({
		{1, 2},
		{3, 4},
		{3, 1},
		{2, 2}
	});
}


TEST(packRectanglesTest, ResultFitsInsideSquare) {
	const auto rects = arbRectangles();
	const auto result = visual::packRectangles(rects);
	const auto size = result.first;
	const auto offsets = result.second;

	EXPECT_EQ(rects.size(), offsets.size());
	for(int i : boost::irange(0, (int)rects.size())) {
		const auto p0 = offsets[i];
		const auto p1 = offsets[i] + rects[i];
		// Larger than origin
		EXPECT_LE(0, p0(0));
		EXPECT_LE(0, p0(1));
		EXPECT_LE(0, p1(0));
		EXPECT_LE(0, p1(1));
		// Smaller than (size, size).
		EXPECT_GE(size, p0(0));
		EXPECT_GE(size, p0(1));
		EXPECT_GE(size, p1(0));
		EXPECT_GE(size, p1(1));
	}
}

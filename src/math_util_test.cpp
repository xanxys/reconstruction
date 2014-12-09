#include "math_util.h"

#include <cmath>

#include <gtest/gtest.h>

TEST(MathUtil, ceilToPowerOf2Examples) {
	using recon::ceilToPowerOf2;

	// <= 0
	EXPECT_EQ(1, ceilToPowerOf2(-123));
	EXPECT_EQ(1, ceilToPowerOf2(0));
	// >0, near 0 and boundaries
	EXPECT_EQ(1, ceilToPowerOf2(1));
	EXPECT_EQ(2, ceilToPowerOf2(2));
	EXPECT_EQ(4, ceilToPowerOf2(3));
	EXPECT_EQ(1024, ceilToPowerOf2(1023));
	EXPECT_EQ(1024, ceilToPowerOf2(1024));
	EXPECT_EQ(2048, ceilToPowerOf2(1025));
	// Large values.
	EXPECT_EQ(std::pow(2, 30), ceilToPowerOf2(std::pow(2, 29) + 1));
}

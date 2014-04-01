#include "voxel_traversal.h"

#include <iostream>
#include <random>
#include <tuple>

#include <boost/range/irange.hpp>
#include <Eigen/Dense>
#include <gtest/gtest.h>

TEST(VoxelTraversalTest, StraightCase) {
	VoxelTraversal tr(1, Eigen::Vector3f(0.5, 0.5, 0.5), Eigen::Vector3f(0, 0, 1));

	for(int i : boost::irange(0, 10)) {
		const auto key = tr.next();
		EXPECT_EQ(std::make_tuple(0, 0, i), key);
	}
}

TEST(VoxelTraversalTest, StraightCaseNegative) {
	VoxelTraversal tr(1, Eigen::Vector3f(0.5, 0.5, 0.5), Eigen::Vector3f(0, 0, -1));

	for(int i : boost::irange(0, 10)) {
		const auto key = tr.next();
		EXPECT_EQ(std::make_tuple(0, 0, 0 - i), key);
	}
}

TEST(VoxelTraversalTest, PathIsValid) {
	std::mt19937 gen;

	for(int i : boost::irange(0, 10)) {
		const Eigen::Vector3f org(
			std::uniform_real_distribution<float>(-10, 10)(gen),
			std::uniform_real_distribution<float>(-10, 10)(gen),
			std::uniform_real_distribution<float>(-10, 10)(gen));

		const auto dir = Eigen::Vector3f(
			std::uniform_real_distribution<float>(-1, 1)(gen),
			std::uniform_real_distribution<float>(-1, 1)(gen),
			std::uniform_real_distribution<float>(-1, 1)(gen)).normalized();

		VoxelTraversal tr(
			std::uniform_real_distribution<float>(1e-3, 1e3)(gen),
			org, dir);

		std::set<std::tuple<int, int, int>> prevs;
		std::tuple<int, int, int> prev;

		for(int i : boost::irange(0, 50)) {
			const auto key = tr.next();

			// Must not re-visit an old cell.
			EXPECT_EQ(prevs.end(), prevs.find(key));

			// Must be adjacent to the previous cell.
			if(i > 0) {
				const int hamming =
					std::abs(std::get<0>(prev) - std::get<0>(key)) +
					std::abs(std::get<1>(prev) - std::get<1>(key)) +
					std::abs(std::get<2>(prev) - std::get<2>(key));

				EXPECT_EQ(1, hamming);
			}

			prev = key;
			prevs.insert(prev);
		}
	}
}

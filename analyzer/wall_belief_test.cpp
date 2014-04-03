#include "wall_belief.h"

#include <memory>

#include <boost/range/irange.hpp>
#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "voxel_traversal.h"

// #include "../data_source.h"

TEST(WallBeliefTest, SplitCCReturnsNull) {
	std::vector<VoxelIndex> voxels;
	EXPECT_EQ(0, WallBelief::splitCC(voxels).size());
}

TEST(WallBeliefTest, SplitCCReturnsSingleton) {
	std::vector<VoxelIndex> voxels = {
		std::make_tuple(1, 2, 3)
	};

	const auto result = WallBelief::splitCC(voxels);
	ASSERT_EQ(1, result.size());
	ASSERT_EQ(1, result[0].size());
	EXPECT_EQ(voxels[0], result[0][0]);
}

TEST(WallBeliefTest, SplitCCConnects6) {
	std::vector<VoxelIndex> voxels = {
		std::make_tuple(0, 0, 0),
		std::make_tuple(0, 0, 1),  // 6-connected
		std::make_tuple(1, 1, 0)   // not 6-conntected
	};

	const auto result = WallBelief::splitCC(voxels);
	ASSERT_EQ(2, result.size());
}

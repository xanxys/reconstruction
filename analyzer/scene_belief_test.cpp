#include "scene_belief.h"

#include <memory>

#include <boost/range/irange.hpp>
#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "../data_source.h"

class SceneBeliefTest : public testing::Test {
protected:
	virtual void SetUp() override {
		// TODO: this call is dependent on files which are not checked in!
		// (part of large dataset)
		pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud =
			DataSource(false).getScene("MS-pumpkin-1");

		FrameBelief frame(cloud);
		auto manhattans = ManhattanBelief::expand(frame);
		auto floors = FloorBelief::expand(*manhattans[0]);
		auto walls = WallBelief::expand(*floors[0]);
		scene.reset(new SceneBelief(*walls[0]));
	}
	std::unique_ptr<SceneBelief> scene;
};

TEST_F(SceneBeliefTest, VoxelsAreNotEmpty) {
	// there should be 2 or more voxels (empty + filled)
	const auto voxels = scene->getVoxelsDetailed();
	EXPECT_GE(voxels.size(), 2);

	bool has_empty = false;
	bool has_filled = false;
	for(const auto& pair : voxels) {
		if(pair.second.state == VoxelState::OCCUPIED) {
			has_filled = true;
		} else if(pair.second.state == VoxelState::EMPTY) {
			has_empty = true;
		}
	}
	EXPECT_TRUE(has_empty);
	EXPECT_TRUE(has_filled);
}

TEST_F(SceneBeliefTest, OriginIsNotFilled) {
	for(const auto& pair : scene->getVoxelsDetailed()) {
		int x, y, z;
		std::tie(x, y, z) = pair.first;

		if(std::abs(x) + std::abs(y) + std::abs(z) <= 2) {
			EXPECT_NE(VoxelState::OCCUPIED, pair.second.state);
		}
	}
}

TEST_F(SceneBeliefTest, VoxelsAreNotTooFar) {
	for(const auto& pair : scene->getVoxelsDetailed()) {
		int x, y, z;
		std::tie(x, y, z) = pair.first;

		EXPECT_GT(1000, std::abs(x) + std::abs(y) + std::abs(z));
	}
}

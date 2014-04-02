#include "manhattan_belief.h"

#include <memory>

#include <boost/range/irange.hpp>
#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "../data_source.h"

class ManhattanBeliefTest : public testing::Test {
protected:
	virtual void SetUp() {
		// TODO: this call is dependent on files which are not checked in!
		// (part of large dataset)
		cloud = DataSource(false).getScene("MS-pumpkin-1");
		frame.reset(new FrameBelief(cloud));
	}

	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud;
	std::unique_ptr<FrameBelief> frame;
};

TEST_F(ManhattanBeliefTest, CopyConstructorPreservesVoxels) {
	auto manhattans = ManhattanBelief::expand(*frame);

	// we're supposed to get some result.
	ASSERT_GT(manhattans.size(), 0);

	ManhattanBelief manhattan2(*manhattans[0]);

	EXPECT_EQ(manhattans[0]->getVoxelsDetailed().size(),
		manhattan2.getVoxelsDetailed().size());
}

TEST_F(ManhattanBeliefTest, CameraTransformsAreConsistent) {
	auto manhattans = ManhattanBelief::expand(*frame);

	// we're supposed to get some result.
	ASSERT_GT(manhattans.size(), 0);

	const Eigen::Matrix3f should_be_identity = manhattans[0]->camera_loc_to_world * manhattans[0]->world_to_camera_loc;
	EXPECT_NEAR(0, (should_be_identity - Eigen::Matrix3f::Identity()).norm(), 1e-6);
}

TEST_F(ManhattanBeliefTest, VoxelsAreNotEmpty) {
	auto manhattans = ManhattanBelief::expand(*frame);

	// we're supposed to get some result.
	ASSERT_GT(manhattans.size(), 0);

	// there should be 2 or more voxels (empty + filled)
	const auto voxels = manhattans[0]->getVoxelsDetailed();
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

TEST_F(ManhattanBeliefTest, OriginIsNotFilled) {
	// invalid depth should have NaN, not 0,
	// so voxels near origin must be always clear.

	auto manhattans = ManhattanBelief::expand(*frame);

	// we're supposed to get some result.
	ASSERT_GT(manhattans.size(), 0);

	for(const auto& pair : manhattans[0]->getVoxelsDetailed()) {
		int x, y, z;
		std::tie(x, y, z) = pair.first;

		if(std::abs(x) + std::abs(y) + std::abs(z) <= 2) {
			EXPECT_NE(VoxelState::OCCUPIED, pair.second.state);
		}
	}
}

TEST_F(ManhattanBeliefTest, VoxelsAreNotTooFar) {

	auto manhattans = ManhattanBelief::expand(*frame);

	// we're supposed to get some result.
	ASSERT_GT(manhattans.size(), 0);

	for(const auto& pair : manhattans[0]->getVoxelsDetailed()) {
		int x, y, z;
		std::tie(x, y, z) = pair.first;

		EXPECT_GT(1000, std::abs(x) + std::abs(y) + std::abs(z));
	}
}

TEST_F(ManhattanBeliefTest, CloudIsTransformed) {
	auto manhattans = ManhattanBelief::expand(*frame);

	// we're supposed to get some result.
	ASSERT_GT(manhattans.size(), 0);

	auto& cloud_m = manhattans[0]->cloud;
	ASSERT_EQ(cloud->points.size(), cloud_m->points.size());

	for(int i : boost::irange(0, (int)cloud->points.size())) {
		if(std::isfinite(cloud->points[i].x)) {
			EXPECT_TRUE(std::isfinite(cloud->points[i].x));
			EXPECT_TRUE(std::isfinite(cloud->points[i].y));
			EXPECT_TRUE(std::isfinite(cloud->points[i].z));

			EXPECT_TRUE(std::isfinite(cloud_m->points[i].x));
			EXPECT_TRUE(std::isfinite(cloud_m->points[i].y));
			EXPECT_TRUE(std::isfinite(cloud_m->points[i].z));

			EXPECT_GT(
				(cloud->points[i].getVector3fMap() - cloud_m->points[i].getVector3fMap()).norm(),
				0);
		}
	}
}

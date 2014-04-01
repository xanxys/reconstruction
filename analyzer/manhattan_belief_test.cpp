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

	// there should be more than 0 voxels
	EXPECT_GT(manhattans[0]->getVoxelsDetailed().size(), 0);
}

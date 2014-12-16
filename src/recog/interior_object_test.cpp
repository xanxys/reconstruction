#include "interior_object.h"

#include <gtest/gtest.h>

#include <visual/mapping.h>


TEST(InteriorObject, FloorLevelIdentity) {
	// Create [-0.5, -0.5, 0] x [0.5, 0.5, 1] box.
	// (lowest z = 0)
	recon::TexturedMesh tm;
	tm.mesh = recon::mapSecond(recon::assignUV(
		recon::createBox(Eigen::Vector3f(0, 0, 0.5), 0.5)));
	tm.diffuse = cv::Mat(64, 64, CV_8UC3);

	std::vector<recon::OBB3f> collisions;
	collisions.emplace_back(recon::AABB3f(Eigen::Vector3f(-0.5, -0.5, 0), Eigen::Vector3f(0.5, 0.5, 1)));
	recon::InteriorObject iobj(tm, collisions);

	// No rotation introduced.
	EXPECT_FLOAT_EQ(0, (iobj.getPose().linear() - Eigen::Matrix3f::Identity()).norm());
	// XY-center is 0 (uncahanged)
	EXPECT_FLOAT_EQ(0, iobj.getPose().translation().x());
	EXPECT_FLOAT_EQ(0, iobj.getPose().translation().y());
	// Floor level is 0 (unchanged)
	EXPECT_FLOAT_EQ(0, iobj.getPose().translation().z());

	// Collision is ok.
	const auto collision = iobj.getCollision();
	ASSERT_EQ(1, collision.size());
	EXPECT_FLOAT_EQ(0,
		(collision[0].toAABB().getMin() - Eigen::Vector3f(-0.5, -0.5, 0)).norm());
	EXPECT_FLOAT_EQ(0,
		(collision[0].toAABB().getMax() - Eigen::Vector3f(0.5, 0.5, 1)).norm());
}

TEST(InteriorObject, GuessFloorLevel) {
	// Create [-0.5, -0.5, -0.5] x [0.5, 0.5, 0.5] box.
	// (lowest z = -0.5) and expect floor level adjusted to z=0.
	recon::TexturedMesh tm;
	tm.mesh = recon::mapSecond(recon::assignUV(
		recon::createBox(Eigen::Vector3f(0, 0, 0), 0.5)));
	tm.diffuse = cv::Mat(64, 64, CV_8UC3);

	std::vector<recon::OBB3f> collisions;
	collisions.emplace_back(recon::AABB3f(
		Eigen::Vector3f(-0.5, -0.5, -0.5), Eigen::Vector3f(0.5, 0.5, 0.5)));
	recon::InteriorObject iobj(tm, collisions);

	// No rotation introduced.
	EXPECT_FLOAT_EQ(0, (iobj.getPose().linear() - Eigen::Matrix3f::Identity()).norm());
	// XY-center is 0 (uncahanged)
	EXPECT_FLOAT_EQ(0, iobj.getPose().translation().x());
	EXPECT_FLOAT_EQ(0, iobj.getPose().translation().y());
	// Floor level is -0.5.
	EXPECT_FLOAT_EQ(-0.5, iobj.getPose().translation().z());

	// Collision is adjusted above z>=0.
	const auto collision = iobj.getCollision();
	ASSERT_EQ(1, collision.size());
	EXPECT_FLOAT_EQ(0,
		(collision[0].toAABB().getMin() - Eigen::Vector3f(-0.5, -0.5, 0)).norm());
	EXPECT_FLOAT_EQ(0,
		(collision[0].toAABB().getMax() - Eigen::Vector3f(0.5, 0.5, 1)).norm());

	// mesh is adjusted above z>=0.
	const auto mesh_after = iobj.getMesh().mesh;
	ASSERT_EQ(tm.mesh.triangles, mesh_after.triangles);
	ASSERT_EQ(tm.mesh.vertices.size(), mesh_after.vertices.size());
	int iv = 0;
	for(const auto& vert : mesh_after.vertices) {
		const auto org_pos = tm.mesh.vertices[iv++].first;
		EXPECT_FLOAT_EQ(org_pos.x(), vert.first.x());
		EXPECT_FLOAT_EQ(org_pos.y(), vert.first.y());
		EXPECT_LE(0, vert.first.z());
	}
}

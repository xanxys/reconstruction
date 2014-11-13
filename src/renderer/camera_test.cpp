#include "camera.h"

#include <gtest/gtest.h>

TEST(CameraTest, OriginProjectsToCenter) {
	Camera camera_vga(640, 480, 1.04719755, Eigen::Transform<float, 3, Eigen::Affine>::Identity());
	
	const auto proj = camera_vga.getMatrix();
	const Eigen::Vector4f pt1(0, 0, 1, 1);
	const Eigen::Vector4f pt2(0, 0, 2, 1);

	const Eigen::Vector3f pt1_ndc = (proj * pt1).head(3) / (proj * pt1)(3);
	EXPECT_FLOAT_EQ(0, pt1_ndc.x());
	EXPECT_FLOAT_EQ(0, pt1_ndc.y());

	const Eigen::Vector3f pt2_ndc = (proj * pt2).head(3) / (proj * pt2)(3);
	EXPECT_FLOAT_EQ(0, pt2_ndc.x());
	EXPECT_FLOAT_EQ(0, pt2_ndc.y());

	// pt2 is farther than pt1.
	EXPECT_GT(pt2_ndc.z(), pt1_ndc.z());
}

TEST(CameraTest, EdgesAreCorrect) {
	// fovv = fovh = 90 degree
	Camera camera_vga(500, 500, 1.57079632679, Eigen::Transform<float, 3, Eigen::Affine>::Identity());

	const auto proj = camera_vga.getMatrix();
	const Eigen::Vector4f pt_r_world(1, 0, 1, 1);
	const Eigen::Vector4f pt_t_world(0, -1, 1, 1);

	const Eigen::Vector3f pt_r_ndc = (proj * pt_r_world).head(3) / (proj * pt_r_world)(3);
	EXPECT_FLOAT_EQ(1, pt_r_ndc.x());
	EXPECT_FLOAT_EQ(0, pt_r_ndc.y());

	const Eigen::Vector3f pt_t_ndc = (proj * pt_t_world).head(3) / (proj * pt_t_world)(3);
	EXPECT_FLOAT_EQ(0, pt_t_ndc.x());
	EXPECT_FLOAT_EQ(1, pt_t_ndc.y());
}

#include "aligned_scan.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <jsoncpp/json/json.h>

TEST(AlignedScans, AffineEncodingIdentity) {
	Eigen::Matrix4f m;
	m.row(0) = Eigen::Vector4f(1, 2, 3, 4);
	m.row(1) = Eigen::Vector4f(5, 6, 7, 8);
	m.row(2) = Eigen::Vector4f(0, 1, 2, 3);

	const Eigen::Affine3f aff(m);
	const Eigen::Affine3f aff_de_en =
		visual::AlignedScans::decodeAffine(
			visual::AlignedScans::encodeAffine(aff));
	EXPECT_FLOAT_EQ(0, (aff.matrix() - aff_de_en.matrix()).norm());
}

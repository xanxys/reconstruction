#include "textured_mesh.h"

#include <vector>

#include <boost/range/irange.hpp>
#include <gtest/gtest.h>

TEST(mergeTexturedMeshes, TextureAreaIsPreserved) {
	const cv::Scalar col0(255, 0, 0);
	const cv::Scalar col1(0, 255, 0);
	std::vector<recon::TexturedMesh> tms;
	{
		recon::TexturedMesh tm0;
		tm0.diffuse = cv::Mat(64, 64, CV_8UC3);
		tm0.diffuse = col0;
		tms.push_back(tm0);
	}
	{
		recon::TexturedMesh tm1;
		tm1.diffuse = cv::Mat(128, 128, CV_8UC3);
		tm1.diffuse = col1;
		tms.push_back(tm1);
	}

	const auto merged = recon::mergeTexturedMeshes(tms);
	// area must be larger or equal to sum of originals.
	EXPECT_GE(merged.diffuse.cols * merged.diffuse.rows, 64 * 64 + 128 * 128);
	// #pixels must be preserved when copying texture.
	int count_col0 = 0;
	int count_col1 = 0;
	for(auto it = merged.diffuse.begin<cv::Vec3b>();
			it != merged.diffuse.end<cv::Vec3b>(); it++) {
		if(cv::Scalar(*it) == col0) {
			count_col0++;
		} else if(cv::Scalar(*it) == col1) {
			count_col1++;
		}
	}

	EXPECT_EQ(64 * 64, count_col0);
	EXPECT_EQ(128 * 128, count_col1);
}

#include "textured_mesh.h"

#include <vector>

#include <boost/range/irange.hpp>
#include <gtest/gtest.h>

TEST(mergeTexturedMeshes, TexturePixelsArePreserved) {
	const cv::Scalar col0(255, 0, 0);
	const cv::Scalar col1(0, 255, 0);
	std::vector<recon::TexturedMesh> tms;
	{
		cv::Mat diffuse(64, 64, CV_8UC3);
		diffuse = col0;
		tms.emplace_back(
			recon::TriangleMesh<Eigen::Vector2f>(),
			diffuse);
	}
	{
		cv::Mat diffuse(128, 128, CV_8UC3);
		diffuse = col1;
		tms.emplace_back(
			recon::TriangleMesh<Eigen::Vector2f>(),
			diffuse);
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

TEST(mergeTexturedMeshes, TextureMappingIsOk) {
	const cv::Vec3b col0(255, 0, 0);
	const cv::Vec3b col1(0, 255, 0);
	std::vector<recon::TexturedMesh> tms;
	{
		recon::TriangleMesh<Eigen::Vector2f> mesh;
		mesh.vertices.emplace_back(
			Eigen::Vector3f::Zero(),
			Eigen::Vector2f(0.5, 0.5));

		cv::Mat diffuse(64, 64, CV_8UC3);
		diffuse = cv::Scalar(col0);

		tms.emplace_back(mesh, diffuse);
	}
	{
		recon::TriangleMesh<Eigen::Vector2f> mesh;
		mesh.vertices.emplace_back(
			Eigen::Vector3f::Ones(),
			Eigen::Vector2f(0.5, 0.5));

		cv::Mat diffuse(128, 128, CV_8UC3);
		diffuse = cv::Scalar(col1);

		tms.emplace_back(mesh, diffuse);
	}

	const auto merged = recon::mergeTexturedMeshes(tms);
	ASSERT_EQ(2, merged.mesh.vertices.size());

	const auto uv0 = merged.mesh.vertices[0].second;
	ASSERT_LE(0, uv0.x());
	ASSERT_LE(0, uv0.y());
	ASSERT_GE(1, uv0.x());
	ASSERT_GE(1, uv0.y());
	EXPECT_EQ(col0,
		merged.diffuse.at<cv::Vec3b>(
			(1 - uv0.y()) * merged.diffuse.rows,
			uv0.x() * merged.diffuse.cols));

	const auto uv1 = merged.mesh.vertices[1].second;
	ASSERT_LE(0, uv1.x());
	ASSERT_LE(0, uv1.y());
	ASSERT_GE(1, uv1.x());
	ASSERT_GE(1, uv1.y());
	EXPECT_EQ(col1,
		merged.diffuse.at<cv::Vec3b>(
			(1 - uv1.y()) * merged.diffuse.rows,
			uv1.x() * merged.diffuse.cols));
}

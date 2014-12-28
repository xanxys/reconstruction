#include "toy_scene.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include <boost/range/irange.hpp>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include <geom/util.h>
#include <logging.h>
#include <recog/interior_boundary.h>
#include <recog/interior_object.h>
#include <visual/mapping.h>
#include <visual/textured_mesh.h>
#include <visual/triangle_mesh.h>

namespace recon {

void populateToyScene(SceneAssetBundle& bundle) {
	INFO("Populating bundle with toys");

	// Create a texture image of a 1px black grid, overlaid
	// on top of flat base_color background.
	auto gen_grid_tex = [](const cv::Vec3b base_color) {
		cv::Mat grid_tex(256, 256, CV_8UC3);
		for(const int y : boost::irange(0, 256)) {
			for(const int x : boost::irange(0, 256)) {
				grid_tex.at<cv::Vec3b>(y, x) =
					((x % 16 == 0) || (y % 16 == 0)) ?
					cv::Vec3b(0, 0, 0) : base_color;
			}
		}
		return grid_tex;
	};

	// Set coordinates.
	bundle.setFloorLevel(0);

	// Set InteriorBoundary.
	{
		// Create 6x5x3 meter room, spanning
		// Notice Y-asymmetry, to test Y-flip phenomenon that
		// once ocurred.
		// [-3, -2, 0], [3, 3, 3]
		const auto mesh = mapSecond(assignUV(
			flipTriangles(
			createBox(Eigen::Vector3f(0, 0.5, 1.5),
				Eigen::Vector3f(3, 0, 0),
				Eigen::Vector3f(0, 2.5, 0),
				Eigen::Vector3f(0, 0, 1.5)))));

		TexturedMesh tm(mesh, gen_grid_tex(cv::Vec3b(255, 255, 255)));

		std::vector<Eigen::Vector2f> polygon = {
			{-3, -2},
			{3, -2},
			{3, 3},
			{-3, 3}
		};
		std::pair<float, float> z_range = {0, 3};
		bundle.setInteriorBoundary(
			InteriorBoundary(tm, polygon, z_range));
	}

	// Add Light.
	bundle.addPointLight(Eigen::Vector3f(0, 0, 2.95));

	// Add green InteriorObject (simple cube).
	// [-0.5, -0.5, 0], [0.5, 0.5, 1]
	{
		TexturedMesh tm(
			mapSecond(assignUV(createBox(Eigen::Vector3f(0, 0, 0.5), 0.5))),
			gen_grid_tex(cv::Vec3b(200, 255, 200)));

		std::vector<OBB3f> collisions;
		collisions.emplace_back(AABB3f(Eigen::Vector3f(-0.5, -0.5, 0), Eigen::Vector3f(0.5, 0.5, 1)));
		InteriorObject iobj(tm, collisions);
		bundle.addInteriorObject(iobj);
	}
	// Add red InteriorObject (small cube + stick attached on top).
	// occupating [-1.5, -1.5, 0], [-1, -1, 1]
	//     |   0.5m stick
	//   |---|  0.5m box
	//__ |___| __
	{
		auto mesh = createBox(Eigen::Vector3f(-1.25, -1.25, 0.25), 0.25);
		mesh.merge(createBox(Eigen::Vector3f(-1.25, -1.25, 0.75),
			Eigen::Vector3f(0.1, 0, 0),
			Eigen::Vector3f(0, 0.1, 0),
			Eigen::Vector3f(0, 0, 0.25)));

		TexturedMesh tm(
			mapSecond(assignUV(mesh)),
			gen_grid_tex(cv::Vec3b(200, 200, 255)));

		std::vector<OBB3f> collisions;
		collisions.emplace_back(AABB3f(
			Eigen::Vector3f(-1.5, -1.5, 0),
			Eigen::Vector3f(-1, -1, 0.5)));
		collisions.emplace_back(AABB3f(
			Eigen::Vector3f(-1.35, -1.35, 0.5),
			Eigen::Vector3f(-1.15, -1.15, 1)));
		InteriorObject iobj(tm, collisions);
		bundle.addInteriorObject(iobj);
	}
}

}  // namespace

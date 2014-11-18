// Put large, module-spanning tests here.
#include <fstream>
#include <map>
#include <random>

#include <logging.h>
#include <visual/cloud_baker.h>
#include <visual/cloud_conversion.h>
#include <visual/mapping.h>
#include <visual/marching_cubes.h>
#include <visual/texture_conversion.h>

#include <gtest/gtest.h>

TEST(MeshPipelineTest, IsosurfaceWorks) {
	INFO("creating metaball");
	const auto mesh_n = visual::extractIsosurface(
		3, [](Eigen::Vector3f p) {
			return (
				1 / (0.01 + p.norm()) +
				1 / (0.01 + (p - Eigen::Vector3f(0.9,0.9,0.9)).norm())
				);
		}, std::make_pair(
			Eigen::Vector3f(-3, -3, -3),
			Eigen::Vector3f(3, 3, 3)),
		0.1);
	std::ofstream test("/tmp/recon-MeshPipelineTest-test.ply");
	mesh_n.serializePLY(test);

	const auto mesh_n_uv = visual::assignUV(mesh_n);
	const auto mesh_uv = visual::mapSecond(mesh_n_uv);
	cv::imwrite("uv.png", visual::visualizeUVMap(mesh_uv));
	cv::imwrite("uv_3d.png", visual::bake3DTexture(mesh_uv,
		[](Eigen::Vector3f p) {
			return p;
		}));

	std::ofstream test_uv("/tmp/recon-MeshPipelineTest-test_uv.obj");
	std::ofstream test_mat("/tmp/recon-MeshPipelineTest-test_uv.mtl");
	mesh_uv.serializeObjWithUv(test_uv, "/tmp/recon-MeshPipelineTest-test_uv.mtl");
	visual::writeObjMaterial(test_mat, "/tmp/recon-MeshPipelineTest-uv_3d.png");
}

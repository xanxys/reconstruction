#include "mapping.h"

#include <vector>

#include <boost/range/irange.hpp>
#include <gtest/gtest.h>

TEST(Chart, SingleTriangle) {
	const std::vector<std::array<int, 3>> tris {
		{{3, 4, 5}}
	};
	const std::map<int, Eigen::Vector2f>& pre_uv {
		{3, {-1, -1}},
		{4, {0, -1}},
		{5, {-1, 0}}
	};
	// Current packer won't rotate triangle.
	visual::Chart chart(tris, pre_uv);
	const auto size = chart.getSize();
	EXPECT_FLOAT_EQ(size(0), 1);
	EXPECT_FLOAT_EQ(size(1), 1);

	const auto mesh = chart.getMesh();
	// Check local vertex - external vertex mapping.
	EXPECT_EQ(1, mesh.triangles.size());
	EXPECT_EQ(3, mesh.vertices.size());
	EXPECT_EQ((std::array<int, 3>{{0, 1, 2}}), mesh.triangles[0]);
	// TODO: Pos is stored in vertex.first, but we don't care about it.
	EXPECT_EQ(3, mesh.vertices[0].second.first);
	EXPECT_EQ(4, mesh.vertices[1].second.first);
	EXPECT_EQ(5, mesh.vertices[2].second.first);
	// Check UV mapping.
	for(const auto& vert : mesh.vertices) {
		EXPECT_LE(0, vert.second.second(0));
		EXPECT_LE(0, vert.second.second(1));
		EXPECT_GE(size(0), vert.second.second(0));
		EXPECT_GE(size(1), vert.second.second(1));
	}
}

TEST(getTriangleAdjacency, SingleTriangleHasNoAdjacency) {
	visual::TriangleMesh<std::nullptr_t> mesh;
	mesh.triangles.push_back({{0, 1, 2}});
	mesh.vertices.emplace_back(Eigen::Vector3f(0, 0, 0), nullptr);
	mesh.vertices.emplace_back(Eigen::Vector3f(1, 0, 0), nullptr);
	mesh.vertices.emplace_back(Eigen::Vector3f(0, 1, 0), nullptr);

	const auto adj = visual::getTriangleAdjacency(mesh);
	EXPECT_TRUE(adj.empty());
}

TEST(getTriangleAdjacency, TwoTrianglesSharingEdge) {
	// getTriangleAdjacency is a topological operation,
	// so positions don't matter.
	visual::TriangleMesh<std::nullptr_t> mesh;
	mesh.triangles.push_back({{0, 1, 2}});  // tri 0
	mesh.triangles.push_back({{1, 0, 3}});  // tri 1
	mesh.vertices.emplace_back(Eigen::Vector3f::Zero(), nullptr);
	mesh.vertices.emplace_back(Eigen::Vector3f::Zero(), nullptr);
	mesh.vertices.emplace_back(Eigen::Vector3f::Zero(), nullptr);
	mesh.vertices.emplace_back(Eigen::Vector3f::Zero(), nullptr);

	const auto adj = visual::getTriangleAdjacency(mesh);
	const std::map<int, std::set<int>> truth_adj = {
		{0, {1}},
		{1, {0}}
	};
	EXPECT_EQ(truth_adj, adj);
}

TEST(getTriangleAdjacency, TwoTrianglesSharingVertex) {
	// getTriangleAdjacency is a topological operation,
	// so positions don't matter.
	visual::TriangleMesh<std::nullptr_t> mesh;
	mesh.triangles.push_back({{0, 1, 2}});  // tri 0
	mesh.triangles.push_back({{0, 3, 4}});  // tri 1
	mesh.vertices.emplace_back(Eigen::Vector3f::Zero(), nullptr);
	mesh.vertices.emplace_back(Eigen::Vector3f::Zero(), nullptr);
	mesh.vertices.emplace_back(Eigen::Vector3f::Zero(), nullptr);
	mesh.vertices.emplace_back(Eigen::Vector3f::Zero(), nullptr);
	mesh.vertices.emplace_back(Eigen::Vector3f::Zero(), nullptr);

	const auto adj = visual::getTriangleAdjacency(mesh);
	const std::map<int, std::set<int>> truth_adj = {
		{0, {1}},
		{1, {0}}
	};
	EXPECT_EQ(truth_adj, adj);
}

TEST(getTriangleAdjacency, TwoTrianglesSeparate) {
	// getTriangleAdjacency is a topological operation,
	// so positions don't matter.
	visual::TriangleMesh<std::nullptr_t> mesh;
	mesh.triangles.push_back({{0, 1, 2}});  // tri 0
	mesh.triangles.push_back({{3, 4, 5}});  // tri 1
	mesh.vertices.emplace_back(Eigen::Vector3f::Zero(), nullptr);
	mesh.vertices.emplace_back(Eigen::Vector3f::Zero(), nullptr);
	mesh.vertices.emplace_back(Eigen::Vector3f::Zero(), nullptr);
	mesh.vertices.emplace_back(Eigen::Vector3f::Zero(), nullptr);
	mesh.vertices.emplace_back(Eigen::Vector3f::Zero(), nullptr);
	mesh.vertices.emplace_back(Eigen::Vector3f::Zero(), nullptr);

	const auto adj = visual::getTriangleAdjacency(mesh);
	const std::map<int, std::set<int>> truth_adj = {};
	EXPECT_EQ(truth_adj, adj);
}


TEST(getCCTest, EmptyGraph) {
	std::set<int> verts;
	std::map<int, std::set<int>> adj;
	EXPECT_TRUE(visual::getCC(verts, adj).empty());
}

TEST(getCCTest, TwoCC) {
	std::set<int> verts{1, 2};
	std::map<int, std::set<int>> adj{
		{1, {}},
		{2, {}}
	};
	const auto ccs = visual::getCC(verts, adj);
	EXPECT_EQ(2, ccs.size());
	EXPECT_EQ(1, ccs[0].size());
	EXPECT_EQ(1, ccs[1].size());
}

TEST(getCCTest, OneCC) {
	std::set<int> verts{1, 2};
	std::map<int, std::set<int>> adj{
		{1, {2}},
		{2, {1}}
	};
	const auto ccs = visual::getCC(verts, adj);
	EXPECT_EQ(1, ccs.size());
	EXPECT_EQ(2, ccs[0].size());
}

TEST(assignUV, UVIsInsideUnitSquare) {
	// single chart
	visual::TriangleMesh<std::nullptr_t> mesh;
	mesh.triangles.push_back({{0, 1, 2}});
	mesh.vertices.emplace_back(Eigen::Vector3f(0, 0, 0), nullptr);
	mesh.vertices.emplace_back(Eigen::Vector3f(1, 0, 0), nullptr);
	mesh.vertices.emplace_back(Eigen::Vector3f(0, 1, 0), nullptr);

	const auto mesh_with_uv = visual::assignUV(mesh);
	EXPECT_EQ(mesh_with_uv.triangles.size(), 1);
	EXPECT_EQ(mesh_with_uv.vertices.size(), 3);
	for(const auto& vert : mesh_with_uv.vertices) {
		const Eigen::Vector2f uv = vert.second.second;
		EXPECT_LE(0, uv(0));
		EXPECT_LE(0, uv(1));
		EXPECT_GE(1, uv(0));
		EXPECT_GE(1, uv(1));
	}
}

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <set>
#include <vector>

#include <boost/optional.hpp>
#include <boost/range/irange.hpp>
#include <Eigen/Dense>

#include <geom/packing.h>
#include <visual/triangle_mesh.h>

namespace recon {

// Partial TriangleMesh that can be represented as
// connected region in 2-d plane.
// (similar to charts in topology)
//
// Connected triangles mapped to 2-d surface without overlapping.
// spans [0, u_size] * [0, v_size]
class Chart {
public:
	// pre_uv: whatever UV coordinates in R^2.
	// Scale will be preserved by Chart, but translations
	// and rotations will be modified, so that
	// triangles will fit smallest AABB of
	// shape [0, u_size] * [0, v_size].
	//
	// both tris and pre_uv will use some external indices.
	Chart(
		const std::vector<std::array<int, 3>>& tris,
		const std::map<int, Eigen::Vector2f>& pre_uv);

	// Returns size after packing (u_size, v_size)
	// time: O(1)
	Eigen::Vector2f getSize() const;

	// Calculates a TrianglMesh with supplied index and
	// packed UV. Position will be undefined.
	// time: O(V)
	TriangleMesh<std::pair<int, Eigen::Vector2f>> getMesh() const;
private:
	// local vertex -> (original vertex index, local UV)]
	std::vector<std::pair<int, Eigen::Vector2f>> vertices;
	// local vertices
	std::vector<std::array<int, 3>> triangles;
	// AABB
	Eigen::Vector2f uv_min;
	Eigen::Vector2f uv_max;
};

std::vector<Chart> divideMeshToCharts(
	const TriangleMesh<std::nullptr_t>& mesh);

// Return adjacency of triangles.
// Two triangles are considered ajacent when they
// share one or more vertices.
// (NO NEED TO SHARE AN EDGE!)
std::map<int, std::set<int>> getTriangleAdjacency(
	const TriangleMesh<std::nullptr_t>& mesh);

// Create a new TriangleMesh with automatically generated UV coordinates.
// UV is represented as Eigen::Vector2f, and will span [0,1]^2.
//
// Number of vertices will generally increase after UV mapping,
// since it's often necessary to split a mesh into several components,
// thus assigning multiple UVs to vertices sharing same location
// (originally represented by a single vertex).
//
// Currently, assignUV won't distort mesh; that is,
// even a slight angle cause neighboring triangles to be placed
// far away.
template<typename Vertex>
TriangleMesh<std::pair<Vertex, Eigen::Vector2f>> assignUV(
		const TriangleMesh<Vertex>& mesh) {
	// Split mesh into Charts, and pack charts.
	const auto charts = divideMeshToCharts(dropAttrib(mesh));

	std::vector<Eigen::Vector2f> uv_sizes;
	for(const auto& chart : charts) {
		uv_sizes.push_back(chart.getSize());
	}

	const auto result = packRectangles(uv_sizes);
	const float scale = 1 / std::get<0>(result);
	const auto offsets = std::get<1>(result);

	// Map local UV to global UV using rectangle packing result.
	// A vertex will not be shared (i.e. will be duplicated with
	// different UVs) if it spans multiple Charts.
	TriangleMesh<std::pair<Vertex, Eigen::Vector2f>> mesh_with_uv;
	for(int i : boost::irange(0, (int)charts.size())) {
		const Eigen::Vector2f offset = offsets[i];
		const auto mesh_chart_local = charts[i].getMesh();
		// Replace vertex id -> Vertex, and local UV -> global UV.
		TriangleMesh<std::pair<Vertex, Eigen::Vector2f>> mesh_chart;
		mesh_chart.triangles = mesh_chart_local.triangles;
		for(const auto& vert_local : mesh_chart_local.vertices) {
			mesh_chart.vertices.emplace_back(
				mesh.vertices[vert_local.second.first].first,
				std::make_pair(
					mesh.vertices[vert_local.second.first].second,
					(vert_local.second.second + offset) * scale));
		}
		mesh_with_uv.merge(mesh_chart);
	}
	return mesh_with_uv;
}

}  // namespace

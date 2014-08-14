#pragma once

#include <array>
#include <algorithm>
#include <cmath>
#include <vector>

#include <boost/optional.hpp>
#include <boost/range/irange.hpp>
#include <Eigen/Dense>

#include "triangle_mesh.h"

namespace visual {

// Try to find packing of rectangles in a (minimum) square region.
// Obviously, it's NP-hard problem, so don't expect near-optimal result.
// This function is optimized for horizontally-long rectangles.
//
// rectangles: size of each rectangle ((0,0) is assumed to be origins)
// return: (size of square, offset of rectangle)
// returned offset is such that all rectangles will fit in [0,square]^2
// without any overlaps.
std::pair<float, std::vector<Eigen::Vector2f>>
	packRectangles(std::vector<Eigen::Vector2f> rectangles);


// Create a new TriangleMesh with automatically generated UV coordinates.
// UV is represented as Eigen::Vector2f, and will span [0,1]^2.
//
// Number of vertices will generally increase after UV mapping,
// since it's often necessary to split a mesh into several components,
// thus assigning multiple UVs to vertices sharing same location
// (originally represented by a single vertex).
template<typename Vertex>
TriangleMesh<std::pair<Vertex, Eigen::Vector2f>> assignUV(const TriangleMesh<Vertex>& mesh) {
	// Wrap all triangles in rectangles.
	std::vector<Eigen::Vector2f> uv_sizes;
	std::vector<std::tuple<
		Eigen::Vector2f, Eigen::Vector2f, Eigen::Vector2f>> local_uvs;
	for(const auto& triangle : mesh.triangles) {
		const std::array<Eigen::Vector3f, 3> vs = {
			mesh.vertices[std::get<0>(triangle)].first,
			mesh.vertices[std::get<1>(triangle)].first,
			mesh.vertices[std::get<2>(triangle)].first
		};

		// Pack local UVs like
		// ----- top
		//  \  |
		//    \| bottom
		// and top has longest edge.
		// In this configuration, it's guranteed that the angle (>90)
		// (if there's one) is always bottom.
		const float d01 = (vs[1] - vs[0]).norm();
		const float d12 = (vs[2] - vs[1]).norm();
		const float d20 = (vs[0] - vs[2]).norm();

		int ix_topleft, ix_topright, ix_bottom;
		if(d01 > d12 && d01 > d20) {
			ix_topleft = 0;
			ix_topright = 1;
			ix_bottom = 2;
		} else if(d12 > d20) {
			ix_topleft = 1;
			ix_topright = 2;
			ix_bottom = 0;
		} else {
			ix_topleft = 2;
			ix_topright = 0;
			ix_bottom = 1;
		}

		const float width = (vs[ix_topright] - vs[ix_topleft]).norm();
		const float semiperimeter = (d01 + d12 + d20) / 2;
		const float height = 2 * std::sqrt(
			semiperimeter * (semiperimeter - d01) *
			(semiperimeter - d12) * (semiperimeter - d20)) / width;

		std::array<Eigen::Vector2f, 3> uvs;
		uvs[ix_topleft] = Eigen::Vector2f(0, 0);
		uvs[ix_topright] = Eigen::Vector2f(width, 0);
		uvs[ix_bottom] = Eigen::Vector2f(
			std::sqrt((vs[ix_topleft] - vs[ix_bottom]).squaredNorm() - std::pow(height, 2)),
			height);

		// Crete padding to avoid color breeding
		//  +----------+----------+  <---
		//  | orig     |          |   | height
		//  | rect     |          |   |
		//  |----------+----------|  <---
		//  |          |          |
		//  |          |          |
		//  +----------+----------+
		//  <--width-->
		uv_sizes.emplace_back(width * 2, height * 2);
		local_uvs.emplace_back(uvs[0], uvs[1], uvs[2]);
	}

	// Map local UV to global UV using rectangle packing result.
	const auto result = packRectangles(uv_sizes);
	const float scale = 1 / std::get<0>(result);
	const auto offsets = std::get<1>(result);

	TriangleMesh<std::pair<Vertex, Eigen::Vector2f>> mesh_with_uv;
	for(int i : boost::irange(0, (int)mesh.triangles.size())) {
		const Eigen::Vector2f offset = offsets[i];

		// Put vertices
		const auto& v0 = mesh.vertices[std::get<0>(mesh.triangles[i])];
		mesh_with_uv.vertices.emplace_back(v0.first, std::make_pair(
				v0.second,
				(offset + std::get<0>(local_uvs[i])) * scale));
		const auto& v1 = mesh.vertices[std::get<1>(mesh.triangles[i])];
		mesh_with_uv.vertices.emplace_back(v1.first, std::make_pair(
				v1.second,
				(offset + std::get<1>(local_uvs[i])) * scale));
		const auto& v2 = mesh.vertices[std::get<2>(mesh.triangles[i])];
		mesh_with_uv.vertices.emplace_back(v2.first, std::make_pair(
				v2.second,
				(offset + std::get<2>(local_uvs[i])) * scale));

		// Put tri.
		mesh_with_uv.triangles.emplace_back(i * 3 + 0, i * 3 + 1, i * 3 + 2);
	}
	return mesh_with_uv;
}

template<typename Vertex>
TriangleMesh<std::nullptr_t> dropAttrib(const TriangleMesh<Vertex>& mesh) {
	TriangleMesh<std::nullptr_t> new_mesh;
	new_mesh.triangles = mesh.triangles;
	for(const auto& vert : mesh.vertices) {
		new_mesh.vertices.push_back(std::make_pair(vert.first, nullptr));
	}
	return new_mesh;
}

}  // namespace

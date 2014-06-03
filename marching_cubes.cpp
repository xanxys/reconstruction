#include "marching_cubes.h"

#include <cmath>
#include <map>

#include <boost/range/irange.hpp>

#include "range3.h"

F32Array3::F32Array3(int nx, int ny, int nz) : nx(nx), ny(ny), nz(nz) {
	data.resize(nx * ny * nz);
}




// TODO: fuse vertices
TriangleMesh<Eigen::Vector3f> extractIsosurface(
	float value, ScalarField3 field,
	BoundingBox region, float resolution) {
	assert(resolution > 0);

	const Eigen::Vector3i size =
		((std::get<1>(region) - std::get<0>(region)) / resolution).cast<int>();

	TriangleMesh<Eigen::Vector3f> mesh;
	for(const Eigen::Vector3i i : range3(size)) {
		const auto vertex0 = std::get<0>(region) + i.cast<float>() * resolution;

		// vertex index encoding
		// MSB:0000 0ZYX:LSB
		std::array<float, 8> values;
		std::array<Eigen::Vector3f, 8> positions;
		std::array<Eigen::Vector3f, 8> normals;
		for(int i_vertex : boost::irange(0, 8)) {
			/*
			const auto delta = Eigen::Vector3f(
				(i_vertex >> 0) & 1,
				(i_vertex >> 1) & 1,
				(i_vertex >> 2) & 1) * resolution;
				*/

			const auto delta = Eigen::Vector3f(
				((i_vertex & 3) == 1 || (i_vertex & 3) == 2) ? 1 : 0,
				((i_vertex & 3) == 2 || (i_vertex & 3) == 3) ? 1 : 0,
				(i_vertex >> 2) & 1) * resolution;
			const auto pos = vertex0 + delta;
			positions[i_vertex] = pos;
			values[i_vertex] = field(pos);
			const float dvdx = (field(pos + Eigen::Vector3f(resolution, 0, 0))
				- field(pos - Eigen::Vector3f(resolution, 0, 0))) / (2 * resolution);
			const float dvdy = (field(pos + Eigen::Vector3f(0, resolution, 0))
				- field(pos - Eigen::Vector3f(0, resolution, 0))) / (2 * resolution);
			const float dvdz = (field(pos + Eigen::Vector3f(0, 0, resolution))
				- field(pos - Eigen::Vector3f(0, 0, resolution))) / (2 * resolution);
			normals[i_vertex] = -Eigen::Vector3f(dvdx, dvdy, dvdz).normalized();
		}
		mesh.merge(tesselateCube(value, values, positions, normals));
	}
	return mesh;
}

// TODO: I don't understand MC exactly. Please replace
// large cryptic numeric table with self-explaining compile-time code.
TriangleMesh<Eigen::Vector3f> tesselateCube(float v_surface,
	const std::array<float, 8>& values,
	const std::array<Eigen::Vector3f, 8>& positions,
	const std::array<Eigen::Vector3f, 8>& normals) {

	// map (>=v_surface) values, bitwise. (LSB==head)
	uint8_t mask_inside = 0;
	for(float v : values) {
		mask_inside >>= 1;
		mask_inside |= (v < v_surface) ? 0x80 : 0;
	}

	// The cube is completely outside or inside.
	if(mask_inside == 0 || mask_inside == 0xff) {
		return TriangleMesh<Eigen::Vector3f>();
	}

	// Generate vertices in sparse arrays.
	const uint16_t edge_flag = edgeTable[mask_inside];

	std::array<Eigen::Vector3f, 12> vert_pos;
	std::array<Eigen::Vector3f, 12> vert_normal;
	for(int i : boost::irange(0, 12)) {
		if(edge_flag && (1 << i) == 0) {
			continue;
		}
		const auto vert_index_pair = edgeToVertex[i];
		const int vert0 = std::get<0>(vert_index_pair);
		const int vert1 = std::get<1>(vert_index_pair);

		// Inverse linear-interpolate value.
		const float t = std::max(0.0f, std::min(1.0f,
			(v_surface - values[vert0]) / (values[vert1] - values[vert0])));

		// Calculate face vertex attributes by linear interpolation.
		vert_pos[i] = positions[vert0] + (positions[vert1] - positions[vert0]) * t;
		vert_normal[i] = normals[vert0] + (normals[vert1] - normals[vert0]) * t;
	}

	// Generate triangles, ignoring unused vertices.
	const auto tri_template = triTable[mask_inside];
	TriangleMesh<Eigen::Vector3f> delta;
	std::map<int, int> compressed_vertex_index;
	for(const int i_vert : tri_template) {
		compressed_vertex_index[i_vert] = delta.vertices.size();
		delta.vertices.push_back(
			std::make_pair(vert_pos[i_vert], vert_normal[i_vert]));
	}
	for(const int i_face :
			boost::irange(0, (int)(tri_template.size() / 3))) {
		delta.triangles.push_back(std::make_tuple(
			compressed_vertex_index[tri_template[i_face * 3 + 0]],
			compressed_vertex_index[tri_template[i_face * 3 + 1]],
			compressed_vertex_index[tri_template[i_face * 3 + 2]]));
	}
	return delta;
}

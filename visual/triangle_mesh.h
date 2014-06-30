#pragma once

#include <algorithm>
#include <array>
#include <iostream>
#include <tuple>
#include <type_traits>
#include <vector>

#include <Eigen/Dense>

namespace visual {

template<typename Vertex>
class TriangleMesh {
public:
	// Merge given mesh into this by simple concatenation (in-place).
	// time(worst): O(|delta.vertices| + |delta.faces|)
	void merge(const TriangleMesh<Vertex>& delta) {
		const int base_index = vertices.size();
		for(const auto& face : delta.triangles) {
			triangles.emplace_back(
				std::get<0>(face) + base_index,
				std::get<1>(face) + base_index,
				std::get<2>(face) + base_index);
		}
		vertices.insert(vertices.end(),
			delta.vertices.begin(), delta.vertices.end());
	}

	// Transform vertex attributes without changing positions.
	/*
	template<typename Mapper>
	auto mapAttribute(Mapper f) const -> TriangleMesh<typename std::result_of<Mapper(const Vertex&)>::type> {
		TriangleMesh<typename std::result_of<Mapper(const Vertex&)>::type> mesh;
		mesh.triangles = triangles;
		mesh.vertices.resize(vertices.size());
		std::transform(vertices.begin(), vertices.end(), mesh.vertices.begin(),
				[&](const std::pair<Eigen::Vector3f, Vertex> vertex) {
					return std::make_pair(vertex.first, f(vertex.second));
				});
		return mesh;
	}
	*/

	/*
	template<typename Mapper>
	auto map(Mapper f) const -> TriangleMesh<typename std::result_of<Mapper(const Vertex&)>::type::> {
		TriangleMesh<typename std::result_of<Mapper(const Vertex&)>::type> mesh;
		mesh.triangles = triangles;
		mesh.vertices.resize(vertices.size());
		std::transform(vertices.begin(), vertices.end(), mesh.vertices.begin(), f);
		return mesh;
	}
	*/


	// Serialize this mesh in ASCII PLY format.
	void serializePLY(std::ostream& output) const {
		output << "ply" << std::endl;
		output << "format ascii 1.0" << std::endl;
		output << "element vertex " << vertices.size() << std::endl;
		output << "property float32 x" << std::endl;
		output << "property float32 y" << std::endl;
		output << "property float32 z" << std::endl;
		output << "element face " << triangles.size() << std::endl;
		output << "property list uint8 int32 vertex_indices" << std::endl;
		output << "end_header" << std::endl;

		for(const auto vertex : vertices) {
			output <<
				vertex.first(0) << " " <<
				vertex.first(1) << " " <<
				vertex.first(2) << std::endl;
		}
		for(const auto triangle : triangles) {
			output << "3 " <<
				std::get<0>(triangle) << " " <<
				std::get<1>(triangle) << " " <<
				std::get<2>(triangle) << std::endl;
		}
	}

	// Serialize this mesh in ASCII PLY format.
	// with UV coordinates.
	// condition: Vertex = Eigen::Vector2f
	void serializeObjWithUv(std::ostream& output, std::string mtllib) const {
		output << "mtllib " << mtllib << std::endl;
		for(const auto vertex : vertices) {
			output << "v " <<
				vertex.first(0) << " " <<
				vertex.first(1) << " " <<
				vertex.first(2) << std::endl;
		}

		for(const auto vertex : vertices) {
			output << "vt " <<
				vertex.second(0) << " " <<
				vertex.second(1) << std::endl;
		}

		output << "g " << "test_obj" << std::endl;
		output << "usemtl " << "obj_uv" << std::endl;
		for(const auto triangle : triangles) {
			output << "f " <<
				(1 + std::get<0>(triangle)) << "/" << (1 + std::get<0>(triangle)) << " " <<
				(1 + std::get<1>(triangle)) << "/" << (1 + std::get<1>(triangle)) << " " <<
				(1 + std::get<2>(triangle)) << "/" << (1 + std::get<2>(triangle)) << std::endl;
		}
	}
public:
	std::vector<std::tuple<int, int, int>> triangles;
	std::vector<std::pair<Eigen::Vector3f, Vertex>> vertices;
};

}  // namespace

#pragma once

#include <array>
#include <iostream>
#include <tuple>
#include <vector>

#include <Eigen/Dense>

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
public:
	std::vector<std::tuple<int, int, int>> triangles;
	std::vector<std::pair<Eigen::Vector3f, Vertex>> vertices;
};

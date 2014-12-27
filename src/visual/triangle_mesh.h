#pragma once

#include <algorithm>
#include <array>
#include <iostream>
#include <tuple>
#include <type_traits>
#include <vector>

#include <Eigen/Dense>

namespace recon {

template<typename Vertex>
class TriangleMesh {
public:
	// Merge given mesh into this by simple concatenation (in-place).
	// time(worst): O(|delta.vertices| + |delta.faces|)
	void merge(const TriangleMesh<Vertex>& delta) {
		const int base_index = vertices.size();
		for(const auto& face : delta.triangles) {
			triangles.push_back({{
				face[0] + base_index,
				face[1] + base_index,
				face[2] + base_index}});
		}
		vertices.insert(vertices.end(),
			delta.vertices.begin(), delta.vertices.end());
	}

	float area() const {
		float area = 0;
		for(const auto& face : triangles) {
			const auto v0 = vertices[face[0]].first;
			const auto v1 = vertices[face[1]].first;
			const auto v2 = vertices[face[2]].first;
			area += (v1 - v0).cross(v2 - v0).norm() * 0.5;
		}
		return area;
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

	void serializePLYWithRgb(std::ostream& output) const {
		output << "ply" << std::endl;
		output << "format ascii 1.0" << std::endl;
		output << "element vertex " << vertices.size() << std::endl;
		output << "property float32 x" << std::endl;
		output << "property float32 y" << std::endl;
		output << "property float32 z" << std::endl;
		output << "property uint8 red" << std::endl;
		output << "property uint8 green" << std::endl;
		output << "property uint8 blue" << std::endl;
		output << "element face " << triangles.size() << std::endl;
		output << "property list uint8 int32 vertex_indices" << std::endl;
		output << "end_header" << std::endl;

		for(const auto vertex : vertices) {
			output <<
				vertex.first(0) << " " <<
				vertex.first(1) << " " <<
				vertex.first(2) << " " <<
				vertex.second(0) << " " <<
				vertex.second(1) << " " <<
				vertex.second(2) << std::endl;
		}
		for(const auto triangle : triangles) {
			output << "3 " <<
				std::get<0>(triangle) << " " <<
				std::get<1>(triangle) << " " <<
				std::get<2>(triangle) << std::endl;
		}
	}

	void serializePLYWithRgbNormal(std::ostream& output) const {
		output << "ply" << std::endl;
		output << "format ascii 1.0" << std::endl;
		output << "element vertex " << vertices.size() << std::endl;
		output << "property float32 x" << std::endl;
		output << "property float32 y" << std::endl;
		output << "property float32 z" << std::endl;
		output << "property uint8 red" << std::endl;
		output << "property uint8 green" << std::endl;
		output << "property uint8 blue" << std::endl;
		output << "property float32 nx" << std::endl;
		output << "property float32 ny" << std::endl;
		output << "property float32 nz" << std::endl;
		output << "element face " << triangles.size() << std::endl;
		output << "property list uint8 int32 vertex_indices" << std::endl;
		output << "end_header" << std::endl;

		for(const auto vertex : vertices) {
			output <<
				vertex.first(0) << " " <<
				vertex.first(1) << " " <<
				vertex.first(2) << " " <<
				std::get<0>(vertex.second)(0) << " " <<
				std::get<0>(vertex.second)(1) << " " <<
				std::get<0>(vertex.second)(2) << " " <<
				std::get<1>(vertex.second)(0) << " " <<
				std::get<1>(vertex.second)(1) << " " <<
				std::get<1>(vertex.second)(2) << std::endl;
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
	void serializeObjWithUv(std::ostream& output,
			const std::string& mat_lib_path,
			const std::string& mat_name) const {
		output << "mtllib " << mat_lib_path << std::endl;
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
		output << "usemtl " << mat_name << std::endl;
		for(const auto triangle : triangles) {
			output << "f " <<
				(1 + std::get<0>(triangle)) << "/" << (1 + std::get<0>(triangle)) << " " <<
				(1 + std::get<1>(triangle)) << "/" << (1 + std::get<1>(triangle)) << " " <<
				(1 + std::get<2>(triangle)) << "/" << (1 + std::get<2>(triangle)) << std::endl;
		}
	}

	// condition: Vertex = std::pair<Eigen::Vector2f, Eigen::Vector3f>
	void serializeObjWithUvNormal(std::ostream& output,
			const std::string& mat_lib_path,
			const std::string& mat_name) const {
		output << "mtllib " << mat_lib_path << std::endl;
		for(const auto vertex : vertices) {
			output << "v " <<
				vertex.first(0) << " " <<
				vertex.first(1) << " " <<
				vertex.first(2) << std::endl;
		}

		for(const auto vertex : vertices) {
			output << "vt " <<
				vertex.second.first(0) << " " <<
				vertex.second.first(1) << std::endl;
		}

		for(const auto vertex : vertices) {
			output << "vn " <<
				vertex.second.second(0) << " " <<
				vertex.second.second(1) << " " <<
				vertex.second.second(2) << std::endl;
		}

		output << "g " << "test_obj" << std::endl;
		output << "usemtl " << mat_name << std::endl;
		for(const auto triangle : triangles) {
			output << "f " <<
				(1 + triangle[0]) << "/" << (1 + triangle[0]) << "/" << (1 + triangle[0]) << " " <<
				(1 + triangle[1]) << "/" << (1 + triangle[1]) << "/" << (1 + triangle[1]) << " " <<
				(1 + triangle[2]) << "/" << (1 + triangle[2]) << "/" << (1 + triangle[2]) << std::endl;
		}
	}
public:
	std::vector<std::array<int, 3>> triangles;
	std::vector<std::pair<Eigen::Vector3f, Vertex>> vertices;
};

// Replace any Vertex attribute with nullptr.
template<typename Vertex>
TriangleMesh<std::nullptr_t> dropAttrib(const TriangleMesh<Vertex>& mesh) {
	TriangleMesh<std::nullptr_t> new_mesh;
	new_mesh.triangles = mesh.triangles;
	for(const auto& vert : mesh.vertices) {
		new_mesh.vertices.push_back(std::make_pair(vert.first, nullptr));
	}
	return new_mesh;
}

// Flip triangles by reversing triangle rotation CCW <-> CW
template<typename Vertex>
TriangleMesh<Vertex> flipTriangles(const TriangleMesh<Vertex>& mesh) {
	TriangleMesh<Vertex> result;
	result.vertices = mesh.vertices;
	for(const auto& tri : mesh.triangles) {
		result.triangles.push_back({{
			tri[2], tri[1], tri[0]
		}});
	}
	return result;
}

template<typename TypeFirst, typename TypeSecond>
TriangleMesh<TypeSecond> mapSecond(const TriangleMesh<std::pair<TypeFirst, TypeSecond>>& mesh) {
	TriangleMesh<Eigen::Vector2f> mesh_snd;
	mesh_snd.triangles = mesh.triangles;
	mesh_snd.vertices.resize(mesh.vertices.size());
	std::transform(mesh.vertices.begin(), mesh.vertices.end(), mesh_snd.vertices.begin(),
		[](const std::pair<Eigen::Vector3f, std::pair<TypeFirst, TypeSecond>>& vertex) {
			return std::make_pair(vertex.first, vertex.second.second);
		});
	return mesh_snd;
}

// Calculate vertex normals assuming sharp edges.
TriangleMesh<std::pair<Eigen::Vector2f, Eigen::Vector3f>>
	assignNormal(const TriangleMesh<Eigen::Vector2f>& mesh);

// Create outward-facing cuboids.
TriangleMesh<std::nullptr_t> createBox(
		const Eigen::Vector3f& center,
		const Eigen::Vector3f& half_dx,
		const Eigen::Vector3f& half_dy,
		const Eigen::Vector3f& half_dz);

TriangleMesh<std::nullptr_t> createBox(
	const Eigen::Vector3f& center, float half_size);

TriangleMesh<std::nullptr_t> mergeCloseVertices(
	const TriangleMesh<std::nullptr_t>& mesh, float distance);

}  // namespace

#include "triangle_mesh.h"

#include <boost/range/irange.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>

namespace recon {

TriangleMesh<std::nullptr_t> createBox(
		const Eigen::Vector3f& center,
		const Eigen::Vector3f& half_dx,
		const Eigen::Vector3f& half_dy,
		const Eigen::Vector3f& half_dz) {
	TriangleMesh<std::nullptr_t> box;
	for(int i : boost::irange(0, 8)) {
		const Eigen::Vector3f vertex_pos = center +
			((i & 0b001) ? 1 : -1) * half_dx +
			((i & 0b010) ? 1 : -1) * half_dy +
			((i & 0b100) ? 1 : -1) * half_dz;
		box.vertices.push_back(std::make_pair(vertex_pos, nullptr));
	}

	// Create inward-facing triangles. (CCW is positive direction)
	// Draw a cube with 000-111 to understand this.
	box.triangles = {
		// X-
		{{0, 4, 2}},
		{{6, 2, 4}},
		// X+
		{{5, 1, 7}},
		{{3, 7, 1}},
		// Y-
		{{0, 1, 4}},
		{{5, 4, 1}},
		// Y+
		{{6, 7, 2}},
		{{3, 2, 7}},
		// Z-
		{{0, 2, 1}},
		{{3, 1, 2}},
		// Z+
		{{6, 4, 7}},
		{{5, 7, 4}}
	};

	return box;
}

TriangleMesh<std::pair<Eigen::Vector2f, Eigen::Vector3f>>
		assignNormal(const TriangleMesh<Eigen::Vector2f>& mesh) {
	TriangleMesh<std::pair<Eigen::Vector2f, Eigen::Vector3f>> result;
	int i_vert_offset = 0;
	for(const auto& tri : mesh.triangles) {
		const Eigen::Vector3f normal =
			(mesh.vertices[tri[1]].first - mesh.vertices[tri[0]].first).cross(
				mesh.vertices[tri[2]].first - mesh.vertices[tri[0]].first).normalized();
		assert(std::abs(normal.norm() - 1) < 1e-3);
		for(const int i_vert : boost::irange(0, 3)) {
			const auto& v = mesh.vertices[tri[i_vert]];
			result.vertices.emplace_back(
				v.first,
				std::make_pair(v.second, normal));
		}
		result.triangles.push_back({{
			i_vert_offset + 0,
			i_vert_offset + 1,
			i_vert_offset + 2
		}});
		i_vert_offset += 3;
	}
	return result;
}

TriangleMesh<std::nullptr_t> createBox(
		const Eigen::Vector3f& center, float half_size) {
	return createBox(center,
		Eigen::Vector3f::UnitX() * half_size,
		Eigen::Vector3f::UnitY() * half_size,
		Eigen::Vector3f::UnitZ() * half_size);
}

TriangleMesh<std::nullptr_t> mergeCloseVertices(
		const TriangleMesh<std::nullptr_t>& mesh, float distance) {
	assert(distance >= 0);
	// Create vertex collapse mapping.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
		new pcl::PointCloud<pcl::PointXYZ>);
	for(const auto& vert : mesh.vertices) {
		pcl::PointXYZ pt;
		pt.getVector3fMap() = vert.first;
		cloud->points.push_back(pt);
	}
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);

	std::map<int, int> ix_mapping;  // old ix -> new ix
	std::vector<std::pair<Eigen::Vector3f, std::nullptr_t>> new_vertices;
	for(const int vix : boost::irange(0, (int)mesh.vertices.size())) {
		// vix is a neighbor of a previously vertex.
		if(ix_mapping.find(vix) != ix_mapping.end()) {
			continue;
		}
		std::vector<int> result_ixs;
		std::vector<float> result_sq_dists;
		const int n_result = kdtree.radiusSearch(
			cloud->points[vix], distance,
			result_ixs, result_sq_dists);
		assert(n_result >= 1);  // query itself must be present
		const int new_vertex = new_vertices.size();
		new_vertices.push_back(mesh.vertices[vix]);
		for(const int result_ix : result_ixs) {
			ix_mapping[result_ix] = new_vertex;
		}
	}
	// all vertices must have mapping.
	assert(ix_mapping.size() == mesh.vertices.size());
	assert(new_vertices.size() <= mesh.vertices.size());

	TriangleMesh<std::nullptr_t> result;
	result.vertices = new_vertices;
	for(const auto& tri : mesh.triangles) {
		// only keep triangle if mapped vertices are unique.
		const int i0 = ix_mapping[tri[0]];
		const int i1 = ix_mapping[tri[1]];
		const int i2 = ix_mapping[tri[2]];
		if(i0 == i1 || i1 == i2 || i2 == i0) {
			continue;
		}
		result.triangles.push_back({{i0, i1, i2}});
	}
	assert(result.triangles.size() <= mesh.triangles.size());
	return result;
}

}  // namespace

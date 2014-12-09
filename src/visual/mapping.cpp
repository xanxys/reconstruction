#include "mapping.h"

#include <cmath>
#include <map>
#include <set>
#include <stack>
#include <stdexcept>

#include <geom/util.h>
#include <logging.h>
#include <math_util.h>

namespace recon {

Chart::Chart(
		const std::vector<std::array<int, 3>>& tris,
		const std::map<int, Eigen::Vector2f>& pre_uv) {
	// Create local vertices.
	std::map<int, int> external_to_local;
	for(const auto& vert : pre_uv) {
		external_to_local[vert.first] = vertices.size();
		vertices.push_back(vert);
	}
	assert(external_to_local.size() == vertices.size());
	assert(external_to_local.size() == pre_uv.size());
	// Map triangle vertices.
	for(const auto& tri : tris) {
		std::array<int, 3> tri_local;
		for(int i : boost::irange(0, 3)) {
			const auto it_v = external_to_local.find(tri[i]);
			if(it_v == external_to_local.cend()) {
				throw std::invalid_argument(
					"Chart: triangle contains unknown vertex id");
			}
			tri_local[i] = it_v->second;
		}
		triangles.push_back(tri_local);
	}
	assert(triangles.size() == tris.size());
	// Calculate AABB.
	uv_min = Eigen::Vector2f(1e3, 1e3);
	uv_max = Eigen::Vector2f(-1e3, -1e3);
	for(const auto& vert : vertices) {
		uv_min = uv_min.cwiseMin(vert.second);
		uv_max = uv_max.cwiseMax(vert.second);
	}
}

Eigen::Vector2f Chart::getSize() const {
	return uv_max - uv_min;
}

TriangleMesh<std::pair<int, Eigen::Vector2f>> Chart::getMesh() const {
	TriangleMesh<std::pair<int, Eigen::Vector2f>> mesh;
	for(const auto& vert : vertices) {
		mesh.vertices.emplace_back(
			Eigen::Vector3f::Zero(),
			std::make_pair(
				vert.first,
				vert.second - uv_min));
	}
	mesh.triangles = triangles;
	return mesh;
}


std::vector<Chart> divideMeshToCharts(
		const TriangleMesh<std::nullptr_t>& mesh) {
	INFO("Starting chart decomposition (#tri, #vert)",
		(int)mesh.triangles.size(), (int)mesh.vertices.size());
	std::set<int> all_tris;
	for(int i : boost::irange(0, (int)mesh.triangles.size())) {
		all_tris.insert(i);
	}
	const auto adjacency = getTriangleAdjacency(mesh);

	// Create planar adjacency by comparing triangle normals.
	std::vector<Eigen::Vector3f> tri_normals;
	for(const auto& tri : mesh.triangles) {
		const Eigen::Vector3f& v0 = mesh.vertices[tri[0]].first;
		const Eigen::Vector3f& v1 = mesh.vertices[tri[1]].first;
		const Eigen::Vector3f& v2 = mesh.vertices[tri[2]].first;
		tri_normals.push_back((v1 - v0).cross(v2 - v0).normalized());
	}
	const float cos_angle_thresh = std::cos(deg_to_rad(1e-3));
	std::map<int, std::set<int>> adj_planar;
	for(const int tri_ix : all_tris) {
		const auto it_adj = adjacency.find(tri_ix);
		if(it_adj == adjacency.cend()) {
			continue;
		}
		std::set<int> planar_neighbors;
		for(const int neighbor_tri : it_adj->second) {
			if(tri_normals[neighbor_tri].dot(tri_normals[tri_ix]) < cos_angle_thresh) {
				continue;
			}
			planar_neighbors.insert(neighbor_tri);
		}
		if(planar_neighbors.empty()) {
			continue;
		}
		adj_planar.emplace(tri_ix, std::move(planar_neighbors));
	}

	const auto planar_ccs = getCC(all_tris, adj_planar);
	INFO("chart decomposition (#tri, #cc)",
		(int)all_tris.size(), (int)planar_ccs.size());

	std::vector<Chart> charts;
	for(const auto& planar_cc : planar_ccs) {
		const auto basis = createOrthogonalBasis(
			tri_normals[*planar_cc.begin()]);
		// Project all vertices.
		std::vector<std::array<int, 3>> tris;
		std::map<int, Eigen::Vector2f> verts_proj;
		for(int tri_ix : planar_cc) {
			assert(0 <= tri_ix && tri_ix < mesh.triangles.size());
			tris.push_back(mesh.triangles[tri_ix]);
			for(int vert_ix : mesh.triangles[tri_ix]) {
				const auto& pos = mesh.vertices[vert_ix].first;
				verts_proj[vert_ix] = Eigen::Vector2f(
					basis.col(0).dot(pos),
					basis.col(1).dot(pos));
			}
		}
		charts.emplace_back(tris, verts_proj);
	}
	return charts;
}


std::map<int, std::set<int>> getTriangleAdjacency(
		const TriangleMesh<std::nullptr_t>& mesh) {
	const int n_tris = mesh.triangles.size();
	std::map<int, std::set<int>> vert_to_tris;
	for(const int ix_tri : boost::irange(0, n_tris)) {
		for(int ix_vert : mesh.triangles[ix_tri]) {
			vert_to_tris[ix_vert].insert(ix_tri);
		}
	}
	std::map<int, std::set<int>> adjacency;
	for(const int ix_tri : boost::irange(0, n_tris)) {
		const auto& tri = mesh.triangles[ix_tri];
		std::set<int> tris_neighbors;
		for(int ix_vert : tri) {
			tris_neighbors.insert(
				vert_to_tris[ix_vert].begin(),
				vert_to_tris[ix_vert].end());
		}
		tris_neighbors.erase(ix_tri);  // remove itself
		if(tris_neighbors.empty()) {
			continue;
		}
		adjacency.emplace(ix_tri, std::move(tris_neighbors));
	}
	return adjacency;
}


std::vector<std::set<int>> getCC(
		const std::set<int>& vertices,
		const std::map<int, std::set<int>>& adjacency) {
	std::vector<std::set<int>> ccs;
	std::set<int> remaining_verts = vertices;
	while(!remaining_verts.empty()) {
		const int seed = *remaining_verts.begin();
		// DFS to get all reachable vertices;
		std::set<int> visited;
		std::stack<int> frontier;
		frontier.push(seed);
		while(!frontier.empty()) {
			const int current = frontier.top();
			frontier.pop();
			if(visited.find(current) != visited.end()) {
				continue;
			}
			visited.insert(current);
			remaining_verts.erase(current);
			const auto children = adjacency.find(current);
			if(children == adjacency.cend()) {
				continue;
			}
			for(int tri : children->second) {
				frontier.push(tri);
			}
		}
		ccs.push_back(std::move(visited));
	}
	return ccs;
}

}  // namespace

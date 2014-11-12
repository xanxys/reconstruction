#include "mapping.h"

#include <map>
#include <set>
#include <stack>

#include <logging.h>
#include <math_util.h>

namespace visual {

std::vector<Chart> divideMeshToCharts(
		const TriangleMesh<std::nullptr_t>& mesh) {
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
	for(int tri_ix : all_tris) {
		std::set<int> planar_neighbors;
		for(int neighbor_tri : adjacency.find(tri_ix)->second) {
			if(tri_normals[neighbor_tri].dot(tri_normals[tri_ix]) < cos_angle_thresh) {
				continue;
			}
			planar_neighbors.insert(neighbor_tri);
		}
		adj_planar.emplace(tri_ix, std::move(planar_neighbors));
	}

	const auto planar_ccs = getCC(all_tris, adj_planar);
	INFO("chart decomposition (#tri, #cc)",
		(int)all_tris.size(), (int)planar_ccs.size());

	std::vector<Chart> charts;
	return charts;
}


std::map<int, std::set<int>> getTriangleAdjacency(
		const TriangleMesh<std::nullptr_t>& mesh) {
	std::map<int, std::set<int>> vert_to_tris;
	for(int ix_tri : boost::irange(0, (int)mesh.triangles.size())) {
		for(int ix_vert : mesh.triangles[ix_tri]) {
			vert_to_tris[ix_tri].insert(ix_vert);
		}
	}
	std::map<int, std::set<int>> adjacency;
	for(int ix_tri : boost::irange(0, (int)mesh.triangles.size())) {
		const auto& tri = mesh.triangles[ix_tri];
		std::set<int> tris_all;
		for(int ix_vert : tri) {
			tris_all.insert(
				vert_to_tris[ix_vert].begin(),
				vert_to_tris[ix_vert].end());
		}
		tris_all.erase(ix_tri);  // remove itself
		adjacency.emplace(ix_tri, std::move(tris_all));
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
			for(int tri : adjacency.find(current)->second) {
				frontier.push(tri);
			}
		}
		ccs.push_back(std::move(visited));
	}
	return ccs;
}



}  // namespace

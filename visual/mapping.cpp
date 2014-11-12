#include "mapping.h"

#include <map>
#include <set>
#include <stack>

namespace visual {

std::vector<Chart> divideConnectedMeshToCharts(
		const TriangleMesh<std::nullptr_t>& mesh,
		const std::vector<int>& tri_ixs) {
	std::vector<Chart> charts;
	return charts;
}

std::vector<std::set<int>> divideMeshToCC(
		const TriangleMesh<std::nullptr_t>& mesh) {
	// Create adjacency of triangles.
	std::map<int, std::set<int>> vert_to_tris;
	for(int ix_tri : boost::irange(0, (int)mesh.triangles.size())) {
		const auto& tri = mesh.triangles[ix_tri];
		vert_to_tris[ix_tri].insert(std::get<0>(tri));
		vert_to_tris[ix_tri].insert(std::get<1>(tri));
		vert_to_tris[ix_tri].insert(std::get<2>(tri));
	}
	std::map<int, std::set<int>> adjacency;
	for(int ix_tri : boost::irange(0, (int)mesh.triangles.size())) {
		const auto& tri = mesh.triangles[ix_tri];
		std::set<int> tris_all;
		tris_all.insert(
			vert_to_tris[std::get<0>(tri)].begin(),
			vert_to_tris[std::get<0>(tri)].end());
		tris_all.insert(
			vert_to_tris[std::get<1>(tri)].begin(),
			vert_to_tris[std::get<1>(tri)].end());
		tris_all.insert(
			vert_to_tris[std::get<2>(tri)].begin(),
			vert_to_tris[std::get<2>(tri)].end());
		tris_all.erase(ix_tri);  // remove itself
		adjacency.emplace(ix_tri, std::move(tris_all));
	}

	std::set<int> all_tris;
	for(int i : boost::irange(0, (int)mesh.triangles.size())) {
		all_tris.insert(i);
	}
	return getCC(all_tris, adjacency);
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

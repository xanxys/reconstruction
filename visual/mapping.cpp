#include "mapping.h"

#include <cmath>
#include <map>
#include <set>
#include <stack>
#include <stdexcept>

#include <logging.h>
#include <math_util.h>

namespace visual {

Eigen::Matrix3f createOrthogonalBasis(
		const Eigen::Vector3f& z) {
	// Choose an arbitrary unit vector
	// not too parallel to z, to increase stability.
	const auto seed = (std::abs(z.x()) > 0.5) ?
		Eigen::Vector3f(0, 1, 0) :
		Eigen::Vector3f(1, 0, 0);

	Eigen::Matrix3f basis;
	basis.col(0) = z.cross(seed).normalized();
	basis.col(1) = z.cross(basis.col(0));
	basis.col(2) = z;
	return basis;
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
	for(const auto& planar_cc : planar_ccs) {
		const auto basis = createOrthogonalBasis(
			tri_normals[*planar_cc.begin()]);

		basis.col(0);
		basis.col(1);
	}
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
		std::set<int> tris_neighbors;
		for(int ix_vert : tri) {
			tris_neighbors.insert(
				vert_to_tris[ix_vert].begin(),
				vert_to_tris[ix_vert].end());
		}
		tris_neighbors.erase(ix_tri);  // remove itself
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

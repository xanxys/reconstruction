#include "util.h"

#include <stack>

namespace recon {

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

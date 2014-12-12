#pragma once

#include <map>
#include <set>
#include <vector>

namespace recon {

// Get connected components of undirected graph.
std::vector<std::set<int>> getCC(
	const std::set<int>& vertices,
	const std::map<int, std::set<int>>& adjacency);

}  // namespace

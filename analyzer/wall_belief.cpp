#include "wall_belief.h"

#include <algorithm>
#include <set>


OrientedBox::OrientedBox(
	Eigen::Vector3f position,
	float ry,
	Eigen::Vector3f size,
	Eigen::Vector3f color,
	bool valid) :
	position(position), ry(ry), size(size), color(color), valid(valid) {
}

Eigen::Vector3f OrientedBox::getPosition() const {
	return position;
}

Eigen::Vector3f OrientedBox::getSize() const {
	return size;
}

Eigen::Vector3f OrientedBox::getColor() const {
	return color;
}

bool OrientedBox::getValid() const {
	return valid;
}

float OrientedBox::getRotationY() const {
	return ry;
}


WallBelief::WallBelief(const WallBelief& that) :
	log(that.log.str()), floor(that.floor), objects(that.objects) {
}

WallBelief::WallBelief(const FloorBelief& floor, int index) :
	floor(floor) {

	std::vector<VoxelIndex> voxels;
	for(auto& pair : floor.manhattan.getVoxelsDetailed()) {
		if(std::get<1>(pair.first) < floor.index && pair.second.state == VoxelState::OCCUPIED) {
			voxels.push_back(pair.first);
		}
	}

	// Split occupied voxels into connected components. (using 6-neighbor)
	std::vector<std::vector<VoxelIndex>> blobs = splitCC(voxels);
	log << blobs.size() << " voxel blobs found" << std::endl;
	for(const auto blob : blobs) {
		objects.push_back(guessBoxForBlob(floor, blob));
	}
}

OrientedBox WallBelief::guessBoxForBlob(const FloorBelief& floor, std::vector<VoxelIndex> blob) {
	assert(blob.size() > 0);

	Eigen::Vector3f accum(0, 0, 0);
	for(const VoxelIndex index : blob) {
		accum += indexToVector(index).cast<float>() * floor.manhattan.getVoxelSize();
	}
	const Eigen::Vector3f avg_pos = accum / blob.size();

	Eigen::Vector3f half_size(0, 0, 0);
	for(const VoxelIndex index : blob) {
		auto pos = indexToVector(index).cast<float>() * floor.manhattan.getVoxelSize();
		half_size = half_size.cwiseMax((pos - avg_pos).cwiseAbs());
	}

	return OrientedBox(
		avg_pos,
		0,
		half_size * 2,
		Eigen::Vector3f(100, 100, 200),
		true);
}

std::vector<std::vector<VoxelIndex>> WallBelief::splitCC(
	std::vector<VoxelIndex> nodes) {

	std::set<VoxelIndex> nodes_remaining(nodes.begin(), nodes.end());

	std::vector<std::vector<VoxelIndex>> ccs;
	while(!nodes_remaining.empty()) {
		// Do DFS to get all connected nodes.
		std::set<VoxelIndex> cc;
		std::vector<VoxelIndex> next;
		next.push_back(*nodes_remaining.begin());

		while(!next.empty()) {
			const VoxelIndex curr = next.back();
			next.pop_back();
			cc.insert(curr);

			// expand
			int x, y, z;
			std::tie(x, y, z) = curr;
			std::vector<VoxelIndex> neighbors = {
				std::make_tuple(x + 1, y, z),
				std::make_tuple(x - 1, y, z),
				std::make_tuple(x, y + 1, z),
				std::make_tuple(x, y - 1, z),
				std::make_tuple(x, y, z + 1),
				std::make_tuple(x, y, z - 1)
			};
			for(const VoxelIndex neighbor : neighbors) {
				if(cc.find(neighbor) == cc.end() &&
					nodes_remaining.find(neighbor) != nodes_remaining.end()) {
					next.push_back(neighbor);
				}
			}
		}

		// Subtract found CC and go on.
		nodes_remaining = set_difference(nodes_remaining, cc);
		ccs.push_back(std::vector<VoxelIndex>(cc.begin(), cc.end()));
	}

	return ccs;
}

std::vector<std::shared_ptr<WallBelief>> WallBelief::expand(const FloorBelief& floor) {
	std::vector<std::shared_ptr<WallBelief>> results;
	results.push_back(std::make_shared<WallBelief>(floor, 0));
	return results;
}

std::vector<OrientedBox> WallBelief::getObjects() const {
	return objects;
}

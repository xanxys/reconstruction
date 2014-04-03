#include "wall_belief.h"

WallBelief::WallBelief(const WallBelief& that) :
	log(that.log.str()), floor(that.floor) {
}

WallBelief::WallBelief(const FloorBelief& floor, int index) :
	floor(floor) {
}

std::vector<std::shared_ptr<WallBelief>> WallBelief::expand(const FloorBelief& floor) {
	std::vector<std::shared_ptr<WallBelief>> results;
	results.push_back(std::make_shared<WallBelief>(floor, 0));
	return results;
}

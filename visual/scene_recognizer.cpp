#include "scene_recognizer.h"

#include <boost/filesystem.hpp>
#include <boost/range/irange.hpp>

#include <visual/cloud_baker.h>

namespace visual {

// Use ear-clipping to triangulate.
// ear: a triangle formed by 3 adjacent vertices, in which
// 2 edges are part of boundary, and the other is part of inside.
//
// It's proven any simple polygon with N>=4 contains an ear.
std::vector<std::array<int, 3>> triangulatePolygon(const std::vector<Eigen::Vector2f>& points) {
	assert(points.size() >= 3);

	// O(1)
	auto cross2d = [](const Eigen::Vector2f& a, const Eigen::Vector2f& b) {
		return a(0) * b(1) - a(1) * b(0);
	};
	auto is_ear = [&](int pred, int curr, int succ) {
		// (pred-curr, succ-curr are boundary) is always true
		// since this is called when N>=4,
		// pred-succ cannot be boundary; it's either outside or inside.
		// pred-succ is inside
		// == curr is convex vertex
		// e.g.
		// pred
		//  |
		//  |  inside
		//  |-------
		// curr    succ

		// cross product = sin, positive when curr is convex.
		return cross2d(points[succ] - points[curr], points[pred] - points[curr]) > 0;
	};

	std::vector<int> indices;
	for(int i : boost::irange(0, (int)points.size())) {
		indices.push_back(i);
	}

	// Removing 1 ear = removing 1 vertex
	// O(N^2) (N+N-1+...)
	std::vector<std::array<int, 3>> tris;
	while(indices.size() > 3) {
		const int n = indices.size();
		// finding ear: O(N)
		for(int i : boost::irange(0, n)) {
			if(is_ear(indices[i], indices[(i + 1) % n], indices[(i + 2) % n])) {
				tris.push_back(std::array<int, 3>({
					indices[i], indices[(i + 1) % n], indices[(i + 2) % n]}));
				// It takes O(N) times to find an ear,
				// so don't care about deletion taking O(N) time.
				indices.erase(indices.begin() + ((i+1) % n));
				break;
			}
		}
	}

	assert(indices.size() == 3);
	tris.push_back(std::array<int, 3>({
		indices[0], indices[1], indices[2]}));
	return tris;
}

void SceneAssetBundle::serializeIntoDirectory(std::string dir_path) const {
	using boost::filesystem::create_directory;
	using boost::filesystem::path;

	create_directory(path(dir_path));
	exterior_mesh.writeWavefrontObject(
		(path(dir_path) / path("exterior_mesh")).string());
}


namespace scene_recognizer {

SceneAssetBundle recognizeScene(const std::vector<SingleScan>& scans) {
	assert(scans.size() > 0);

	SceneAssetBundle bundle;
	bundle.exterior_mesh = visual::CloudBaker(scans[0].old_style).generateRoomMesh();
	return bundle;
}

}  // namespace

}  // namespace

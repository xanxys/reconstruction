#include "nelder_mead.h"

#include <algorithm>

#include <boost/range/irange.hpp>

namespace recon {

// cf. http://sysplan.nams.kyushu-u.ac.jp/gen/edu/Algorithms/DownHillSimplexAlgorithm/simplex.pdf
std::pair<Eigen::VectorXf, float> minimize_nelder_mead(
		std::function<float(const Eigen::VectorXf&)> target,
		const Eigen::VectorXf& initial_param,
		int max_iter,
		const float size) {
	using Vertex = std::pair<Eigen::VectorXf, float>;

	const int n = initial_param.size();
	assert(n >= 1);

	const float alpha = 1.0;
	const float beta = 0.5;
	const float gamma = 2;

	// Intialize a mutable simplex. Each vertex (pos, value)
	std::vector<Vertex> vertices;
	vertices.reserve(n + 1);
	for(const int i : boost::irange(0, n + 1)) {
		Eigen::VectorXf pos = initial_param;
		if(i < n) {
			pos(i) += size;
		}
		vertices.emplace_back(pos, target(pos));
	}

	// used for sorting.
	std::vector<int> indices;
	for(const int i : boost::irange(0, n + 1)) {
		indices.push_back(i);
	}

	auto gen_vertex = [&target](const Eigen::VectorXf& pos) {
		return std::make_pair(pos, target(pos));
	};

	for(const int step : boost::irange(0, max_iter)) {
		// Choose worst 2 & best 1 elements.
		std::sort(indices.begin(), indices.end(), [&vertices](int ia, int ib) {
			return vertices[ia].second < vertices[ib].second;
		});
		const int i_max = indices[n];
		const int i_max2 = indices[n - 1];
		const int i_min = indices[0];

		Eigen::VectorXf center(n);
		center.setConstant(0);
		for(const int i : boost::irange(0, n + 1)) {
			if(i == i_max) {
				continue;
			}
			center += vertices[i].first;
		}
		center /= n;

		// Reflection:
		//       * max
		//
		// ------*------- hyperplane (all other vertices)
		//         center
		//     * reflected
		const auto refl = gen_vertex(
			(1 + alpha) * center - alpha * vertices[i_max].first);

		if(refl.second < vertices[i_min].second) {
			const auto expand = gen_vertex(
				gamma * refl.first + (1 - gamma) * center);
			if(expand.second < refl.second) {
				vertices[i_max] = expand;
			} else {
				vertices[i_max] = refl;
			}
		} else if(vertices[i_max2].second < refl.second && refl.second < vertices[i_max].second) {
			const auto contract = gen_vertex(
				beta * vertices[i_max].first + (1 - beta) * center);
			if(contract.second < vertices[i_max].second) {
				vertices[i_max] = contract;
			} else {
				for(const int i : boost::irange(0, n + 1)) {
					if(i == i_min) {
						continue;
					}
					vertices[i] = gen_vertex(
						(vertices[i_min].first + vertices[i].first) * 0.5);
				}
			}
		} else if(vertices[i_max].second <= refl.second) {
			const auto contract = gen_vertex(
				beta * vertices[i_max].first + (1 - beta) * center);
			if(contract.second < vertices[i_max].second) {
				vertices[i_max] = contract;
			} else {
				for(const int i : boost::irange(0, n + 1)) {
					if(i == i_min) {
						continue;
					}
					vertices[i] = gen_vertex(
						(vertices[i_min].first + vertices[i].first) * 0.5);
				}
			}
		} else {
			vertices[i_max] = refl;
		}
	}

	// Return minimum vertex;
	return *std::min_element(vertices.begin(), vertices.end(),
		[](const Vertex& va, const Vertex& vb) {
		return va.second < vb.second;
	});
}

}  // namespace

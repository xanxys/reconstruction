#pragma once

#include <memory>
#include <tuple>
#include <vector>

#include <boost/range/irange.hpp>
#include <Eigen/Dense>

#include <range3.h>
#include <visual/dense_voxel.h>

namespace visual {

template<typename VertexAttrib>
DenseVoxel<bool> meshToVoxel(
	const TriangleMesh<VertexAttrib>& mesh, Eigen::Vector3f offset, float scale, Eigen::Vector3i shape) {
	DenseVoxel<bool> voxel(shape);

	for(const auto& tri : mesh.triangles) {
		const auto& p0 = mesh.vertices[std::get<0>(tri)].first;
		const auto dp1 = mesh.vertices[std::get<1>(tri)].first - p0;
		const auto dp2 = mesh.vertices[std::get<2>(tri)].first - p0;
		const int n1 = 1 + static_cast<int>(dp1.norm() / scale);
		const int n2 = 1 + static_cast<int>(dp2.norm() / scale);

		for(int i1 : boost::irange(0, n1)) {
			for(int i2 : boost::irange(0, n2)) {
				const float t1 = i1 / n1;
				const float t2 = i2 / n2;
				if(t1 + t2 > 1) {
					break;
				}

				const Eigen::Vector3f p = p0 + dp1 * t1 + dp2 * t2;
				const Eigen::Vector3i p_i = ((p - offset) / scale).cast<int>();
				if((p_i.array() >= Eigen::Vector3i::Zero().array()).all() &&
					(p_i.array() < shape.array()).all()) {
					voxel[p_i] = true;
				}
			}
		}
	}
	return voxel;
}

}  // namespace

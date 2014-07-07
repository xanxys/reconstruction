#pragma once

#include <memory>
#include <vector>

#include <Eigen/Dense>

#include <range3.h>

namespace visual {

template<typename Cell>
class DenseVoxel {
public:
	DenseVoxel(Eigen::Vector3i size) :
		offset(0),
		strides(1, size(0), size(0) * size(1)),
		data(new std::vector<Cell>(size(0) * size(1) * size(2))),
		bound(size) {
	}

	typename std::vector<Cell>::reference operator[](const Eigen::Vector3i& index) {
		return (*data)[offset + strides.dot(index)];
	}
	const typename std::vector<Cell>::reference operator[](const Eigen::Vector3i& index) const {
		return (*data)[offset + strides.dot(index)];
	}


	Eigen::Vector3i shape() const {
		return bound;
	}
private:
	// Numpy-like addressing.
	// ix_1d = offset + stride `dot` ix_3d
	Eigen::Vector3i bound;
	int offset;
	Eigen::Vector3i strides;
	std::shared_ptr<std::vector<Cell>> data;
};

}  // namespace

#include "voxel_conversion.h"

namespace visual {
namespace voxel_conversion {

TriangleMesh<std::nullptr_t> extractVoxelSurface(
		const DenseVoxel<bool>& dv,
		const Eigen::Vector3i& imin,
		const float voxel_size) {

	TriangleMesh<std::nullptr_t> mesh;
	for(const auto& i : range3(dv.shape())) {
		const bool inside =
			(Eigen::Vector3i::Zero().array() < i.array()).all() &&
			(i.array() < (dv.shape() - Eigen::Vector3i(1, 1, 1)).array()).all();

		// Don't output non-surface-facing cubes.
		// TODO: ideally, we should operate at quads level, not cube level.
		bool can_skip = true;
		if(inside) {
			for(int axis : boost::irange(0, 3)) {
				Eigen::Vector3i delta = Eigen::Vector3i::Zero();
				delta(axis) += 1;

				if(dv[i + delta] != dv[i]) {
					can_skip = false;
					break;
				}
				if(dv[i - delta] != dv[i]) {
					can_skip = false;
					break;
				}
			}
		} else {
			can_skip = false;
		}

		if(!can_skip && dv[i]) {
			const Eigen::Vector3f pos = (imin + i).cast<float>() * voxel_size;
			mesh.merge(triangle_mesh::createBox(pos + Eigen::Vector3f::Ones() * (0.5 * voxel_size), voxel_size));
		}
	}
	return mesh;
}

}  // namespace
}  // namespace

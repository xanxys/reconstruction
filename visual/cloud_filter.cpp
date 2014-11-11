#include "cloud_filter.h"

#include <map>
#include <tuple>

#include <boost/optional.hpp>
#include <Eigen/Dense>
#include <pcl/kdtree/kdtree_flann.h>

#include <logging.h>
#include <math_util.h>

namespace visual {
namespace cloud_filter {

pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr squashRegistrationError(
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input,
		float error_radius) {
	assert(error_radius > 0);

	DEBUG("squash:begin");

	// Create uniform grid for fixed-radius NN query.
	const float ortho_error_radius = error_radius * 0.3;
	std::map<
		std::tuple<int, int, int>,
		std::vector<pcl::PointXYZRGBNormal>> buckets;
	for(const auto& pt : input->points) {
		const auto fi = pt.getVector3fMap() / error_radius;
		const auto ix = std::make_tuple(
			(int)std::floor(fi(0)),
			(int)std::floor(fi(1)),
			(int)std::floor(fi(2)));
		buckets[ix].push_back(pt);
	}

	DEBUG("squash:proc begin");

	// The method is:
	// for all point:
	//   take cyrindrical neighbors (pointing normal direction)
	//   and average them.
	// Process bucket with 3^3 - 1 neighboring buckets.
	const float cos_thresh = std::cos(pi / 8);
	auto result = pcl::PointCloud<pcl::PointXYZRGBNormal>().makeShared();
	for(const auto& bucket : buckets) {
		int x, y, z;
		std::tie(x, y, z) = bucket.first;
		// Collect all points in neighboring buckets (+itself).
		std::vector<pcl::PointXYZRGBNormal> points_nn;
		for(int dx = -1; dx < 2; dx++) {
			for(int dy = -1; dy < 2; dy++) {
				for(int dz = -1; dz < 2; dz++) {
					const auto it = buckets.find(std::make_tuple(
						x + dx, y + dy, z + dz));
					if(it == buckets.end()) {
						continue;
					}
					points_nn.insert(
						points_nn.end(),
						it->second.begin(),
						it->second.end());
				}
			}
		}
		// Process each point in center bucket.
		for(const auto& pt : bucket.second) {
			int accum = 0;
			Eigen::Vector3f pos_accum = Eigen::Vector3f::Zero();
			Eigen::Vector3f color_accum = Eigen::Vector3f::Zero();
			Eigen::Vector3f normal_accum = Eigen::Vector3f::Zero();
			for(const auto& pt_target : points_nn) {
				// Check normal alignment.
				const float cos_normals = pt_target.getNormalVector3fMap().dot(
					pt.getNormalVector3fMap());
				if(cos_normals < cos_thresh) {
					continue;
				}
				// Check if pt_target is in cylinder(pt).
				const auto delta = pt_target.getVector3fMap() - pt.getVector3fMap();
				const float d_proj = delta.dot(pt.getNormalVector3fMap());
				if(std::abs(d_proj) > error_radius) {
					continue;
				}
				const auto d_perp = delta - d_proj * pt.getNormalVector3fMap();
				if(d_perp.norm() > ortho_error_radius) {
					continue;
				}
				// Accumulate.
				accum++;
				pos_accum += pt_target.getVector3fMap();
				color_accum += Eigen::Vector3f(pt_target.r, pt_target.g, pt_target.b);
				normal_accum += pt_target.getNormalVector3fMap();
			}
			if(accum <= 1) {
				result->points.push_back(pt);
			} else {
				pcl::PointXYZRGBNormal pt_avg;
				pt_avg.getVector3fMap() = pos_accum / accum;
				pt_avg.getNormalVector3fMap() = (normal_accum / accum).normalized();
				pt_avg.r = color_accum(0) / accum;
				pt_avg.g = color_accum(1) / accum;
				pt_avg.b = color_accum(2) / accum;
				result->points.push_back(pt_avg);
			}
		}
	}

	DEBUG("squash:end");
	return result;
}

/*
def squash_reg_error_3d(pts, error_radius=0.2):
	kdt = scipy.spatial.cKDTree(pts[:, :3])

	cos_thresh = math.cos(math.pi / 8)
	new_pts = []
	for p in pts:
		ixs = kdt.query_ball_point(p[:3], error_radius)
		if len(ixs) <= 1:
			new_pts.append(p)
		else:
			pts_subset = pts[ixs]
			cos = np.dot(pts_subset[:, 6:9], p[6:9])
			delta = pts_subset[:, :3] - p[:3]
			proj_v = np.outer(
				np.dot(delta, p[6:9]),
				p[6:9])
			perp_v = delta - proj_v
			perp = (perp_v ** 2).sum(axis=1) ** 0.5

			pts_subset = pts_subset[(cos > cos_thresh) & (perp < ortho_error_radius)]
			if len(pts_subset) <= 1:
				new_pts.append(p)
			else:
				new_pts.append(pts_subset.mean(axis=0))
	return np.array(new_pts)
*/

}  // namespace
}  // namespace

#include "shape_fitter.h"

#include <algorithm>
#include <complex>
#include <numeric>
#include <set>
#include <stdexcept>

#include <fstream>

#include <boost/math/constants/constants.hpp>
#include <boost/range/irange.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/point_types.h>

#include "logging.h"

namespace visual {
namespace shape_fitter {

TriangleMesh<std::nullptr_t> fitExtrusion(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	TriangleMesh<std::nullptr_t> mesh;

	const auto poly = extractPolygon2D(cloud);
	const auto h_range = extractHeightRange(cloud);

	// Create wall
	for(int i : boost::irange(0, (int)poly.size())) {
		const auto xy0 = poly[(i + 1) % poly.size()];
		const auto xy1 = poly[i];

		// Looked from inside, vertices are laid out like this:
		// 3----2  -- h_range.second
		// | \  |
		// 0----1  -- h_range.first
		// xy0 xy1
		// i+1 <- i
		TriangleMesh<std::nullptr_t> quad;
		quad.vertices.emplace_back(Eigen::Vector3f(xy0.x(), xy0.y(), h_range.first), nullptr);
		quad.vertices.emplace_back(Eigen::Vector3f(xy1.x(), xy1.y(), h_range.first), nullptr);
		quad.vertices.emplace_back(Eigen::Vector3f(xy1.x(), xy1.y(), h_range.second), nullptr);
		quad.vertices.emplace_back(Eigen::Vector3f(xy0.x(), xy0.y(), h_range.second), nullptr);

		quad.triangles.push_back(std::make_tuple(0, 1, 3));
		quad.triangles.push_back(std::make_tuple(2, 3, 1));

		mesh.merge(quad);
	}

	// Create floor + ceiling.
	// floor & ceiling are both inward facing, so
	// when projected onto XY plane,
	// floor looks CCW (identical to tris), ceiling CW (flipped).
	const auto tris = triangulatePolygon(poly);
	auto create_cap = [&](const float z, const bool flip) {
		TriangleMesh<std::nullptr_t> cap;
		for(const auto& pt2d : poly) {
			const Eigen::Vector3f pt3d(pt2d(0), pt2d(1), z);
			cap.vertices.push_back(std::make_pair(pt3d, nullptr));
		}
		for(const auto& tri : tris) {
			cap.triangles.push_back(flip ?
				std::make_tuple(tri[2], tri[1], tri[0]) :
				std::make_tuple(tri[0], tri[1], tri[2]));
		}
		return cap;
	};

	mesh.merge(create_cap(h_range.first, false));  // floor
	mesh.merge(create_cap(h_range.second, true));  // ceiling

	return mesh;
}

TriangleMesh<std::nullptr_t> fitOBB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	const float angle_step = 0.05;

	// We call two horizontal axes in OBB (s,t) to avoid confusion.
	// when angle==0, (x,y,z) == (t,y,s)

	// Find smallest 2D OBB (st projection).
	INFO("Finding Y-rotation angle");
	float best_area = 1e3;
	float best_angle = -1;
	std::pair<float, float> best_s_range, best_t_range;
	for(float angle = 0; angle < boost::math::constants::pi<float>() / 2; angle += angle_step) {
		// Create z and x histogram of rotated cloud.
		std::vector<float> ss, ts;
		Eigen::Matrix2f rot_world_to_local = Eigen::Rotation2D<float>(angle).matrix();
		for(auto it = cloud->points.cbegin(); it != cloud->points.cend(); it++) {
			const Eigen::Vector2f pt_zx(it->z, it->x);
			const auto pt_st = rot_world_to_local * pt_zx;
			ss.push_back(pt_st(0));
			ts.push_back(pt_st(1));
		}

		// Create local AABB.
		const auto s_range = robustMinMax(ss);
		const auto t_range = robustMinMax(ts);
		const float area =
			(std::get<1>(s_range) - std::get<0>(s_range)) *
			(std::get<1>(t_range) - std::get<0>(t_range));

		DEBUG("@angle", angle, " area=", area);

		if(area < best_area) {
			best_area = area;
			best_angle = angle;
			best_s_range = s_range;
			best_t_range = t_range;
		}
	}
	assert(best_angle >= 0);

	// Find Y range (1D AABB).
	std::vector<float> ys;
	for(auto it = cloud->points.cbegin(); it != cloud->points.cend(); it++) {
		ys.push_back(it->y);
	}
	const auto y_range = robustMinMax(ys);

	// Construct 6 faces of the best OBB.
	const Eigen::Matrix2f local_to_world = Eigen::Rotation2D<float>(-best_angle).matrix();
	const auto splus_zx = local_to_world * Eigen::Vector2f(1, 0);
	const auto tplus_zx = local_to_world * Eigen::Vector2f(0, 1);

	Eigen::Vector3f splus(splus_zx(1), 0, splus_zx(0));
	Eigen::Vector3f yplus(0, 1, 0);
	Eigen::Vector3f tplus(tplus_zx(1), 0, tplus_zx(0));

	// Create box from Y,S,T axes.
	return createBox(
		tplus * mean(best_t_range) +
		yplus * mean(y_range) +
		splus * mean(best_s_range),
		tplus * half(best_t_range),
		yplus * half(y_range),
		splus * half(best_s_range));
}

float mean(const std::pair<float, float>& pair) {
	return (pair.first + pair.second) / 2;
}

float half(const std::pair<float, float>& pair) {
	return (pair.second - pair.first) / 2;
}

std::pair<float, float> robustMinMax(std::vector<float>& values) {
	std::sort(values.begin(), values.end());
	return std::make_pair(
		values[int(values.size() * 0.01)],
		values[int(values.size() * 0.99)]);
}

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

std::vector<Eigen::Vector2f> extractPolygon2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	if(cloud->points.size() < 1000) {
		WARN("Number of points might be too small for polygon extraction", (int)cloud->points.size());
	}

	// Squash Z-axis.
	std::vector<Eigen::Vector2f> points;
	for(const auto& pt : cloud->points) {
		points.emplace_back(pt.x, pt.y);
	}

	// downsample by grid filter.
	const float resolution = 0.1;
	std::map<std::pair<int, int>, std::vector<Eigen::Vector2f>> tiles;
	for(const auto& pt : points) {
		const auto pt_i = (pt / resolution).cast<int>();
		tiles[std::make_pair(pt_i(0), pt_i(1))].push_back(pt);
	}
	std::vector<Eigen::Vector2f> points_downsampled;
	for(const auto& tile : tiles) {
		points_downsampled.push_back(
			std::accumulate(
				tile.second.begin(), tile.second.end(),
				Eigen::Vector2f(0, 0)) / tile.second.size());
	}

	return calculateConcaveHull(points_downsampled, 20);
}

std::pair<float, float> extractHeightRange(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	std::vector<float> zs;
	for(const auto& pt : cloud->points) {
		zs.push_back(pt.z);
	}
	return robustMinMax(zs);
}

bool intersectSegments(
	std::pair<Eigen::Vector2f, Eigen::Vector2f> a,
	std::pair<Eigen::Vector2f, Eigen::Vector2f> b) {
	const Eigen::Vector2f da = a.second - a.first;
	const Eigen::Vector2f db = b.second - b.first;

	// Solve (ta, tb) in a0 + ta * da = b0 + tb * db
	// (da, -db) (ta, tb)^T = b0 - a0
	Eigen::Matrix2f m;
	m.col(0) = da;
	m.col(1) = -db;
	const Eigen::Vector2f v = b.first - a.first;

	const auto dec = m.colPivHouseholderQr();

	if(dec.rank() < 2) {
		// lines are parallel or some other degenerate case
		return false;
	} else {
		const Eigen::Vector2f ts = m.inverse() * v;
		return (0 < ts(0) && ts(0) < 1) && (0 < ts(1) && ts(1) < 1);
	}
}

std::vector<Eigen::Vector2f> calculateConcaveHull(
	const std::vector<Eigen::Vector2f>& points, int k_min) {
	if(points.size() < 3) {
		throw std::runtime_error("Points are too few to form a polygon");
	}

	// Return k-nearest neighbors in nearest-first order.
	// TODO: rewrite with a kd-tree if it's too slow.
	auto query_nearest = [&](const Eigen::Vector2f& query, int k) {
		std::vector<int> result;
		for(int i : boost::irange(0, (int)points.size())) {
			result.push_back(i);
		}
		std::sort(result.begin(), result.end(), [&](int ix_a, int ix_b) {
			return (points[ix_a] - query).norm() < (points[ix_b] - query).norm();
		});
		result.resize(std::min(k, static_cast<int>(points.size())));
		return result;
	};

	// Select the point with minimum Y.
	const int start_ix = std::distance(points.begin(), std::min_element(points.begin(), points.end(),
		[](const Eigen::Vector2f& a, const Eigen::Vector2f& b) {
			return a(1) < b(1);
		}));
	const Eigen::Vector2f start = points[start_ix];
	std::complex<float> prev_angle = -1;

	std::vector<Eigen::Vector2f> circle = {start};
	std::set<int> circle_s = {start_ix};
	while(true) {
		const auto current = circle.back();

		auto intersecting = [&](int i) {
			const auto new_segment = std::make_pair(current, points[i]);
			// irange(0, size() - 1): exclude prev-current segment, because
			// 1. it's guaranteed not intersect (they share a single point)
			// 2. sometimes lead to false intersection when included
			auto segs = boost::irange(0, (int)circle.size() - 1);
			return std::any_of(segs.begin(), segs.end(), [&](int i) {
				return intersectSegments(new_segment,
					std::make_pair(circle[i], circle[i + 1]));
			});
		};

		//  Return CCW angle from prev_angle.
		auto calc_angle = [&](int i) {
			const float pi = 3.14159265359;
			auto dp = points[i] - current;
			const std::complex<float> next_angle = std::complex<float>(dp(0), dp(1));
			float delta_angle = std::arg(next_angle / prev_angle);
			// If we use 0 here, there'll be almost degenerate case
			// of turning back to previous point
			if(delta_angle > pi * 0.1) {
				delta_angle -= 2 * pi;
			}
			return delta_angle;
		};

		int k = k_min;
		int best_ix;
		while(true) {
			// Query k + 1 and drop the first (= query itself) point.
			const auto ixs_and_self = query_nearest(current, k + 1);
			auto ixs = std::vector<int>(ixs_and_self.begin() + 1, ixs_and_self.end());

			// Prefer the point with largest CW rotation.
			std::sort(ixs.begin(), ixs.end(), [&](int a, int b) {
				return calc_angle(a) < calc_angle(b);
			});

			std::vector<int> candidates;
			for(int ix : ixs) {
				// Reject already added points (start is ok, though)
				if(ix != start_ix && circle_s.count(ix) > 0) {
					continue;
				}
				// Reject self-intersecting points.
				if(intersecting(ix)) {
					continue;
				}
				candidates.push_back(ix);
			}

			// retry with larger k if there's no valid candidate
			if(candidates.size() == 0) {
				k = 1 + static_cast<int>(k * 1.5);
				if(k >= points.size()) {
					throw std::runtime_error("Circle search exhausted; probably some bug");
				}
			}
			else {
				best_ix = candidates[0];
				break;
			}
		}

		// Succesfully returned to the start.
		if(best_ix == start_ix) {
			break;
		}
		// Non-start loop found.
		if(circle_s.find(best_ix) != circle_s.end()) {
			throw std::runtime_error(
				"Unexpected loop found in concanve hull search");
		}

		// Prepare for the next jump
		const auto dp = current - points[best_ix];
		prev_angle = std::complex<float>(dp(0), dp(1));
		circle.push_back(points[best_ix]);
		circle_s.insert(best_ix);
	}

	return circle;
}

TriangleMesh<std::nullptr_t> createBox(
		Eigen::Vector3f center,Eigen::Vector3f half_dx,
		Eigen::Vector3f half_dy, Eigen::Vector3f half_dz) {
	TriangleMesh<std::nullptr_t> box;
	for(int i : boost::irange(0, 8)) {
		const Eigen::Vector3f vertex_pos = center +
			((i & 0b001) ? 1 : -1) * half_dx +
			((i & 0b010) ? 1 : -1) * half_dy +
			((i & 0b100) ? 1 : -1) * half_dz;
		box.vertices.push_back(std::make_pair(vertex_pos, nullptr));
	}

	// Create inward-facing triangles. (CCW is positive direction)
	// Draw a cube with 000-111 to understand this.
	box.triangles = {
		// X-
		std::make_tuple(0, 2, 4),
		std::make_tuple(6, 4, 2),
		// X+
		std::make_tuple(5, 7, 1),
		std::make_tuple(3, 1, 7),
		// Y-
		std::make_tuple(0, 4, 1),
		std::make_tuple(5, 1, 4),
		// Y+
		std::make_tuple(6, 2, 7),
		std::make_tuple(3, 7, 2),
		// Z-
		std::make_tuple(0, 1, 2),
		std::make_tuple(3, 2, 1),
		// Z+
		std::make_tuple(6, 7, 4),
		std::make_tuple(5, 4, 7)
	};

	return box;
}

}  // namespace
}  // namespace

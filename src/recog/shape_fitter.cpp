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

#include <geom/triangulation.h>
#include <logging.h>
#include <math_util.h>

namespace recon {

std::tuple<
	std::vector<Eigen::Vector2f>,
	std::pair<float, float>>
		fitExtrudedPolygon(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	const auto poly = extractPolygon2D(cloud);
	const auto h_range = extractHeightRange(cloud);
	return std::make_tuple(poly, h_range);
}

ExtrudedPolygonMesh::ExtrudedPolygonMesh(
		const std::vector<Eigen::Vector2f>& poly,
		const std::pair<float, float>& h_range) {
	assert(poly.size() >= 3);
	// Create all vertices, as composition of CCW top ring
	// and bottom ring.
	std::vector<int> verts_bottom;
	std::vector<int> verts_top;
	for(const auto& xy : poly) {
		verts_bottom.push_back(mesh.vertices.size());
		mesh.vertices.emplace_back(
			Eigen::Vector3f(xy.x(), xy.y(), h_range.first),
			nullptr);
		verts_top.push_back(mesh.vertices.size());
		mesh.vertices.emplace_back(
			Eigen::Vector3f(xy.x(), xy.y(), h_range.second),
			nullptr);
	}

	// Append wall quads.
	for(int i : boost::irange(0, (int)poly.size())) {
		// Looked from inside, vertices are laid out like this:
		// o----o  -- verts_top
		// | \  |
		// o----o  -- verts_bottom
		// j <- i
		const int j = (i + 1) % poly.size();

		mesh.triangles.push_back({
			verts_bottom[i], verts_top[i], verts_top[j]});
		mesh.triangles.push_back({
			verts_bottom[j], verts_bottom[i], verts_top[j]});
	}

	// Create floor + ceiling.
	// floor & ceiling are both inward facing, so
	// when projected onto XY plane,
	// floor looks CCW (identical to tris), ceiling CW (flipped).
	DEBUG("Triangulating");
	const auto tris = triangulatePolygon(poly);
	auto append_cap = [&](const bool is_ceiling) {
		std::vector<int> tri_ixs;
		for(const auto& tri : tris) {
			tri_ixs.push_back(mesh.triangles.size());
			if(is_ceiling) {
				mesh.triangles.push_back({{
					verts_top[tri[2]],
					verts_top[tri[1]],
					verts_top[tri[0]]}});
			} else {
				mesh.triangles.push_back({{
					verts_bottom[tri[0]],
					verts_bottom[tri[1]],
					verts_bottom[tri[2]]}});
			}
		}
		return tri_ixs;
	};

	floor_tri_ixs = append_cap(false);
	ceiling_tri_ixs = append_cap(true);
	DEBUG("Extruded polygon mesh #v=", (int)mesh.vertices.size());

	floor_poly_verts = verts_bottom;
}


std::vector<int> ExtrudedPolygonMesh::getFloorPolygonVertices() const {
	return floor_poly_verts;
}


const TriangleMesh<std::nullptr_t>& ExtrudedPolygonMesh::getMesh() const {
	return mesh;
}


float mean(const std::pair<float, float>& pair) {
	return (pair.first + pair.second) / 2;
}

float half(const std::pair<float, float>& pair) {
	return (pair.second - pair.first) / 2;
}

std::pair<float, float> robustMinMax(std::vector<float>& values, const float tile) {
	assert(0 <= tile && tile <= 1);
	std::sort(values.begin(), values.end());
	return std::make_pair(
		values[int(values.size() * tile)],
		values[int(values.size() * (1 - tile))]);
}

bool isPolygonCCW(const std::vector<Eigen::Vector2f>& points) {
	auto cross2d = [](const Eigen::Vector2f& a, const Eigen::Vector2f& b) {
		return a(0) * b(1) - a(1) * b(0);
	};
	// Use Shoelace formula (Green's thereom) to calculate signed area.
	// cf. https://en.wikipedia.org/wiki/Shoelace_formula
	const int n = points.size();
	float area = 0;
	for(int i : boost::irange(0, n)) {
		area += cross2d(points[i], points[(i + 1) % n]);
	}
	area /= 2;

	return area > 0;
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
		const auto pt_i = pt / resolution;
		tiles[std::make_pair(
			(int)std::floor(pt_i(0)),
			(int)std::floor(pt_i(1)))].push_back(pt);
	}
	const int n_thresh = 0.2 * points.size() / tiles.size();
	INFO("2D downsampler: rejection thresh", n_thresh);
	int n_reject = 0;
	std::vector<Eigen::Vector2f> points_downsampled;
	for(const auto& tile : tiles) {
		// Ignore tile with too few points.
		if(tile.second.size() < n_thresh) {
			n_reject++;
			continue;
		}
		points_downsampled.push_back(
			std::accumulate(
				tile.second.begin(), tile.second.end(),
				Eigen::Vector2f(0, 0)) / tile.second.size());
	}
	INFO("Rejected tiles:", n_reject, "of", (int)tiles.size());

	return calculateConcaveHull(points_downsampled, 20);
}

std::pair<float, float> extractHeightRange(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	std::vector<float> zs;
	for(const auto& pt : cloud->points) {
		zs.push_back(pt.z);
	}
	return robustMinMax(zs, 0.001);
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

	const Eigen::Vector2f ts = m.inverse() * v;
	return (0 < ts(0) && ts(0) < 1) && (0 < ts(1) && ts(1) < 1);
}

std::vector<Eigen::Vector2f> calculateConcaveHull(
	const std::vector<Eigen::Vector2f>& points, const int k_min) {
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
			// irange(0, size() - 2): exclude prev-current segment, because
			// 1. it's guaranteed not intersect (they share a single point)
			// 2. sometimes lead to false intersection when included
			auto segs = boost::irange(0, std::max(0, (int)circle.size() - 2));
			return std::any_of(segs.begin(), segs.end(), [&](int i) {
				return intersectSegments(new_segment,
					std::make_pair(circle[i], circle[i + 1]));
			});
		};

		//  Return CCW angle from prev_angle.
		auto calc_angle = [&](int i) {
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
			if(points[ixs[0]] == current) {
				throw std::runtime_error("More than one points found at identical position. Cannot create concave hull.");
			}

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
				DEBUG("k(next)=", k);
				if(k >= points.size()) {
					DEBUG("circle.size", (int)circle.size());
					WARN("Circle search exhausted; probably some bug; exiting prematurely");
					return circle;
					//throw std::runtime_error("Circle search exhausted; probably some bug");
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
		if(circle_s.count(best_ix) > 0) {
			throw std::runtime_error(
				"Unexpected loop found in concanve hull search");
		}

		// Prepare for the next jump
		const auto dp = current - points[best_ix];
		prev_angle = std::complex<float>(dp(0), dp(1));
		circle.push_back(points[best_ix]);
		circle_s.insert(best_ix);
	}
	INFO("Concave hull completed with #vertices", (int)circle.size());
	assert(isSaneSimplePolygon(circle));
	return circle;
}


bool isSaneSimplePolygon(const std::vector<Eigen::Vector2f>& points, const float eps) {
	const int n_vert = points.size();
	if(n_vert < 3) {
		return false;
	}
	auto mod = [&](int x) {
		if(x >= 0) {
			return x % n_vert;
		} else {
			return (x % n_vert) + n_vert;
		}
	};

	// check for too short edge
	for(int i : boost::irange(0, (int)points.size())) {
		const auto v0 = points[i];
		const auto v1 = points[(i + 1) % n_vert];
		if((v0 - v1).norm() < eps) {
			return false;
		}
	}

	// Check for self-intersection.
	for(int i : boost::irange(0, (int)points.size())) {
		const auto seg_i = std::make_pair(
			points[i], points[mod(i + 1)]);
		for(int j : boost::irange(0, (int)points.size())) {
			// exclude adjacent edges & itself from check
			if(j == mod(i - 1) || j == i || j == mod(i + 1) || j == mod(i + 2)) {
				continue;
			}

			const auto seg_j = std::make_pair(
				points[j], points[mod(j + 1)]);
			if(intersectSegments(seg_i, seg_j)) {
				return false;
			}
		}
	}
	return true;
}

}  // namespace

#include "cloud_conversion.h"

namespace recon {

// Get barycentric coordinate of the narest point on triangle surface.
Eigen::Vector2f nearestBarycentricApprox(
	const Eigen::Vector3f& v0, const Eigen::Vector3f& v1, const Eigen::Vector3f& v2, const Eigen::Vector3f& p) {
	const auto d1 = v1 - v0;
	const auto d2 = v2 - v0;
	Eigen::Matrix3f m;
	m.col(0) = d1;
	m.col(1) = d2;
	m.col(2) = d1.cross(d2);

	const Eigen::Vector3f coord = m.inverse() * (p - v0);
	// approximate barycentric coordinates.
	float bary0 = std::max(0.0f, coord(0));
	float bary1 = std::max(0.0f, coord(1));
	if(bary1 + bary0 > 1) {
		const float scale = 1 / (bary0 + bary1);
		bary0 *= scale;
		bary1 *= scale;
	}
	return Eigen::Vector2f(bary0, bary1);
}

// Get linear-interpolate coordinate of the surfact point, which is nearest to the given point.
std::pair<float, Eigen::Vector2f> nearestCoordinate(
	const TriangleMesh<Eigen::Vector2f>& mesh, const Eigen::Vector3f p) {
	float min_dist = std::numeric_limits<float>::max();
	Eigen::Vector2f coord;
	for(const auto& tri : mesh.triangles) {
		const Eigen::Vector3f v0 = mesh.vertices[std::get<0>(tri)].first;
		const Eigen::Vector3f v1 = mesh.vertices[std::get<1>(tri)].first;
		const Eigen::Vector3f v2 = mesh.vertices[std::get<2>(tri)].first;
		const auto bary = nearestBarycentricApprox(v0, v1, v2, p);

		const Eigen::Vector3f pt_nearest = v0 + (v1 - v0) * bary(0) + (v2 - v0) * bary(1);
		const float dist = (p - pt_nearest).norm();
		if(dist < min_dist) {
			min_dist = dist;

			const Eigen::Vector2f c0 = mesh.vertices[std::get<0>(tri)].second;
			const Eigen::Vector2f c1 = mesh.vertices[std::get<1>(tri)].second;
			const Eigen::Vector2f c2 = mesh.vertices[std::get<2>(tri)].second;
			coord = c0 + (c1 - c0) * bary(0) + (c2 - c0) * bary(1);
		}
	}
	return std::make_pair(min_dist, coord);
}


// Get linear-interpolate coordinate of the surfact point, which is nearest to the given point.
std::tuple<float, Eigen::Vector2f, Eigen::Vector3f> nearestCoordinateWithNormal(
	const TriangleMesh<Eigen::Vector2f>& mesh, const Eigen::Vector3f p) {
	float min_dist = std::numeric_limits<float>::max();
	Eigen::Vector2f coord;
	Eigen::Vector3f normal;
	for(const auto& tri : mesh.triangles) {
		const Eigen::Vector3f v0 = mesh.vertices[std::get<0>(tri)].first;
		const Eigen::Vector3f v1 = mesh.vertices[std::get<1>(tri)].first;
		const Eigen::Vector3f v2 = mesh.vertices[std::get<2>(tri)].first;
		const auto bary = nearestBarycentricApprox(v0, v1, v2, p);

		const Eigen::Vector3f pt_nearest = v0 + (v1 - v0) * bary(0) + (v2 - v0) * bary(1);
		const float dist = (p - pt_nearest).norm();
		if(dist < min_dist) {
			min_dist = dist;

			const Eigen::Vector2f c0 = mesh.vertices[std::get<0>(tri)].second;
			const Eigen::Vector2f c1 = mesh.vertices[std::get<1>(tri)].second;
			const Eigen::Vector2f c2 = mesh.vertices[std::get<2>(tri)].second;
			coord = c0 + (c1 - c0) * bary(0) + (c2 - c0) * bary(1);
			normal = (v1 - v0).cross(v2 - v0).normalized();
		}
	}
	return std::make_tuple(min_dist, coord, normal);
}

}  // namespace

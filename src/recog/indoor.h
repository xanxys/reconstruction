// Contains indoor-specific representation and analyzing algorithms.
#pragma once

#include <vector>

#include <Eigen/Dense>

namespace recon {

// Represents Manhattan indoor room coordinates.
// But RoomFrame doesn't hold raw scans.
// (aligned scans and RoomFrame are implicitly tied by having
// shared world coordinate system)
//
// Main purpose of RoomFrame is information compression.
// (put another way, it's a very strong prior)
// Do NOT put fine-detailed contour here.
// New model (such as arcs) should be implemented in such cases.
//
// To make it obvious, do not store large objects here.
// (i.e. PointCloud, TexturedMesh, cv::Mat, etc.)
//
// RoomFrames defines not just entire 3-d coordinates, but
// * boundary of room (maybe fuzzy around windows)
// * wall / ceiling / floor coodinates
//
// Z+ is assumed to be up, but Z offset and other Manhattan axes
// are arbitrary.
//
// features:
// Mahnatta-aligned, rectangular pit or protrusion.
// One of edge must be substantial (>50%) of base surface.
// Depth must be small (<1m).
// (It should be called "wall" if it's very deep)
//
// Very thin or small "feature" should be called fixed object instead.
// (e.g. AC, lights)
class RoomFrame {
public:
	RoomFrame();

	// Set Z range that corresponds to min and max inside the room.
	// (ill-defined when there's holes in ceiling etc.)
	void setHRange(float z0, float z1);

	std::pair<float, float> getHRange() const;

	// Get wall contour after Manhattan enforcement as CCW ordered
	// vertices.
	// Warning: result will not align well with world X, Y axes.
	std::vector<Eigen::Vector2f> getSimplifiedContour() const;
public:
	std::vector<Eigen::Vector2f> wall_polygon;
private:
	// up and principle directions.
	Eigen::Vector3f up;
	Eigen::Vector3f prin0;
	Eigen::Vector3f prin1;

	// z range
	float z0;
	float z1;
};

}  // namespace

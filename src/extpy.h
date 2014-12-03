// C++ <-> JSON wrapper for extpy/ calls, since it's decided
// that this project won't use protocol buffers.
// Write these only for somewhat stabilized APIs.
#pragma once

#include <string>
#include <vector>

#include <boost/optional.hpp>
#include <Eigen/Dense>

namespace recon {

// No-visualization version.
std::vector<Eigen::Vector2f> extFixContour(
	const std::vector<Eigen::Vector2f>& room_poly,
	const boost::optional<std::string>& vis_path);

}  // namespace

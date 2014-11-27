#pragma once

namespace recon {

constexpr double pi = 3.14159265359;

constexpr double deg_to_rad(double deg) {
	return deg * (pi / 180.0);
}

}  // namespace

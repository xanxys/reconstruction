#pragma once

namespace recon {

constexpr double pi = 3.14159265359;

constexpr double deg_to_rad(double deg) {
	return deg * (pi / 180.0);
}

// Return nearest power of 2 number >= x.
// ceilToPowerOf2(x) = 1 for x <=0.
int ceilToPowerOf2(int x);


}  // namespace

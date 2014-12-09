#include "math_util.h"

namespace recon {

int ceilToPowerOf2(int x) {
	int r = 1;
	while(r < x) {
		r *= 2;
	}
	return r;
}

}  // namespace

#include "packing.h"

#include <algorithm>

#include <boost/optional.hpp>

#include <logging.h>

namespace recon {

std::pair<float, std::vector<Eigen::Vector2f>>
	packRectangles(std::vector<Eigen::Vector2f> rectangles) {
	const float min_area = std::accumulate(rectangles.begin(), rectangles.end(), 0.0f,
			[](float accum, const Eigen::Vector2f& rect) {
				return accum + rect(0) * rect(1);
			});

	// Sort width of boxes in decreasing order.
	/* TODO: needs index recovering if we do this.
	std::sort(rectangles.begin(), rectangles.end(),
		[](const Eigen::Vector2f& rect_a, const Eigen::Vector2f& rect_b) {
			return rect_a(0) > rect_b(0);
		});
		*/

	// Starting with 1x of the minimum area, enlarge until packing succeeds.
	for(float area_scale = 1; true; area_scale += 0.5) {
		INFO("Trying scale", area_scale);
		const float area = min_area * area_scale;
		const float size = std::sqrt(area);

		// Packing strategy:
		// Goal: Create columns like this.
		// |.....|.....|...|..|
		// |.....|.....|...|..|
		// |.....|.....|...|..|
		// |.....|.....|...|..|
		// |.....|.....|...|..|
		//
		// Each column contains hozirontally(X-axis) long boxes.
		//

		std::vector<Eigen::Vector2f> offsets;
		float column_left = 0;
		float column_height = 0;
		boost::optional<float> column_width;
		bool failed = false;
		for(const auto rect : rectangles) {
			// Abort if current column sticks out of square region.
			if(column_left > size) {
				failed = true;
				break;
			}

			// Move on to next column if current one is full.
			if(column_height + rect(1) > size) {
				assert(column_width);
				column_left += *column_width;
				column_height = 0;
				column_width = boost::none;
			}

			// If triangle is too large even after column-switch,
			// then abort.
			if(column_height + rect(1) > size) {
				failed = true;
				break;
			}

			// Add this triangle.
			offsets.emplace_back(column_left, column_height);
			column_height += rect(1);
			if(!column_width) {
				column_width = rect(0);
			} else {
				column_width = std::max(*column_width, rect(0));
			}
		}
		// Did the last column fit?
		if(column_width && column_left + *column_width >= size) {
			failed = true;
		}
		if(!failed) {
			INFO("Rectangle packing: efficiency=", min_area / area);
			return std::make_pair(size, offsets);
		}
	}
}

}  // namespace

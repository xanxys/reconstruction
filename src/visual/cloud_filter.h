#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace recon {

// Smooth registration error using method in
// http://www-video.eecs.berkeley.edu/papers/elturner/thesis_paper.pdf
// Sec. 3.2
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr squashRegistrationError(
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input,
	float error_radius = 0.2);

}  // namespace

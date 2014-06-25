#include "cloud_baker.h"

#include <array>
#include <iostream>
#include <fstream>
#include <limits>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <logging.h>
#include <visual/cloud_conversion.h>
#include <visual/mapping.h>
#include <visual/marching_cubes.h>
#include <visual/scene_converter.h>
#include <visual/texture_conversion.h>

namespace visual {

CloudBaker::CloudBaker(const Json::Value& cloud_json) {
	cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
	for(const auto& point : cloud_json) {
		pcl::PointXYZRGB pt;
		pt.x = point["x"].asDouble();
		pt.y = point["y"].asDouble();
		pt.z = point["z"].asDouble();
		pt.r = point["r"].asDouble();
		pt.g = point["g"].asDouble();
		pt.b = point["b"].asDouble();
		cloud->points.push_back(pt);
	}
}

void CloudBaker::writeWavefrontObject() {
	const auto cloud_colorless = decolor(*cloud);
	TriangleMesh<std::nullptr_t> mesh = OBBFitter(cloud_colorless).extract();
	const auto mesh_uv = mapSecond(assignUV(mesh));
	cv::imwrite("uv_box.png", visualizeUVMap(mesh_uv));

	// Project points to the surface and draw circle onto texture.
	const int tex_size = 2048;
	cv::Mat texture(tex_size, tex_size, CV_8UC3);
	texture = cv::Scalar(0, 0, 0);
	for(const auto& point : cloud->points) {
		const Eigen::Vector3f pos = point.getVector3fMap();
		const auto uv = nearestCoordinate(mesh_uv, pos);
		// DEBUG("nearestUV", uv(0), uv(1));

		const cv::Scalar color(point.b, point.g, point.r);
		cv::circle(
			texture, eigenToCV(swapY(uv) * tex_size), 1,
			color, -1);
	}
	cv::imwrite("uv_pt_baked.png", texture);

	std::ofstream room_box_uv("room_box_uv.obj");
	mesh_uv.serializeObjWithUv(room_box_uv, "uv_baked.mtl");
	std::ofstream mat_baked("uv_baked.mtl");
	writeObjMaterial(mat_baked, "uv_pt_baked.png");

//	WavefrontObject obj;
	//return obj;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CloudBaker::decolor(const pcl::PointCloud<pcl::PointXYZRGB>& cloud) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_colorless(new pcl::PointCloud<pcl::PointXYZ>());
	for(const auto& point : cloud) {
		pcl::PointXYZ pt;
		pt.x = point.x;
		pt.y = point.y;
		pt.z = point.z;
		cloud_colorless->points.push_back(pt);
	}
	return cloud_colorless;
}

/*
VirtualFile CloudBaker::writeImage(std::string name, const cv::Mat& image) {
	cv::imencode8
}
*/

}  // namespace

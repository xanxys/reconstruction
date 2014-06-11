#include "cloud_baker.h"

#include <array>
#include <iostream>
#include <fstream>
#include <limits>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cloud_conversion.h>
#include <logging.h>
#include <mapping.h>
#include <marching_cubes.h>
#include <recon_server.h>
#include <scene_converter.h>
#include <texture_conversion.h>

CloudBaker::CloudBaker(const Json::Value& cloud) : cloud(cloud) {
}

void CloudBaker::writeWavefrontObject() {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>());
	for(const auto& point : cloud) {
		pcl::PointXYZ pt;
		pt.x = point["x"].asDouble();
		pt.y = point["y"].asDouble();
		pt.z = point["z"].asDouble();
		cloud_pcl->points.push_back(pt);
	}

	TriangleMesh<std::nullptr_t> mesh = OBBFitter(cloud_pcl).extract();
	const auto mesh_uv = mapSecond(assignUV(mesh));
	cv::imwrite("uv_box.png", visualizeUVMap(mesh_uv));

	// Project points to the surface and draw circle onto texture.
	const int tex_size = 2048;
	cv::Mat texture(tex_size, tex_size, CV_8UC3);
	texture = cv::Scalar(0, 0, 0);
	for(const auto& point : cloud) {
		const Eigen::Vector3f pos(
			point["x"].asDouble(),
			point["y"].asDouble(),
			point["z"].asDouble());
		const auto uv = nearestCoordinate(mesh_uv, pos);
		// DEBUG("nearestUV", uv(0), uv(1));

		const cv::Scalar color(
			point["b"].asDouble(),
			point["g"].asDouble(),
			point["r"].asDouble());

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

/*
VirtualFile CloudBaker::writeImage(std::string name, const cv::Mat& image) {
	cv::imencode8
}
*/

#include "cloud_baker.h"

#include <array>
#include <iostream>
#include <fstream>
#include <limits>

#include <boost/filesystem.hpp>
#include <boost/range/irange.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <analyzer/voxel_traversal.h>
#include <logging.h>
#include <range2.h>
#include <visual/cloud_conversion.h>
#include <visual/mapping.h>
#include <visual/marching_cubes.h>
#include <visual/texture_conversion.h>

namespace visual {

using Tuple3i = std::tuple<int, int, int>;

void TexturedMesh::writeWavefrontObject(std::string dir_name) const {
	const boost::filesystem::path dir_path(dir_name);
	boost::filesystem::create_directory(dir_path);

	const boost::filesystem::path name_obj("object.obj");
	const boost::filesystem::path name_mtl("object.mtl");
	const boost::filesystem::path name_diffuse("diffuse.png");

	// Write bunch of files.
	std::ofstream f_obj((dir_path / name_obj).string());
	mesh.serializeObjWithUv(f_obj, name_mtl.string());

	std::ofstream f_mtl((dir_path / name_mtl).string());
	writeObjMaterial(f_mtl, name_diffuse.string());

	cv::imwrite((dir_path / name_diffuse).string(), diffuse);
}

void TexturedMesh::writeWavefrontObjectFlat(std::string prefix) const {
	const boost::filesystem::path name_obj(prefix + "object.obj");
	const boost::filesystem::path name_mtl(prefix + "object.mtl");
	const boost::filesystem::path name_diffuse(prefix + "diffuse.png");

	// Write bunch of files.
	std::ofstream f_obj(name_obj.string());
	mesh.serializeObjWithUv(f_obj, name_mtl.filename().string());

	std::ofstream f_mtl(name_mtl.string());
	writeObjMaterial(f_mtl, name_diffuse.filename().string());

	cv::imwrite(name_diffuse.string(), diffuse);
}
}  // namespace

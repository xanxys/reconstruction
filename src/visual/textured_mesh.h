#pragma once

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <logging.h>
#include <visual/triangle_mesh.h>

namespace recon {

// A triangle mesh with single diffuse texture.
class TexturedMesh {
public:
	// attrib = uv
	TexturedMesh(
		const TriangleMesh<Eigen::Vector2f>& mesh,
		const cv::Mat& diffuse);
	// attrib = normal, uv
	TexturedMesh(
		const TriangleMesh<std::pair<Eigen::Vector3f, Eigen::Vector2f>>& mesh,
		const cv::Mat& diffuse);

	// Write texture, material, geometry into specified directory.
	void writeWavefrontObject(const std::string& dir_name) const;

	// Write bunch of files with specified prefix.
	// e.g. when prefix = "test/a"
	// * test/a_object.obj
	// * test/a_object.mtl
	// * test/a_diffuse.png
	void writeWavefrontObjectFlat(const std::string& prefix) const;

	// Extrapolate one pixels around boundary of UV-mapped region,
	// to remove bleeding black color at texture seams.
	// note: this function could introduce additional color bleeding
	// artifact if UV mapping have too little gaps.
	void extrapolateAtlasBoundary();
protected:
	// Get (square) texture size, assuming diffuse is already set.
	int getTextureSize() const;
public:
	// TODO: encapsulation breach!
	// has_normal and mesh_w_normal creates implicit coupling with InteriorObject
	// and SceneAssetBundle. Do something about it.
	bool has_normal;  // super hackish way to allow InteriorObject to have pre-calculated normals.
	TriangleMesh<Eigen::Vector2f> mesh;
	TriangleMesh<std::pair<Eigen::Vector3f, Eigen::Vector2f>> mesh_w_normal;
	cv::Mat diffuse;
};

// Return a single TexturedMesh (with single texture), representing
// original meshes without texture quality loss.
// color bleeding might occur near edges.
TexturedMesh mergeTexturedMeshes(
	const std::vector<TexturedMesh>& meshes);

void writeObjMaterial(std::ostream& output,
	const std::string& texture_path,
	const std::string& material_name);

// Dump 3D world positions of each pixel of UV-mapped mesh
// as a texture.
// return: CV_32FC3 of size (tex_size, tex_size)
// each pixel is (x, y, z)
cv::Mat getPositionMapInUV(
	const TriangleMesh<Eigen::Vector2f>& mesh, int tex_size,
	const Eigen::Vector3f& default_value);

}  // namespace

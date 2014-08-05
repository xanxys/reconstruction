#include "scene_recognizer.h"

#include <fstream>

#include <boost/filesystem.hpp>
#include <boost/range/irange.hpp>

#include <third/ICP.h>
#include <visual/cloud_baker.h>
#include <visual/shape_fitter.h>

namespace visual {

void SceneAssetBundle::serializeIntoDirectory(std::string dir_path_raw) const {
	using boost::filesystem::create_directory;
	using boost::filesystem::path;
	using boost::filesystem::remove_all;

	const path dir_path(dir_path_raw);

	// Remove directory if exists, mimicing overwriting semantics
	// of a single file.
	remove_all(dir_path);
	create_directory(dir_path);
	exterior_mesh.writeWavefrontObject(
		(dir_path / path("exterior_mesh")).string());
	{
		std::ofstream debug_points_file((dir_path / path("debug_points_distance.ply")).string());
		serializeDebugPoints(debug_points_distance).serializePLYWithRgb(debug_points_file);
	}
	{
		std::ofstream debug_points_file((dir_path / path("debug_points_merged.ply")).string());
		serializeDebugPoints(debug_points_merged).serializePLYWithRgb(debug_points_file);
	}
	{
		std::ofstream json_file((dir_path / path("small_data.json")).string());
		json_file << Json::FastWriter().write(serializeSmallData());
	}
}

TriangleMesh<Eigen::Vector3f> SceneAssetBundle::serializeDebugPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) const {
	TriangleMesh<Eigen::Vector3f> mesh;
	for(const auto& pt : cloud->points) {
		mesh.vertices.push_back(std::make_pair(
			pt.getVector3fMap(),
			Eigen::Vector3f(pt.r, pt.g, pt.b)));
	}
	return mesh;
}

Json::Value SceneAssetBundle::serializeSmallData() const {
	Json::Value small_data;
	for(const auto& pos : point_lights) {
		Json::Value light;
		light["pos"]["x"] = pos(0);
		light["pos"]["y"] = pos(1);
		light["pos"]["z"] = pos(2);
		small_data["lights"].append(light);
	}
	return small_data;
}


SingleScan::SingleScan(Json::Value& cloud_json) {
	// Convert input coords. space to make Z+ up.
	// Only required for old dataset which was Y+ up.
	Eigen::Matrix3f m;
	m.col(0) = Eigen::Vector3f(0, 1, 0);
	m.col(1) = Eigen::Vector3f(0, 0, 1);
	m.col(2) = Eigen::Vector3f(1, 0, 0);
	cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
	for(const auto& point : cloud_json) {
		pcl::PointXYZRGB pt;
		pt.x = point["x"].asDouble();
		pt.y = point["y"].asDouble();
		pt.z = point["z"].asDouble();
		pt.r = point["r"].asDouble();
		pt.g = point["g"].asDouble();
		pt.b = point["b"].asDouble();

		pt.getVector3fMap() = m * pt.getVector3fMap();
		cloud->points.push_back(pt);
	}
}

SingleScan::SingleScan(const std::string& scan_dir) {
	using boost::filesystem::path;
	INFO("Loading a scan from", scan_dir);
	std::ifstream f_input((path(scan_dir) / path("points.json")).string());
	if(!f_input.is_open()) {
		throw std::runtime_error("Cloudn't open points.json");
	}
	Json::Value cloud_json;
	Json::Reader().parse(f_input, cloud_json);

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


std::vector<Eigen::Vector3f> recognize_lights(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
	// Calculate approximate ceiling height.
	std::vector<float> zs;
	for(const auto& pt : cloud->points) {
		zs.push_back(pt.z);
	}
	const auto zs_range = visual::shape_fitter::robustMinMax(zs);
	const float z_ceiling = zs_range.second;

	// Project points to ceiling quad.
	// The quad is created in a way the texture contains
	// non-distorted ceiling image.
	// TODO: need to discard faraway points.
	INFO("Detecting lights at z =", z_ceiling);

	// TODO: 10 here is hardcoded. remove.
	TriangleMesh<Eigen::Vector2f> quad;
	quad.vertices.push_back(std::make_pair(
		Eigen::Vector3f(-10, -10, z_ceiling),
		Eigen::Vector2f(0, 0)));
	quad.vertices.push_back(std::make_pair(
		Eigen::Vector3f(10, -10, z_ceiling),
		Eigen::Vector2f(1, 0)));
	quad.vertices.push_back(std::make_pair(
		Eigen::Vector3f(10, 10, z_ceiling),
		Eigen::Vector2f(1, 1)));
	quad.vertices.push_back(std::make_pair(
		Eigen::Vector3f(-10, 10, z_ceiling),
		Eigen::Vector2f(0, 1)));
	quad.triangles.push_back(std::make_tuple(0, 1, 2));
	quad.triangles.push_back(std::make_tuple(2, 3, 0));

	// Make it grayscale and remove image noise by blurring.
	const TexturedMesh ceiling_geom = cloud_baker::bakePointsToMesh(cloud, quad);
	cv::Mat ceiling_gray;
	cv::cvtColor(ceiling_geom.diffuse, ceiling_gray, cv::COLOR_BGR2GRAY);
	cv::GaussianBlur(ceiling_gray, ceiling_gray, cv::Size(31, 31), 10);

	// Detect blobs (saturated lights).
	std::vector<Eigen::Vector3f> lights;
	cv::SimpleBlobDetector detector;
	std::vector<cv::KeyPoint> blobs;
	detector.detect(ceiling_gray, blobs);
	INFO("Blobs for ceiling image, #=", (int)blobs.size());
	for(const auto& blob : blobs) {
		Json::Value v;
		v["size"] = blob.size;
		v["pt"]["x"] = blob.pt.x;
		v["pt"]["y"] = blob.pt.y;
		v["angle"] = blob.angle;
		INFO("Blob detected", v);

		// TODO: 10 here is hardcoded. remove.
		Eigen::Vector3f pt3d(
			(float)blob.pt.x / ceiling_gray.cols * 20.0 - 10.0,
			(float)blob.pt.y / ceiling_gray.cols * 20.0 - 10.0,
			z_ceiling);
		lights.push_back(pt3d);
	}
	return lights;
}

namespace scene_recognizer {

SceneAssetBundle recognizeScene(const std::vector<SingleScan>& scans) {
	assert(!scans.empty());

	INFO("Merging points in multiple scans");
	const auto points_merged = mergePoints(scans);
	INFO("# of points after merge:", (int)points_merged->points.size());

	INFO("Approximating exterior shape by an extruded polygon");
	const auto cloud_colorless = decolor(*points_merged);
	TriangleMesh<std::nullptr_t> room_mesh = shape_fitter::fitExtrusion(cloud_colorless);

	INFO("Creating assets");
	SceneAssetBundle bundle;
	bundle.exterior_mesh = visual::cloud_baker::bakePointsToMesh(points_merged, room_mesh);
	bundle.debug_points_distance = visual::cloud_baker::colorPointsByDistance(points_merged, room_mesh);
	bundle.debug_points_merged = points_merged;
	bundle.point_lights = visual::recognize_lights(points_merged);
	return bundle;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const float grid_size) {
	std::map<std::tuple<int, int, int>, std::vector<Eigen::Vector3f>> tiles;
	for(const auto& pt_xyz : cloud->points) {
		const Eigen::Vector3f pt = pt_xyz.getVector3fMap();
		const auto pt_i = (pt / grid_size).cast<int>();
		tiles[std::make_tuple(pt_i(0), pt_i(1), pt_i(2))].push_back(pt);
	}
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_new(new pcl::PointCloud<pcl::PointXYZ>);
	for(const auto& tile : tiles) {
		pcl::PointXYZ pt;
		pt.getVector3fMap() = std::accumulate(
			tile.second.begin(), tile.second.end(),
			Eigen::Vector3f(0, 0, 0)) / tile.second.size();
		cloud_new->points.push_back(pt);
	}
	return cloud_new;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr mergePoints(const std::vector<SingleScan>& scans) {
	assert(!scans.empty());
	if(scans.size() == 1) {
		return scans.front().cloud;
	} else {
		auto to_matrix = [](const SingleScan& scan) {
			const auto cloud = downsample(decolor(*scan.cloud), 0.1);
			const int n = cloud->points.size();
			Eigen::Matrix3Xd mat(3, n);
			for(int i : boost::irange(0, n)) {
				mat.col(i) = cloud->points[i].getVector3fMap().cast<double>();
			}
			return mat;
		};

		auto m_source = to_matrix(scans[0]);
		auto m_target = to_matrix(scans[1]);

		INFO("Running Sparse ICP");
		// See this youtube video for quick summary of good p value.
		// https://www.youtube.com/watch?v=ii2vHBwlmo8
		SICP::Parameters params;
		params.max_icp = 100;
		params.p = 0.7;

		// The type signature do look like it allows any Scalar, but only
		// double will work in reality.
		Eigen::Matrix3Xd m_source_orig = m_source;
		SICP::point_to_point(m_source, m_target, params);

		// Recover motion.
		const Eigen::Affine3f m = RigidMotionEstimator::point_to_point(
			m_source_orig, m_source).cast<float>();
		INFO("Affine |t|=", m.translation().norm());

		// Merge color cloud.
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged(new pcl::PointCloud<pcl::PointXYZRGB>);
		for(const auto& pt : scans[0].cloud->points) {
			pcl::PointXYZRGB pt_new = pt;
			pt_new.getVector3fMap() = m * pt.getVector3fMap();
			merged->points.push_back(pt_new);
		}
		for(const auto& pt : scans[1].cloud->points) {
			merged->points.push_back(pt);
		}
		return merged;
	}
}


pcl::PointCloud<pcl::PointXYZ>::Ptr decolor(const pcl::PointCloud<pcl::PointXYZRGB>& cloud) {
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

}  // namespace
}  // namespace

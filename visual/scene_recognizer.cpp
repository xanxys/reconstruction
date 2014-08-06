#include "scene_recognizer.h"

#include <fstream>

#include <boost/filesystem.hpp>
#include <boost/optional.hpp>
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
		std::ofstream debug_points_file((dir_path / path("debug_points_interior.ply")).string());
		serializeDebugPoints(debug_points_interior).serializePLYWithRgb(debug_points_file);
	}
	{
		std::ofstream debug_points_file((dir_path / path("debug_points_interior_distance.ply")).string());
		serializeDebugPoints(debug_points_interior_distance).serializePLYWithRgb(debug_points_file);
	}
	{
		std::ofstream debug_points_file((dir_path / path("debug_points_merged.ply")).string());
		serializeDebugPoints(debug_points_merged).serializePLYWithRgb(debug_points_file);
	}
	{
		std::ofstream json_file((dir_path / path("small_data.json")).string());
		json_file << Json::FastWriter().write(serializeSmallData());
	}
	int count = 0;
	for(const auto& interior : interior_objects) {
		const std::string name = "interior_" + std::to_string(count++);
		interior.writeWavefrontObject((dir_path / name).string());
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


SingleScan::SingleScan(Json::Value& cloud_json) :
		pre_rotation(0) {
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

SingleScan::SingleScan(const std::string& scan_dir, float pre_rotation) :
		pre_rotation(pre_rotation) {
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
	const auto extrusion = shape_fitter::fitExtrusion(cloud_colorless);
	const auto room_mesh = std::get<0>(extrusion);
	const auto room_polygon = std::get<1>(extrusion);

	INFO("Modeling boxes along wall");
	const auto cloud_interior = visual::cloud_baker::colorPointsByDistance(points_merged, room_mesh, true);
	const auto cloud_interior_dist = visual::cloud_baker::colorPointsByDistance(points_merged, room_mesh, false);
	const auto box_ticks = decomposeWallBoxes(decolor(*cloud_interior), room_polygon);
	INFO("Box candidates found", (int)box_ticks.size());
	std::vector<TexturedMesh> boxes;
	for(const auto& tick_range : box_ticks) {
		const auto maybe_box = createWallBox(room_polygon, std::get<2>(extrusion), tick_range, cloud_interior);
		if(maybe_box) {
			boxes.push_back(*maybe_box);
		}
	}
	INFO("Box actually created", (int)boxes.size());

	INFO("Creating assets");
	SceneAssetBundle bundle;
	bundle.point_lights = visual::recognize_lights(points_merged);
	bundle.exterior_mesh = visual::cloud_baker::bakePointsToMesh(points_merged, room_mesh);
	bundle.interior_objects = boxes;
	bundle.debug_points_interior = cloud_interior;
	bundle.debug_points_interior_distance = cloud_interior_dist;
	bundle.debug_points_merged = points_merged;
	return bundle;
}

boost::optional<TexturedMesh> createWallBox(
		const std::vector<Eigen::Vector2f>& polygon,
		std::pair<float, float> z_range,
		std::pair<int, int> ticks,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
	auto p0 = polygon[ticks.first];
	auto p1 = polygon[ticks.second];
	const float depth = 0.4;
	const float height = 0.5;
	const Eigen::Vector2f edge_d = p1 - p0;
	const Eigen::Vector2f edge_n = Eigen::Vector2f(-edge_d(1), edge_d(0)).normalized();
	const Eigen::Vector2f center_2d = (p0 + p1) / 2 + edge_n * (depth * 0.5);

	const Eigen::Vector3f center = append(center_2d, height / 2 + z_range.first);

	const auto box_mesh = shape_fitter::createBox(
		center,
		append(edge_d, 0) / 2,
		append(edge_n, 0) * (depth / 2),
		Eigen::Vector3f(0, 0, height / 2));

	const auto box = visual::cloud_baker::bakePointsToMesh(cloud, box_mesh);
	return boost::optional<TexturedMesh>(box);
}

std::vector<std::pair<int, int>> decomposeWallBoxes(
		pcl::PointCloud<pcl::PointXYZ>::Ptr interior_cloud,
		const std::vector<Eigen::Vector2f>& polygon) {
	std::vector<int> counts;
	// Project points to edges of polygon.
	const int n = polygon.size();
	for(int i : boost::irange(0, n)) {
		const Eigen::Vector2f v0 = polygon[i];
		const Eigen::Vector2f edge_d = polygon[(i + 1) % n] - polygon[i];
		const float edge_len = edge_d.norm();
		const Eigen::Vector2f edge_d_norm = edge_d / edge_len;
		const Eigen::Vector2f edge_n = Eigen::Vector2f(-edge_d(1), edge_d(0)).normalized();
		const float max_distance = 0.5;
		// Count points that falls in this region. (*counted .ignored)
		// .  .   . distance
		//  | *  |
		// .|   *|
		// -+----+-> edge_d
		//  v0
		counts.push_back(0);
		for(const auto& pt : interior_cloud->points) {
			const Eigen::Vector2f pt2d = pt.getVector3fMap().head(2);
			auto dp = pt2d - v0;
			const float distance = dp.dot(edge_n);
			if(distance < 0 || distance > max_distance) {
				continue;
			}
			const float t = dp.dot(edge_d_norm);
			if(t < 0 || t > edge_len) {
				continue;
			}
			counts[i] ++;
		}
	}
	Json::Value wall_histogram;
	for(const auto val : counts) {
		wall_histogram.append(val);
	}
	{
		std::ofstream of("wall_histogram.json");
		of << Json::FastWriter().write(wall_histogram);
	}

	std::vector<std::pair<int, int>> ticks;
	const int thresh = 10;
	boost::optional<int> start_ix;
	for(int i : boost::irange(0, n + 1)) {
		const bool ir = (counts[i % n] > thresh);
		if(start_ix) {
			if(!ir) {
				ticks.push_back(std::make_pair(
					*start_ix, i % n));
				start_ix = boost::none;
			}
		} else {
			if(ir) {
				start_ix = i % n;
			}
		}
	}

	return ticks;
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
		// TODO: remove this!!!!
		// Apply pre rotation.
		for(auto& scan : scans) {
			Eigen::Matrix3f pre_rot;
			pre_rot = Eigen::AngleAxisf(scan.pre_rotation, Eigen::Vector3f::UnitZ());
			for(auto& pt : scan.cloud->points) {
				pt.getVector3fMap() = pre_rot * pt.getVector3fMap();
			}
		}

		// Apply translation to match 2D centroid.
		// Since the scanner can scan 360 degree,
		// it's expected that scan contains most part of 2D XY slice
		// of the room, while height range can vary by occlusion.
		std::vector<Eigen::Vector2f> centroids;
		for(auto& scan : scans) {
			std::vector<float> xs;
			std::vector<float> ys;
			for(auto& pt : scan.cloud->points) {
				xs.push_back(pt.x);
				ys.push_back(pt.y);
			}
			centroids.emplace_back(
				shape_fitter::mean(shape_fitter::robustMinMax(xs, 0.01)),
				shape_fitter::mean(shape_fitter::robustMinMax(ys, 0.01)));
		}
		for(int i : boost::irange(1, (int)scans.size())) {
			auto dp = centroids[0] - centroids[i];
			const Eigen::Vector3f trans(dp(0), dp(1), 0);
			INFO("Pre-aligning scan by centroid", i, trans(0), trans(1));
			for(auto& pt : scans[i].cloud->points) {
				pt.getVector3fMap() += trans;
			}
		}

		// 0: target
		// 1,2,..: source
		auto to_matrix = [](const SingleScan& scan) {
			const auto cloud = downsample(decolor(*scan.cloud), 0.05);
			const int n = cloud->points.size();
			Eigen::Matrix3Xd mat(3, n);
			for(int i : boost::irange(0, n)) {
				mat.col(i) = cloud->points[i].getVector3fMap().cast<double>();
			}
			return mat;
		};

		// Merge scans[0]
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged(new pcl::PointCloud<pcl::PointXYZRGB>);
		for(const auto& pt : scans[0].cloud->points) {
			merged->points.push_back(pt);
		}
		// Merge others.
		for(int i : boost::irange(1, (int)scans.size())) {
			auto m_source = to_matrix(scans[i]);
			auto m_target = to_matrix(scans[0]);

			INFO("Running Sparse ICP", i);
			// See this youtube video for quick summary of good p value.
			// https://www.youtube.com/watch?v=ii2vHBwlmo8
			SICP::Parameters params;
			params.max_icp = 250;
			params.p = 0.7;
			params.stop = 0;

			// The type signature do look like it allows any Scalar, but only
			// double will work in reality.
			Eigen::Matrix3Xd m_source_orig = m_source;
			SICP::point_to_point(m_source, m_target, params);

			// Recover motion.
			const Eigen::Affine3f m = RigidMotionEstimator::point_to_point(
				m_source_orig, m_source).cast<float>();
			INFO("Affine |t|=", m.translation().norm());

			// Merge color cloud.
			for(const auto& pt : scans[i].cloud->points) {
				pcl::PointXYZRGB pt_new = pt;
				pt_new.getVector3fMap() = m * pt.getVector3fMap();
				merged->points.push_back(pt_new);
			}
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

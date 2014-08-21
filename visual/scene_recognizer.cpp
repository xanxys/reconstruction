#include "scene_recognizer.h"

#include <fstream>

#include <boost/filesystem.hpp>
#include <boost/optional.hpp>
#include <boost/range/irange.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <third/ICP.h>
#include <visual/cloud_baker.h>
#include <visual/film.h>
#include <visual/mapping.h>
#include <visual/mesh_intersecter.h>
#include <visual/shape_fitter.h>
#include <visual/texture_conversion.h>

namespace visual {

const double pi = 3.14159265359;

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
	WARN("Since SingleScan Json constructor is no longer supported, program might crash when newer feature is used; consider migration");
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
	cloud_w_normal.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
	for(const auto& point : cloud_json) {
		pcl::PointXYZRGBNormal pt;
		pt.x = point["x"].asDouble();
		pt.y = point["y"].asDouble();
		pt.z = point["z"].asDouble();
		pt.r = point["r"].asDouble();
		pt.g = point["g"].asDouble();
		pt.b = point["b"].asDouble();
		pt.normal_x = point["nx"].asDouble();
		pt.normal_y = point["ny"].asDouble();
		pt.normal_z = point["nz"].asDouble();
		cloud_w_normal->points.push_back(pt);
	}

	INFO("Loading equirectangular images");
	er_rgb = cv::imread((path(scan_dir) / path("rgb.png")).string());
	er_intensity = cv::imread((path(scan_dir) / path("intensity.png")).string());
	er_depth = cv::imread((path(scan_dir) / path("depth.png")).string());

	if(er_rgb.size() == cv::Size(0, 0)) {
		WARN("Image was not loaded properly");
		throw std::runtime_error("Couldn't load ER RGB image");
	}
	if(er_rgb.size() != er_intensity.size() || er_rgb.size() != er_depth.size()) {
		throw std::runtime_error("Inconsistent image sizes");
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


AlignedScans::AlignedScans(const std::vector<SingleScan>& scans) {
	assert(!scans.empty());

	merged.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
	for(const auto& pt : scans[0].cloud->points) {
		merged->points.push_back(pt);
	}
	scans_with_pose.push_back(std::make_pair(
		scans[0],
		Eigen::Affine3f::Identity()));

	// 0: target
	// 1,2,..: source
	// return: (position matrix, normal matrix)
	auto to_matrix = [](pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud) {
		const auto ds_cloud = scene_recognizer::downsample<pcl::PointXYZRGBNormal>(cloud, 0.05);
		const int n = ds_cloud->points.size();
		Eigen::Matrix3Xd mat(3, n);
		Eigen::Matrix3Xd mat_n(3, n);
		for(int i : boost::irange(0, n)) {
			mat.col(i) = ds_cloud->points[i].getVector3fMap().cast<double>();
			mat_n.col(i) = ds_cloud->points[i].getNormalVector3fMap().cast<double>();
		}
		return std::make_pair(mat, mat_n);
	};

	// Merge others.
	for(int i : boost::irange(1, (int)scans.size())) {
		INFO("Calculating best pre-alignment between", 0, i);
		const auto pre_align = prealign(scans[0], scans[i]);
		INFO("Prealign Affine |t|=", pre_align.translation().norm());

		auto m_source_pa = to_matrix(scene_recognizer::applyTransform(scans[i].cloud_w_normal, pre_align));
		auto m_target = to_matrix(scans[0].cloud_w_normal);

		INFO("Running Sparse ICP", i);
		// See this youtube video for quick summary of good p value.
		// https://www.youtube.com/watch?v=ii2vHBwlmo8
		SICP::Parameters params;
		params.max_icp = 200;
		params.p = 0.6;
		//params.stop = 0;

		// The type signature do look like it allows any Scalar, but only
		// double will work in reality.
		Eigen::Matrix3Xd m_source_orig = m_source_pa.first;
		SICP::point_to_plane(m_source_pa.first, m_target.first, m_target.second, params);

		// Recover motion.
		const Eigen::Affine3f fine_align = RigidMotionEstimator::point_to_point(
			m_source_orig, m_source_pa.first).cast<float>();
		INFO("Fine Affine |t|=", fine_align.translation().norm());

		const Eigen::Affine3f trans = fine_align * pre_align;
		// Merge color cloud.
		for(const auto& pt : scans[i].cloud->points) {
			pcl::PointXYZRGB pt_new = pt;
			pt_new.getVector3fMap() = trans * pt.getVector3fMap();
			merged->points.push_back(pt_new);
		}
		scans_with_pose.push_back(std::make_pair(
			scans[i],
			trans));
	}

	applyLeveling();
}

void AlignedScans::applyLeveling() {
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01);

	seg.setInputCloud(merged);
	seg.segment(*inliers, *coefficients);

	if(inliers->indices.size() == 0) {
		WARN ("Could not estimate a planar model for the given dataset.");
		return;
	}

	INFO("Leveling plane:",
		coefficients->values[0], coefficients->values[1],
		coefficients->values[2], coefficients->values[3]);
}

Eigen::Affine3f AlignedScans::prealign(const SingleScan& target, const SingleScan& source) {
	std::vector<Eigen::Affine3f> seeds;
	const int n_rot = 60;
	for(int i : boost::irange(0, n_rot)) {
		seeds.push_back(
			Eigen::Affine3f(Eigen::AngleAxisf(
				(float)i / n_rot * 2 * pi,
				Eigen::Vector3f::UnitZ())));
	}

	auto extract_centroid = [](pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud) {
		std::vector<float> xs;
		std::vector<float> ys;
		for(auto& pt : cloud->points) {
			xs.push_back(pt.x);
			ys.push_back(pt.y);
		}
		return Eigen::Vector2f(
			shape_fitter::mean(shape_fitter::robustMinMax(xs, 0.01)),
			shape_fitter::mean(shape_fitter::robustMinMax(ys, 0.01)));
	};

	// source --seed--> source' --centroid-align--> source'' ==? target
	float best_dist = 1e6;
	Eigen::Affine3f best_trans;
	for(const auto& seed_trans : seeds) {
		const auto cloud_seeded = scene_recognizer::applyTransform(source.cloud_w_normal, seed_trans);
		const auto dp = extract_centroid(target.cloud_w_normal) - extract_centroid(cloud_seeded);
		const Eigen::Affine3f trans_centroid(Eigen::Translation3f(Eigen::Vector3f(dp(0), dp(1), 0)));

		const float dist = scene_recognizer::cloudDistance(scene_recognizer::applyTransform(cloud_seeded, trans_centroid), target.cloud_w_normal);
		DEBUG("Dist(angle)", dist);
		if(dist < best_dist) {
			best_dist = dist;
			best_trans = trans_centroid * seed_trans;
		}
	}
	INFO("Pre-align: best dist=", best_dist);
	return best_trans;
}

std::vector<std::pair<SingleScan, Eigen::Affine3f>> AlignedScans::getScansWithPose() const {
	return scans_with_pose;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr AlignedScans::getMergedPoints() const {
	return merged;
}


namespace scene_recognizer {

SceneAssetBundle recognizeScene(const std::vector<SingleScan>& scans) {
	assert(!scans.empty());

	INFO("Merging points in multiple scans");
	const AlignedScans scans_aligned(scans);
	const auto points_merged = scans_aligned.getMergedPoints();
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
	bundle.exterior_mesh = bakeTexture(scans_aligned, room_mesh);
	bundle.interior_objects = boxes;
	bundle.debug_points_interior = cloud_interior;
	bundle.debug_points_interior_distance = cloud_interior_dist;
	bundle.debug_points_merged = points_merged;
	return bundle;
}

// Apply affine transform to given XYZ+RGB+Normal point cloud,
// and return new transformed cloud.
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr applyTransform(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, const Eigen::Affine3f& trans) {
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
	for(auto& pt : cloud->points) {
		pcl::PointXYZRGBNormal pt_new = pt;
		pt_new.getVector3fMap() = trans * pt.getVector3fMap();
		pt_new.getNormalVector3fMap() = trans * pt.getNormalVector3fMap();
		new_cloud->points.push_back(pt_new);
	}
	return new_cloud;
}

// This function will be called several times for pre-alignment:
// focus on speed rather than accurancy.
float cloudDistance(
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr c1,
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr c2) {
	// Collect single smaple per each bin.
	const float res = 0.3;
	const float weight_normal_cos = 5;
	const float weight_color_l2 = 0.01;
	// PosIndex -> (Normal, RGB)
	std::map<
		std::tuple<int, int, int>,
		std::pair<
			std::vector<std::tuple<Eigen::Vector3f, Eigen::Vector3f>>,
			std::vector<std::tuple<Eigen::Vector3f, Eigen::Vector3f>>>> cells;
	for(const auto& pt : c1->points) {
		const auto ix = (pt.getVector3fMap() / res).cast<int>();
		const auto tix = std::make_tuple(ix(0), ix(1), ix(2));
		cells[tix].first.push_back(std::make_tuple(
			pt.getNormalVector3fMap(),
			pt.getRGBVector3i().cast<float>()));
	}
	for(const auto& pt : c2->points) {
		const auto ix = (pt.getVector3fMap() / res).cast<int>();
		const auto tix = std::make_tuple(ix(0), ix(1), ix(2));
		cells[tix].second.push_back(std::make_tuple(
			pt.getNormalVector3fMap(),
			pt.getRGBVector3i().cast<float>()));
	}

	// Compare each bin's score.
	auto calc_stat = [](const std::vector<std::tuple<Eigen::Vector3f, Eigen::Vector3f>>& samples) {
		const int n = samples.size();
		if(n == 0) {
			return boost::optional<std::tuple<Eigen::Vector3f, Eigen::Vector3f>>();
		}
		Eigen::Vector3f accum_n = Eigen::Vector3f::Zero();
		Eigen::Vector3f accum_c = Eigen::Vector3f::Zero();
		for(const auto& sample : samples) {
			accum_n += std::get<0>(sample);
			accum_c += std::get<1>(sample);
		}
		return boost::optional<std::tuple<Eigen::Vector3f, Eigen::Vector3f>>(std::make_tuple(
			(accum_n / n).normalized(),
			accum_c / n));
	};
	int n_bins = 0;
	float accum_distance = 0;
	for(const auto& bin : cells) {
		const auto& samples = bin.second;
		const auto stat1 = calc_stat(samples.first);
		const auto stat2 = calc_stat(samples.second);
		if(!stat1 || !stat2) {
			continue;
		}

		const float dist =
			weight_normal_cos * (std::get<0>(*stat1) - std::get<0>(*stat2)).norm() +
			weight_color_l2 * (std::get<1>(*stat1) - std::get<1>(*stat2)).norm();
		accum_distance += dist;
		n_bins++;
	}
	if(n_bins == 0) {
		throw std::runtime_error("similarity() is not defined for empty point clouds");
	}
	return 1.0 / n_bins; //accum_distance / (n_bins * std::sqrt(n_bins));  // More bins == more similar
}

TexturedMesh bakeTexture(const AlignedScans& scans, const TriangleMesh<std::nullptr_t>& shape_wo_uv) {
	const int tex_size = 4096;
	FilmRGB8U film(tex_size, tex_size, 1.0);

	TriangleMesh<Eigen::Vector2f> shape = mapSecond(assignUV(shape_wo_uv));
	MeshIntersecter intersecter(shape_wo_uv);
	INFO("Baking texture to mesh with #tri=", (int)shape.triangles.size());
	int n_hits = 0;
	int n_all = 0;
	for(const auto& scan_and_pose : scans.getScansWithPose()) {
		const auto scan = scan_and_pose.first;
		const auto l_to_w = scan_and_pose.second;
		for(int y : boost::irange(0, scan.er_rgb.rows)) {
			for(int x : boost::irange(0, scan.er_rgb.cols)) {
				// Ignore N/A samples.
				// TODO: 0,0,0 rarely occurs naturaly, but when it does,
				// consider migration to RGBA image.
				const cv::Vec3b color = scan.er_rgb.at<cv::Vec3b>(y, x);
				if(color == cv::Vec3b(0, 0, 0)) {
					continue;
				}

				const float theta = (float)y / scan.er_rgb.rows * pi;
				const float phi = -(float)x / scan.er_rgb.cols * 2 * pi;

				const auto org_local = Eigen::Vector3f::Zero();
				const auto dir_local = Eigen::Vector3f(
					std::sin(theta) * std::cos(phi),
					std::sin(theta) * std::sin(phi),
					std::cos(theta));
				Ray ray(
					l_to_w * org_local,
					l_to_w.rotation() * dir_local);
				const auto isect = intersecter.intersect(ray);
				if(isect) {
					const auto tri = shape.triangles[std::get<0>(*isect)];
					const auto uv0 = shape.vertices[std::get<0>(tri)].second;
					const auto uv1 = shape.vertices[std::get<1>(tri)].second;
					const auto uv2 = shape.vertices[std::get<2>(tri)].second;
					const auto uv = std::get<1>(*isect)(0) * (uv1 - uv0) + std::get<1>(*isect)(1) * (uv2 - uv0) + uv0;

					film.record(swapY(uv) * tex_size, color);
					n_hits++;
				}
				n_all++;
			}
		}
		// Exit with only one scan, since currently multiple-scan fusion
		// results in
		// 1. ghosting due to misalignment (or calibration error)
		// 2. color artifact (exposure / color balance difference)
		break;
	}
	INFO("Baking Hits/All", n_hits, n_all);

	// pack everything.
	TexturedMesh tm;
	tm.diffuse = film.extract();
	tm.mesh = shape;
	return tm;
}

boost::optional<TexturedMesh> createWallBox(
		const std::vector<Eigen::Vector2f>& polygon,
		std::pair<float, float> z_range,
		std::pair<int, int> ticks,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
	const auto p0 = polygon[ticks.first];
	const auto p1 = polygon[ticks.second];
	const Eigen::Vector2f edge_d = p1 - p0;
	const Eigen::Vector2f edge_d_norm = edge_d.normalized();
	const float edge_len = edge_d.norm();
	const Eigen::Vector2f edge_n = Eigen::Vector2f(-edge_d(1), edge_d(0)).normalized();

	// Collect points.
	const float max_distance = 0.8;
	std::vector<float> heights;
	std::vector<float> depths;
	for(const auto& pt : cloud->points) {
		const Eigen::Vector2f pt2d = pt.getVector3fMap().head(2);
		auto dp = pt2d - p0;
		const float distance = dp.dot(edge_n);
		if(distance < 0 || distance > max_distance) {
			continue;
		}
		const float t = dp.dot(edge_d_norm);
		if(t < 0 || t > edge_len) {
			continue;
		}

		heights.push_back(pt.z - z_range.first);
		depths.push_back(distance);
	}

	const float depth = shape_fitter::robustMinMax(depths, 0.2).second;
	const float height = shape_fitter::robustMinMax(heights, 0.2).second;

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

pcl::PointCloud<pcl::PointXYZ>::Ptr decolor(const pcl::PointCloud<pcl::PointXYZRGB>& cloud) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_colorless(new pcl::PointCloud<pcl::PointXYZ>());
	for(const auto& point : cloud) {
		pcl::PointXYZ pt;
		pt.getVector3fMap() = point.getVector3fMap();
		cloud_colorless->points.push_back(pt);
	}
	return cloud_colorless;
}

}  // namespace
}  // namespace

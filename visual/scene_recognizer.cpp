#include "scene_recognizer.h"

#include <fstream>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/optional.hpp>
#include <boost/range/irange.hpp>
// Unfortunately, CGAL headers are sensitive to include orders.
// these are copied from example code in the docs.
// DO NOT TOUCH IT!
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/create_offset_polygons_2.h>
// insanity continues (3d polyhedra)
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/point_generators_3.h>
#include <CGAL/algorithm.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/convex_hull_3.h>
// insanity ends here
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <visual/cloud_baker.h>
#include <visual/cloud_base.h>
#include <visual/film.h>
#include <visual/mapping.h>
#include <visual/mesh_intersecter.h>
#include <visual/shape_fitter.h>
#include <visual/texture_conversion.h>

namespace visual {

const double pi = 3.14159265359;

std::vector<Eigen::Vector3f> recognize_lights(
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
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

void test_segmentation(
		SceneAssetBundle& bundle,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
	
	pcl::search::Search<pcl::PointXYZRGB>::Ptr tree =
		boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB>> (new pcl::search::KdTree<pcl::PointXYZRGB>);
	
	pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod (tree);
	normal_estimator.setInputCloud (cloud);
	normal_estimator.setKSearch (50);
	normal_estimator.compute (*normals);
	

	// pcl::IndicesPtr indices (new std::vector <int>);
	// pcl::PassThrough<pcl::PointXYZ> pass;
	// pass.setInputCloud (cloud);
	// pass.setFilterFieldName ("z");
	// pass.setFilterLimits (0.0, 1.0);
	// pass.filter (*indices);

	pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
	reg.setMinClusterSize (50);
	reg.setMaxClusterSize (1000000);
	reg.setSearchMethod (tree);
	reg.setNumberOfNeighbours (30);
	reg.setInputCloud (cloud);
	//reg.setIndices (indices);
	reg.setInputNormals(normals);
	reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
	reg.setCurvatureThreshold (1.0);

	std::vector <pcl::PointIndices> clusters;
	reg.extract (clusters);

	INFO("Number of clusters=", (int)clusters.size());
	INFO("Size of 1st cluster", (int)clusters[0].indices.size());
	/*
	for(int i : boost::irange(0, (int)clusters.size())) {

	}
	int counter = 0;
	while (counter < clusters[0].indices.size ())
	{
	std::cout << clusters[0].indices[counter] << ", ";
	counter++;
	if (counter % 10 == 0)
	  std::cout << std::endl;
	}
	std::cout << std::endl;
	*/

	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
	//pcl::visualization::CloudViewer viewer ("Cluster viewer");
	//viewer.showCloud(colored_cloud);
	bundle.addDebugPointCloud("clusters", colored_cloud);

}

void recognizeScene(SceneAssetBundle& bundle, const std::vector<SingleScan>& scans) {
	assert(!scans.empty());

	INFO("Merging points in multiple scans");
	const AlignedScans scans_aligned(bundle, scans);
	const auto points_merged = scans_aligned.getMergedPointsNormal();
	bundle.addDebugPointCloud("points_merged", points_merged);
	INFO("# of points after merge:", (int)points_merged->points.size());

	INFO("Approximating exterior shape by an extruded polygon");
	const auto cloud_colorless = cloud_base::cast<pcl::PointXYZRGBNormal, pcl::PointXYZ>(points_merged);
	const auto extrusion = shape_fitter::fitExtrusion(cloud_colorless);
	auto room_mesh = std::get<0>(extrusion);
	auto room_polygon = std::get<1>(extrusion);
	auto room_hrange = std::get<2>(extrusion);
	// We're just lucky to get correct room polygon without excluding outside points first.
	// points
	// |-inside
	// | |-exterior
	// | |-interior
	// |-outside (either mirror or window, there's a case RGB is transparent and IR is reflected)
	INFO("Splitting inside/outside");
	auto points_inside = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	auto points_outside = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
	typedef K::Point_2 Point;
	typedef CGAL::Polygon_2<K> Polygon_2;
	std::vector<Point> pp;
	for(const auto& pt : room_polygon) {
		pp.emplace_back(pt.x(), pt.y());
	}
	Polygon_2 room_polygon_cgal(pp.begin(), pp.end());
	assert(room_polygon_cgal.is_simple());
	/** CGAL-4.2 doesn't like C++11 (boost::shared_ptr cannot be implicitly casted to bool)
	auto polys = CGAL::create_exterior_skeleton_and_offset_polygons_2(
		offset, room_polygon_cgal);
	*/
	for(const auto& pt3 : points_merged->points) {
		// hack to simulate offset, assuming origin is near centroid
		Eigen::Vector2f d(pt3.x, pt3.y);
		d *= -0.2 / d.norm();

		const Point pt(pt3.x + d.x(), pt3.y + d.y());
		if(room_polygon_cgal.bounded_side(pt) != CGAL::ON_UNBOUNDED_SIDE) {
			points_inside->points.push_back(pt3);
		} else {
			points_outside->points.push_back(pt3);
		}
	}
	bundle.addDebugPointCloud("points_outside", points_outside);

	INFO("Recalculating extrusion");
	const auto extrusion_refined = shape_fitter::fitExtrusion(cloud_base::cast<pcl::PointXYZRGBNormal, pcl::PointXYZ>(points_inside));
	room_mesh = std::get<0>(extrusion_refined);
	room_polygon = std::get<1>(extrusion_refined);
	room_hrange = std::get<2>(extrusion_refined);

	INFO("Modeling boxes along wall");
	auto cloud_interior_pre = visual::cloud_baker::colorPointsByDistance<pcl::PointXYZRGBNormal>(points_inside, room_mesh, true);
	INFO("Rejecting near-ceiling points");
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_interior(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	const float ceiling_range = 0.65;
	for(const auto& pt : cloud_interior_pre->points) {
		if(pt.z > room_hrange.second - ceiling_range) {
			continue;
		}
		cloud_interior->points.push_back(pt);
	}

	bundle.addDebugPointCloud("points_interior", cloud_interior);
	const auto cloud_interior_dist = visual::cloud_baker::colorPointsByDistance<pcl::PointXYZRGBNormal>(points_inside, room_mesh, false);
	bundle.addDebugPointCloud("points_interior_distance", cloud_interior_dist);

	std::vector<TexturedMesh> boxes;
	/*
	const auto box_ticks = decomposeWallBoxes(cloud_base::cast<pcl::PointXYZRGBNormal, pcl::PointXYZ>(cloud_interior), room_polygon);
	INFO("Box candidates found", (int)box_ticks.size());
	for(const auto& tick_range : box_ticks) {
		const auto maybe_box = createWallBox(room_polygon, room_hrange, tick_range,
			cloud_base::cast<pcl::PointXYZRGBNormal, pcl::PointXYZRGB>(cloud_interior));
		if(maybe_box) {
			boxes.push_back(*maybe_box);
		}
	}
	INFO("Box actually created", (int)boxes.size());
	*/

	INFO("Creating assets");
	bundle.point_lights = visual::recognize_lights(cloud_base::cast<pcl::PointXYZRGBNormal, pcl::PointXYZRGB>(points_inside));
	bundle.exterior_mesh = bakeTexture(scans_aligned, room_mesh);
	bundle.interior_objects = boxes;
}


void recognizeScene2(SceneAssetBundle& bundle, const std::vector<SingleScan>& scans) {
	Json::Value cloud_json = bundle.loadJson("debug_filtered.json");

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(
		new pcl::PointCloud<pcl::PointXYZRGBNormal>());
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
		cloud->points.push_back(pt);
	}
	DEBUG("Loaded #points", (int)cloud->points.size());

	splitObjects(bundle, cloud);
}


void splitObjects(
		SceneAssetBundle& bundle,
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_org) {
	using K = CGAL::Exact_predicates_inexact_constructions_kernel;
	using Point_3 = K::Point_3;
	using Polyhedron_3 = CGAL::Polyhedron_3<K>;

	pcl::search::Search<pcl::PointXYZRGB>::Ptr tree =
		boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB>>(
			new pcl::search::KdTree<pcl::PointXYZRGB>);

	// decompose to XYZRGB + Normal
	auto cloud = visual::cloud_base::cast<pcl::PointXYZRGBNormal, pcl::PointXYZRGB>(cloud_org);
	auto normals = visual::cloud_base::cast<pcl::PointXYZRGBNormal, pcl::Normal>(cloud_org);

	INFO("Doing EC");
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
	ec.setClusterTolerance(0.02); // 2cm
	ec.setMinClusterSize(100);
	ec.setMaxClusterSize(100000);  // 100000: most small objects / 500000: everything incl. tabgles
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);

	std::vector<pcl::PointIndices> clusters;
	ec.extract(clusters);

	INFO("Number of clusters=", (int)clusters.size());
	INFO("Size of 1st cluster", (int)clusters[0].indices.size());

	// create polyhedron.
	int i_cluster = 0;
	for(const auto& indices : clusters) {
		std::vector<Point_3> points;
		for(int ix : indices.indices) {
			auto pt = cloud->points[ix];
			points.emplace_back(pt.x, pt.y, pt.z);
		}
		Polyhedron_3 poly;
		CGAL::convex_hull_3(points.begin(), points.end(), poly);
		INFO("Polyhedron #vert", (int)poly.size_of_vertices());
		assert(poly.is_pure_triangle());

		visual::TriangleMesh<std::nullptr_t> mesh;
		for(auto it_f = poly.facets_begin(); it_f != poly.facets_end(); it_f++) {
			const int v0 = mesh.vertices.size();
			auto it_e = it_f->facet_begin();
			for(int i : boost::irange(0, 3)) {
				const auto p = it_e->vertex()->point();
				mesh.vertices.push_back(std::make_pair(
					Eigen::Vector3f(p.x(), p.y(), p.z()),
					nullptr));
				it_e++;
			}
			mesh.triangles.push_back(std::make_tuple(
				v0, v0 + 1, v0 + 2));
		}

		bundle.addMesh("poly_" + std::to_string(i_cluster), mesh);

		// bake texture
		const auto tex_mesh = visual::cloud_baker::bakePointsToMesh(cloud, mesh);
		bundle.addMesh("poly_" + std::to_string(i_cluster), tex_mesh);
		i_cluster++;
	}

	std::default_random_engine generator;
	std::uniform_int_distribution<int> distribution(1, 255);

	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud(
		new pcl::PointCloud <pcl::PointXYZRGB>);
	for(const auto& indices : clusters) {
		const int r = distribution(generator);
		const int g = distribution(generator);
		const int b = distribution(generator);
		for(int ix : indices.indices) {
			auto pt = cloud->points[ix];
			pt.r = r;
			pt.g = g;
			pt.b = b;
			colored_cloud->points.push_back(pt);
		}
	}

	bundle.addDebugPointCloud("second_clusters", colored_cloud);
}


// Apply affine transform to given XYZ+RGB+Normal point cloud,
// and return new transformed cloud.
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr applyTransform(
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud,
		const Eigen::Affine3f& trans) {
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
	for(auto& pt : cloud->points) {
		pcl::PointXYZRGBNormal pt_new = pt;
		pt_new.getVector3fMap() = trans * pt.getVector3fMap();
		pt_new.getNormalVector3fMap() = trans.rotation() * pt.getNormalVector3fMap();
		new_cloud->points.push_back(pt_new);
	}
	return new_cloud;
}

TexturedMesh bakeTexture(
		const AlignedScans& scans,
		const TriangleMesh<std::nullptr_t>& shape_wo_uv) {
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
				const cv::Vec3b color = scan.er_rgb(y, x);
				if(color == cv::Vec3b(0, 0, 0)) {
					continue;
				}
				n_all++;

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

				const float accept_dist = 0.1;
				const auto isect = intersecter.intersect(ray);
				if(isect) {
					const float t = std::get<2>(*isect);
					if(std::abs(t - scan.er_depth(y, x)) >= accept_dist) {
						continue;
					}
					const auto tri = shape.triangles[std::get<0>(*isect)];
					const auto uv0 = shape.vertices[std::get<0>(tri)].second;
					const auto uv1 = shape.vertices[std::get<1>(tri)].second;
					const auto uv2 = shape.vertices[std::get<2>(tri)].second;
					const auto uv = std::get<1>(*isect)(0) * (uv1 - uv0) + std::get<1>(*isect)(1) * (uv2 - uv0) + uv0;

					film.record(swapY(uv) * tex_size, color);
					n_hits++;
				}
			}
		}
		// Exit with only one scan, since currently multiple-scan fusion
		// results in
		// 1. ghosting due to misalignment (or calibration error)
		// 2. color artifact (exposure / color balance difference)
		// break;
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
	const Eigen::Vector3f center = cloud_base::append(center_2d, height / 2 + z_range.first);

	const auto box_mesh = shape_fitter::createBox(
		center,
		cloud_base::append(edge_d, 0) / 2,
		cloud_base::append(edge_n, 0) * (depth / 2),
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

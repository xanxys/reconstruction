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
#include <jsoncpp/json/json.h>
#include <opencv2/opencv.hpp>
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

#include <math_util.h>
#include <program_proxy.h>
#include <recog/shape_fitter.h>
#include <visual/cloud_baker.h>
#include <visual/cloud_base.h>
#include <visual/cloud_filter.h>
#include <visual/film.h>
#include <visual/mapping.h>
#include <visual/mesh_intersecter.h>
#include <visual/texture_conversion.h>

namespace visual {

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
	quad.triangles.push_back({{0, 1, 2}});
	quad.triangles.push_back({{2, 3, 0}});

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

RoomFrame::RoomFrame() {
	up = Eigen::Vector3f(0, 0, 1);
}

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

	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
	bundle.addDebugPointCloud("clusters", colored_cloud);

}

void splitEachScan(
		SceneAssetBundle& bundle, CorrectedSingleScan& ccs, RoomFrame& rframe) {
	INFO("Recognizing single scan", ccs.raw_scan.getScanId());
	// Fit polygon to ccs.

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
	const auto extrusion = shape_fitter::fitExtrudedPolygon(cloud_colorless);
	//auto room_mesh = std::get<0>(extrusion);
	auto room_polygon = std::get<0>(extrusion);
	auto room_hrange = std::get<1>(extrusion);
	//auto room_ceiling_ixs = std::get<3>(extrusion);

	Json::Value fc_arg;
	Json::Value poly_json;
	for(const auto& pt : room_polygon) {
		Json::Value pt_json;
		pt_json.append(pt(0));
		pt_json.append(pt(1));
		poly_json.append(pt_json);
	}
	fc_arg["points"] = poly_json;
	if(bundle.isDebugEnabled()) {
		fc_arg["vis_path"] = bundle.reservePath("contour.png");
	}
	const Json::Value contour_result = call_external("extpy/fix_contour.py", fc_arg);
	std::vector<Eigen::Vector2f> contour_points;
	for(const auto& pt_json : contour_result["points"]) {
		contour_points.emplace_back(
			pt_json[0].asDouble(), pt_json[1].asDouble());
	}

	RoomFrame rframe;
	rframe.wall_polygon = contour_points;
	for(auto& ccs :  scans_aligned.getScansWithPose()) {
		splitEachScan(bundle, ccs, rframe);
	}
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
	// Create offseted polygon to increase robustness.
	const K::FT offset = 0.2;
	const auto polys = CGAL::create_exterior_skeleton_and_offset_polygons_2(
		offset, room_polygon_cgal);
	assert(polys.size() == 2);
	const auto& room_polygon_larger = *polys[1];  // it seems like the second one is offsetted polygon.
	for(const auto& pt3 : points_merged->points) {
		Eigen::Vector2f d(pt3.x, pt3.y);
		const Point pt(pt3.x, pt3.y);
		if(room_polygon_larger.bounded_side(pt) != CGAL::ON_UNBOUNDED_SIDE) {
			points_inside->points.push_back(pt3);
		} else {
			points_outside->points.push_back(pt3);
		}
	}
	bundle.addDebugPointCloud("points_outside", points_outside);

	INFO("Materializing extruded polygon mesh");
	//const auto extrusion_refined = shape_fitter::fitExtrusion(cloud_base::cast<pcl::PointXYZRGBNormal, pcl::PointXYZ>(points_inside));
	const auto extrusion_mesh = shape_fitter::generateExtrusion(
		rframe.wall_polygon, room_hrange);
	const auto room_mesh = std::get<0>(extrusion_mesh);
	const auto room_ceiling_ixs = std::get<1>(extrusion_mesh);

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

	auto filtered = cloud_filter::squashRegistrationError(cloud_interior);
	bundle.addDebugPointCloud("filtered", filtered);

	const auto cloud_interior_dist = visual::cloud_baker::colorPointsByDistance<pcl::PointXYZRGBNormal>(points_inside, room_mesh, false);
	bundle.addDebugPointCloud("points_interior_distance", cloud_interior_dist);

	INFO("Creating assets");
	const auto exterior = recognizeExterior(
		scans_aligned, room_mesh, room_ceiling_ixs,
		cloud_base::cast<pcl::PointXYZRGBNormal, pcl::PointXYZRGB>(points_inside));
	bundle.point_lights = exterior.second;
	bundle.exterior_mesh = exterior.first;

	INFO("Splitting objects");
	splitObjects(bundle, filtered, scans_aligned);
}

std::pair<TexturedMesh, std::vector<Eigen::Vector3f>>
	recognizeExterior(
		const AlignedScans& scans_aligned,
		const TriangleMesh<std::nullptr_t>& room_mesh,
		const std::vector<int>& ceiling_ixs,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_inside) {

	// TODO: desirable pipeline if similar process' needs arise elsewhere.
	// partial polygons -> texture region 2D mask + XYZ mapping + normals etc.
	// reverse lookup.
	// Maybe overly complex??

	// Just calculate AABB of ceiling and project to it is enough?
		/*
	Eigen::Vector3f aabb_min(1e3, 1e3, 1e3);
	Eigen::Vector3f aabb_max(-1e3, -1e3, -1e3);
	for(const int ix_tri : ceiling_ixs) {
		const auto ix_verts = room_mesh.triangles[ix_tri];
		aabb_min = aabb_min.cwiseMin(
			room_mesh.vertices[std::get<0>(ix_verts)].first);
		aabb_max = aabb_max.cwiseMax(
			room_mesh.vertices[std::get<0>(ix_verts)].first);
		aabb_min = aabb_min.cwiseMin(
			room_mesh.vertices[std::get<1>(ix_verts)].first);
		aabb_max = aabb_max.cwiseMax(
			room_mesh.vertices[std::get<1>(ix_verts)].first);
			aabb_min = aabb_min.cwiseMin(
			room_mesh.vertices[std::get<2>(ix_verts)].first);
		aabb_max = aabb_max.cwiseMax(
			room_mesh.vertices[std::get<2>(ix_verts)].first);
	}
	// the ceiling must be perpendicular to Z plane.
	assert(std::abs((aabb_max - aabb_min).z()) < 1e-3);
	
	*/
	auto tex_mesh = bakeTexture(scans_aligned, room_mesh, 0.4);

	// TODO: proper ceiling texture extraction.
	return std::make_pair(
		tex_mesh,
		visual::recognize_lights(cloud_inside));
}


void splitObjects(
		SceneAssetBundle& bundle,
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_org,
		const AlignedScans& scans) {
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
	ec.setMaxClusterSize(10000000);  // 100000: most small objects / 500000: everything incl. tabgles
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);

	std::vector<pcl::PointIndices> clusters;
	ec.extract(clusters);

	INFO("Number of clusters=", (int)clusters.size());
	INFO("Size of 1st cluster", (int)clusters[0].indices.size());

	// create polyhedron.
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
			mesh.triangles.push_back({{v0, v0 + 1, v0 + 2}});
		}

		// bake texture
		//const auto tex_mesh = visual::cloud_baker::bakePointsToMesh(cloud, mesh);
		const auto tex_mesh = bakeTexture(scans, mesh);
		bundle.addInteriorObject(tex_mesh);
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


int ceilToPowerOf2(int x) {
	int r = 1;
	while(r <= x) {
		r *= 2;
	}
	return r;
}


TexturedMesh bakeTexture(
		const AlignedScans& scans,
		const TriangleMesh<std::nullptr_t>& shape_wo_uv,
		const float accept_dist) {
	// Calculate good texture size.
	// * must be pow of 2 (not mandatory, but for efficiency)
	// * texture pixel area \propto actual area
	const float area = shape_wo_uv.area();
	const int px_raw = std::sqrt(area) * 300;
	const int tex_size = ceilToPowerOf2(px_raw);
	FilmRGB8U film(tex_size, tex_size, 1.0);
	INFO("Choosing texture size v. real area", tex_size, area);

	TriangleMesh<Eigen::Vector2f> shape = mapSecond(assignUV(shape_wo_uv));
	MeshIntersecter intersecter(shape_wo_uv);
	INFO("Baking texture to mesh with #tri=", (int)shape.triangles.size());
	int n_hits = 0;
	int n_all = 0;
	for(const auto& corrected_scan : scans.getScansWithPose()) {
		const auto scan = corrected_scan.raw_scan;
		const auto l_to_w = corrected_scan.local_to_world;
		for(int y : boost::irange(0, scan.er_rgb.rows)) {
			for(int x : boost::irange(0, scan.er_rgb.cols)) {
				// Ignore N/A samples.
				// TODO: 0,0,0 rarely occurs naturaly, but when it does,
				// consider migration to RGBA image.
				const cv::Vec3b raw_color = scan.er_rgb(y, x);
				if(raw_color == cv::Vec3b(0, 0, 0)) {
					continue;
				}
				cv::Vec3f color = raw_color;  // BGR
				color[0] *= corrected_scan.color_multiplier(2);
				color[1] *= corrected_scan.color_multiplier(1);
				color[2] *= corrected_scan.color_multiplier(0);
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

}  // namespace
}  // namespace

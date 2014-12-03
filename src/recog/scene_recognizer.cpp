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
// insanity continues (3d tri mesh + AABB)
#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>
// insanity ends here
#include <Eigen/QR>
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

#include <extpy.h>
#include <math_util.h>
#include <optimize/gradient_descent.h>
#include <recog/shape_fitter.h>
#include <visual/cloud_baker.h>
#include <visual/cloud_base.h>
#include <visual/cloud_filter.h>
#include <visual/film.h>
#include <visual/mapping.h>
#include <visual/mesh_intersecter.h>
#include <visual/texture_conversion.h>


namespace recon {

std::vector<Eigen::Vector3f> recognize_lights(
		SceneAssetBundle& bundle,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
	// Calculate approximate ceiling height.
	std::vector<float> zs;
	for(const auto& pt : cloud->points) {
		zs.push_back(pt.z);
	}
	const auto zs_range = robustMinMax(zs);
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
	const TexturedMesh ceiling_geom = bakePointsToMesh(cloud, quad);
	cv::Mat ceiling_gray;
	cv::cvtColor(ceiling_geom.diffuse, ceiling_gray, cv::COLOR_BGR2GRAY);
	cv::GaussianBlur(ceiling_gray, ceiling_gray, cv::Size(31, 31), 10);
	if(bundle.isDebugEnabled()) {
		cv::imwrite(
			bundle.reservePath("ceiling_raw.png"), ceiling_geom.diffuse);
		cv::imwrite(
			bundle.reservePath("ceiling_processed.png"), ceiling_gray);
	}

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

RoomFrame::RoomFrame() {
	up = Eigen::Vector3f(0, 0, 1);
}

void RoomFrame::setHRange(float z0, float z1) {
	assert(z0 < z1);
	this->z0 = z0;
	this->z1 = z1;
}

std::pair<float, float> RoomFrame::getHRange() const {
	return std::make_pair(z0, z1);
}

std::vector<Eigen::Vector2f> RoomFrame::getSimplifiedContour() const {
	assert(wall_polygon.size() >= 3);
	return wall_polygon;
}

void splitEachScan(
		SceneAssetBundle& bundle, CorrectedSingleScan& ccs, RoomFrame& rframe) {
	using K = CGAL::Simple_cartesian<float>;
	using Point = K::Point_3;
	using Triangle = K::Triangle_3;
	using Iterator = std::vector<Triangle>::iterator;
	using Primitive = CGAL::AABB_triangle_primitive<K, Iterator>;
	using AABB_triangle_traits = CGAL::AABB_traits<K, Primitive>;
	using Tree = CGAL::AABB_tree<AABB_triangle_traits>;

	INFO("Recognizing single scan", ccs.raw_scan.getScanId());
	// Generate mesh that represents the wall. (with vertex normals)
	const auto contour = rframe.getSimplifiedContour();
	TriangleMesh<std::nullptr_t> wrapping =
		std::get<0>(
			generateExtrusion(contour, rframe.getHRange()));

	auto v3_to_point = [](const Eigen::Vector3f& v) {
		return Point(v(0), v(1), v(2));
	};

	auto point_to_v3 = [](const Point& p) {
		return Eigen::Vector3f(p.x(), p.y(), p.z());
	};

	auto apply_param = [&](const Eigen::VectorXf& param) {
		TriangleMesh<std::nullptr_t> wrapping_adj = wrapping;
		for(const int i : boost::irange(0, (int)wrapping.vertices.size())) {
			wrapping_adj.vertices[i].first += Eigen::Vector3f(
					param(i * 3 + 0), param(i * 3 + 1), 0);
		}
		return wrapping_adj;
	};

	// Free parameters: vertex displacements.
	// Cost function: avg. squared distance to points
	// Cut off at 3sigma (99.6%) of non-systematic, Gaussian error.
	const float dist_sigma = 0.02;  // Spec of UTM-30LX says 3cm error bounds for <10m.
	const float dist_thresh = 0.3;  // largest syetematic error (distortion)

	const auto cl_world = ccs.getCloudInWorld();
	auto eval = [&](const Eigen::VectorXf& param) {
		std::vector<Eigen::Vector3f> verts_adj;
		for(const int i : boost::irange(0, (int)wrapping.vertices.size())) {
			verts_adj.push_back(
				wrapping.vertices[i].first + Eigen::Vector3f(
					param(i * 3 + 0), param(i * 3 + 1), 0));
		}

		std::vector<Triangle> tris;
		tris.reserve(wrapping.triangles.size());
		for(const auto& tri : wrapping.triangles) {
			tris.emplace_back(
				v3_to_point(verts_adj[tri[0]]),
				v3_to_point(verts_adj[tri[1]]),
				v3_to_point(verts_adj[tri[2]]));
		}
		Tree tree(tris.begin(), tris.end());
		tree.accelerate_distance_queries();

		float cost = 0;
		Eigen::VectorXf grad(param.size());
		grad.setConstant(0);
		for(const auto& pt : cl_world->points) {
			const Eigen::Vector3f query = pt.getVector3fMap();
			const auto result = tree.closest_point_and_primitive(v3_to_point(query));
			const auto isect = point_to_v3(result.first);

			const float dist = (pt.getVector3fMap() - isect).norm();
			const Eigen::Vector3f dcost_disect = 2 * (isect - query);

			// When isect = t0 * v0 + t1 * v1 + t2 *v2,
			// d(cost)/d(isect) = t0 * d(cost) / d(v0) + ...
			Eigen::Matrix3f vs;
			const int tri_ix = std::distance(tris.begin(), result.second);
			assert(0 <= tri_ix && tri_ix < wrapping.triangles.size());
			vs.col(0) = verts_adj[wrapping.triangles[tri_ix][0]];
			vs.col(1) = verts_adj[wrapping.triangles[tri_ix][1]];
			vs.col(2) = verts_adj[wrapping.triangles[tri_ix][2]];
			const Eigen::Vector3f ts = vs.colPivHouseholderQr().solve(isect);

			// accumulate
			cost += std::pow(std::min(dist, dist_thresh), 2);
			for(int i : boost::irange(0, 3)) {
				const int vert_ix = wrapping.triangles[tri_ix][i];
				grad.segment<3>(vert_ix * 3) += ts(i) * dcost_disect;
			}
		}
		DEBUG("cost=", cost, " |grad|=", grad.norm());
		return std::make_pair(cost, grad);
	};

	const int dof = 3 * wrapping.vertices.size();

	INFO("Optimizing mesh / DoF=", dof);
	Eigen::VectorXf param(dof);
	param.setConstant(0);
	// step coefficient very important for convergence.
	// WARNING: this creates implicit coupling with |gradient|.
	// You can no longer multiply constant to cost freely.
	const auto result = minimize_gradient_descent(eval, param, 20, 1e-6);

	INFO("optimized cost=", result.second);

	if(bundle.isDebugEnabled()) {
		const std::string prefix = "debug_ccsrf_" + ccs.raw_scan.getScanId();
		bundle.addDebugPointCloud(prefix + "_world", cl_world);
		bundle.addMesh(prefix + "_pre", wrapping);
		bundle.addMesh(prefix + "_post", apply_param(result.first));
		// label
		const auto mesh_adj = apply_param(result.first);
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cl_world_color_labeled(
			new pcl::PointCloud<pcl::PointXYZRGBNormal>());
		cl_world_color_labeled->points = cl_world->points;
		{
			std::vector<Triangle> tris;
			tris.reserve(mesh_adj.triangles.size());
			for(const auto& tri : mesh_adj.triangles) {
				tris.emplace_back(
					v3_to_point(mesh_adj.vertices[tri[0]].first),
					v3_to_point(mesh_adj.vertices[tri[1]].first),
					v3_to_point(mesh_adj.vertices[tri[2]].first));
			}
			Tree tree(tris.begin(), tris.end());
			tree.accelerate_distance_queries();

			for(auto& pt : cl_world_color_labeled->points) {
				const float dist = std::sqrt(tree.squared_distance(v3_to_point(pt.getVector3fMap())));


				pt.r = (dist < dist_sigma * 3) ? 255 : 0;
				pt.g = std::min(255, static_cast<int>(dist * 1000));
				pt.b = 0;
			}
		}
		bundle.addDebugPointCloud(prefix + "_labels", cl_world_color_labeled);
	}
	// Wiggle mesh so that points will be close to faces.
	// A vertex can move around more freely in normal direction.


}

std::pair<
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr,
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> splitInOut(
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr points_merged,
		const std::vector<Eigen::Vector2f>& room_polygon) {
	auto points_inside = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	auto points_outside = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	using K = CGAL::Exact_predicates_inexact_constructions_kernel;
	using Point = K::Point_2;
	using Polygon_2 = CGAL::Polygon_2<K>;
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
	return std::make_pair(points_inside, points_outside);
}

void recognizeScene(SceneAssetBundle& bundle, const std::vector<SingleScan>& scans) {
	assert(!scans.empty());

	INFO("Merging points in multiple scans");
	const AlignedScans scans_aligned(bundle, scans);
	const auto points_merged = scans_aligned.getMergedPointsNormal();
	bundle.addDebugPointCloud("points_merged", points_merged);
	INFO("# of points after merge:", (int)points_merged->points.size());

	INFO("Approximating exterior shape by an extruded polygon");
	const auto cloud_colorless = cast<pcl::PointXYZRGBNormal, pcl::PointXYZ>(points_merged);
	const auto extrusion = fitExtrudedPolygon(cloud_colorless);
	auto room_polygon = std::get<0>(extrusion);
	auto room_hrange = std::get<1>(extrusion);

	const std::vector<Eigen::Vector2f> contour_points = extFixContour(
		room_polygon,
		bundle.isDebugEnabled() ?
			boost::optional<std::string>(bundle.reservePath("contour.png")) :
			boost::none);

	RoomFrame rframe;
	rframe.setHRange(room_hrange.first, room_hrange.second);
	rframe.wall_polygon = contour_points;
	for(auto& ccs :  scans_aligned.getScansWithPose()) {
		splitEachScan(bundle, ccs, rframe);
	}

	INFO("Splitting inside/outside");
	auto points_inout = splitInOut(points_merged, room_polygon);
	auto points_inside = points_inout.first;
	auto points_outside = points_inout.second;
	bundle.addDebugPointCloud("points_outside", points_outside);

	INFO("Materializing extruded polygon mesh");
	const auto extrusion_mesh = generateExtrusion(
		rframe.wall_polygon, room_hrange);
	const auto room_mesh = std::get<0>(extrusion_mesh);
	const auto room_ceiling_ixs = std::get<1>(extrusion_mesh);

	INFO("Modeling boxes along wall");
	auto cloud_interior_pre = colorPointsByDistance<pcl::PointXYZRGBNormal>(points_inside, room_mesh, true);
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

	auto filtered = squashRegistrationError(cloud_interior);
	bundle.addDebugPointCloud("filtered", filtered);

	const auto cloud_interior_dist = colorPointsByDistance<pcl::PointXYZRGBNormal>(points_inside, room_mesh, false);
	bundle.addDebugPointCloud("points_interior_distance", cloud_interior_dist);

	INFO("Creating assets");
	const auto exterior = recognizeExterior(
		bundle,
		scans_aligned, room_mesh, room_ceiling_ixs,
		cast<pcl::PointXYZRGBNormal, pcl::PointXYZRGB>(points_inside));
	bundle.point_lights = exterior.second;
	bundle.setExteriorMesh(exterior.first);

	INFO("Splitting objects");
	splitObjects(bundle, filtered, scans_aligned);
}

std::pair<TexturedMesh, std::vector<Eigen::Vector3f>>
	recognizeExterior(
		SceneAssetBundle& bundle,
		const AlignedScans& scans_aligned,
		const TriangleMesh<std::nullptr_t>& room_mesh,
		const std::vector<int>& ceiling_ixs,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_inside) {

	// TODO: desirable pipeline if similar process' needs arise elsewhere.
	// partial polygons -> texture region 2D mask + XYZ mapping + normals etc.
	// reverse lookup.
	// Maybe overly complex??
	auto tex_mesh = bakeTexture(scans_aligned, room_mesh, 0.4);

	// TODO: proper ceiling texture extraction.
	return std::make_pair(
		tex_mesh,
		recognize_lights(bundle, cloud_inside));
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
	auto cloud = cast<pcl::PointXYZRGBNormal, pcl::PointXYZRGB>(cloud_org);
	auto normals = cast<pcl::PointXYZRGBNormal, pcl::Normal>(cloud_org);

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

	if(bundle.isDebugEnabled()) {
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

	// create a polyhedron for each cluster.
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

		TriangleMesh<std::nullptr_t> mesh;
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
		const auto tex_mesh = bakeTexture(scans, mesh);
		bundle.addInteriorObject(tex_mesh);
	}
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

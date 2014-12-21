#include "scene_recognizer.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/optional.hpp>
#include <boost/range/irange.hpp>
// Unfortunately, CGAL headers are sensitive to include orders.
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_triangle_primitive.h>
#include <CGAL/algorithm.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/ch_graham_andrew.h>
#include <CGAL/convex_hull_3.h>
#include <CGAL/create_offset_polygons_2.h>
#include <CGAL/point_generators_3.h>
#include <CGAL/Surface_mesh_default_triangulation_3.h>
#include <CGAL/Complex_2_in_triangulation_3.h>
#include <CGAL/make_surface_mesh.h>
#include <CGAL/Implicit_surface_3.h>
#include <CGAL/IO/output_surface_facets_to_polyhedron.h>
#include <CGAL/boost/graph/graph_traits_Polyhedron_3.h>
#include <CGAL/Surface_mesh_simplification/HalfedgeGraph_Polyhedron_3.h>
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_ratio_stop_predicate.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Edge_length_cost.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Midpoint_placement.h>
// insanity ends here
#include <Eigen/QR>
#include <opencv2/opencv.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/poisson.h>

#include <extpy.h>
#include <geom/simplification.h>
#include <geom/triangulation.h>
#include <graph/util.h>
#include <math_util.h>
#include <optimize/gradient_descent.h>
#include <range2.h>
#include <recog/shape_fitter.h>
#include <scidata/field.h>
#include <visual/cloud_baker.h>
#include <visual/cloud_base.h>
#include <visual/cloud_filter.h>
#include <visual/film.h>
#include <visual/mapping.h>
#include <visual/texture_conversion.h>

namespace recon {

std::vector<Eigen::Vector3f> recognizeLights(
		SceneAssetBundle& bundle,
		const RoomFrame& rframe,
		const AlignedScans& scans_aligned,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
	assert(!rframe.wall_polygon.empty());
	// Calculate 3D quad of ceiling from rframe.
	Eigen::Vector2f vmin(1e10, 1e10);
	Eigen::Vector2f vmax = -vmin;
	for(const auto& v : rframe.wall_polygon) {
		vmin = vmin.cwiseMin(v);
		vmax = vmax.cwiseMax(v);
	}
	const float z_ceiling = rframe.getHRange().second;
	const Eigen::Vector3f normal_ceiling(0, 0, -1);

	// Projection settings.
	const float px_per_meter_low = 10;  // used many times for quality evaluation
	const float px_per_meter = 100;

	// Project points to the ceiling quad.
	// TODO: need to occluding points.
	INFO("Detecting lights at z =", z_ceiling);

	// Choose best scan by quality.
	const auto ccs = scans_aligned.getScansWithPose();
	assert(ccs.size() > 0);
	auto calc_scan_quality_for_ceiling = [&](const CorrectedSingleScan& scan) {
		const int width = (vmax - vmin).x() * px_per_meter_low;
		const int height = (vmax - vmin).y() * px_per_meter_low;

		const Eigen::Affine3f world_to_local = scan.local_to_world.inverse();
		cv::Mat quality(height, width, CV_32F);
		for(const int y : boost::irange(0, height)) {
			for(const int x : boost::irange(0, width)) {
				const auto pt2d_w = Eigen::Vector2f(x, y) / px_per_meter_low + vmin;
				const Eigen::Vector3f pt3d_w(pt2d_w.x(), pt2d_w.y(), z_ceiling);
				const Eigen::Vector3f pt3d_l = world_to_local * pt3d_w;
				const Eigen::Vector3f pt3d_l_n = pt3d_l / pt3d_l.norm();

				// smaller is better. (<0 is invalid, though)
				const float length_per_angle = -pt3d_l.norm() / pt3d_l_n.dot(normal_ceiling);
				quality.at<float>(y, x) = length_per_angle;
			}
		}
		if(bundle.isDebugEnabled()) {
			cv::imwrite(
				bundle.reservePath("ceiling_quality_" + scan.raw_scan.getScanId() + ".png"),
				quality);
		}
		return cv::mean(quality)[0];
	};
	std::vector<float> quals;
	for(const auto& scan : ccs) {
		quals.push_back(calc_scan_quality_for_ceiling(scan));
	}
	const auto q_mm = std::minmax_element(quals.begin(), quals.end());
	INFO("Quality: best=", *q_mm.first, "worst=", *q_mm.second);

	const int best_scan_ix = std::distance(quals.begin(),
		std::min_element(quals.begin(), quals.end()));
	const auto& scan = ccs[best_scan_ix];
	INFO("Choosing ceiling texture base scan id=", scan.raw_scan.getScanId());

	const Eigen::Affine3f world_to_local = scan.local_to_world.inverse();
	const int width = (vmax - vmin).x() * px_per_meter;
	const int height = (vmax - vmin).y() * px_per_meter;
	cv::Mat mapping(height, width, CV_32FC2);
	cv::Mat quality(height, width, CV_32F);
	for(const int y : boost::irange(0, height)) {
		for(const int x : boost::irange(0, width)) {
			const auto pt2d_w = Eigen::Vector2f(x, y) / px_per_meter + vmin;
			const Eigen::Vector3f pt3d_w(pt2d_w.x(), pt2d_w.y(), z_ceiling);
			const Eigen::Vector3f pt3d_l = world_to_local * pt3d_w;
			const Eigen::Vector3f pt3d_l_n = pt3d_l / pt3d_l.norm();

			// smaller is better. (<0 is invalid, though)
			const float length_per_angle = -pt3d_l.norm() / pt3d_l_n.dot(normal_ceiling);

			const float theta = std::acos(pt3d_l_n.z());
			const float phi = -std::atan2(pt3d_l_n.y(), pt3d_l_n.x());
			const float phi_pos = (phi > 0) ? phi : (phi + 2 * pi);

			const float er_x = scan.raw_scan.er_rgb.cols * phi_pos / (2 * pi);
			const float er_y = scan.raw_scan.er_rgb.rows * theta / pi;
			mapping.at<cv::Vec2f>(y, x) = cv::Vec2f(er_x, er_y);
			quality.at<float>(y, x) = length_per_angle;
		}
	}


	cv::Mat proj_new;
	cv::remap(scan.raw_scan.er_rgb, proj_new, mapping, cv::Mat(),
		cv::INTER_LINEAR, cv::BORDER_REPLICATE);

	// Make it grayscale.
	cv::Mat ceiling_gray;
	cv::cvtColor(proj_new, ceiling_gray, cv::COLOR_BGR2GRAY);
	if(bundle.isDebugEnabled()) {
		cv::imwrite(
			bundle.reservePath("ceiling_quality.png"), visualize_field2(quality));
		cv::imwrite(
			bundle.reservePath("ceiling_gray.png"), ceiling_gray);
		cv::imwrite(
			bundle.reservePath("ceiling.png"), proj_new);
	}

	// Detect blobs (saturated lights).
	const float light_margin = 0.05;  // we need slight margin between light and ceiling.
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

		const auto pt2d = Eigen::Vector2f(blob.pt.x, blob.pt.y) / px_per_meter + vmin;
		const Eigen::Vector3f pt3d(pt2d.x(), pt2d.y(), z_ceiling - light_margin);
		lights.push_back(pt3d);
	}
	return lights;
}

std::vector<MiniCluster> splitEachScan(
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
		ExtrudedPolygonMesh(contour, rframe.getHRange()).getMesh();

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

	// We need to use huge threshold after all;
	// fitting to correct wall, instead of some other objects
	// is hard. Only (probably) slightly better
	// than merged ones.
	const float thresh_label = 0.15;

	const std::string prefix = "debug_ccsrf_" + ccs.raw_scan.getScanId();
	bundle.addDebugPointCloud(prefix + "_world", cl_world);
	bundle.addMesh(prefix + "_pre", wrapping);
	bundle.addMesh(prefix + "_post", apply_param(result.first));
	// label
	const auto mesh_adj = apply_param(result.first);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cl_world_color_labeled(
		new pcl::PointCloud<pcl::PointXYZRGBNormal>());
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cl_world_interior(
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
			if(dist >= thresh_label) {
				cl_world_interior->points.push_back(pt);
			}
			pt.r = (dist < thresh_label) ? 255 : 0;
			pt.g = std::min(255, static_cast<int>(dist * 1000));
			pt.b = 0;
		}
	}
	if(bundle.isDebugEnabled()) {
		bundle.addDebugPointCloud(prefix + "_labels", cl_world_color_labeled);
	}

	std::vector<MiniCluster> mini_clusters;
	{
		using K = CGAL::Exact_predicates_inexact_constructions_kernel;
		using Point_3 = K::Point_3;
		using Polyhedron_3 = CGAL::Polyhedron_3<K>;

		pcl::search::Search<pcl::PointXYZRGB>::Ptr tree =
			boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB>>(
				new pcl::search::KdTree<pcl::PointXYZRGB>);

		// decompose to XYZRGB + Normal
		auto cloud = cast<pcl::PointXYZRGBNormal, pcl::PointXYZRGB>(cl_world_interior);
		auto normals = cast<pcl::PointXYZRGBNormal, pcl::Normal>(cl_world_interior);

		INFO("splitEachScan: Doing EC");
		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
		ec.setClusterTolerance(0.02); // 2cm
		ec.setMinClusterSize(100);
		ec.setMaxClusterSize(10000000);  // 100000: most small objects / 500000: everything incl. tabgles
		ec.setSearchMethod(tree);
		ec.setInputCloud(cloud);

		std::vector<pcl::PointIndices> clusters;
		ec.extract(clusters);
		INFO("splitEachScan: Number of clusters=", (int)clusters.size());

		int i_cluster = 0;
		for(const auto& indices : clusters) {
			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cluster_cloud(
				new pcl::PointCloud<pcl::PointXYZRGBNormal>);
			for(const int ix : indices.indices) {
				cluster_cloud->points.push_back(cl_world_interior->points[ix]);
			}

			Eigen::Vector3f pos_accum = Eigen::Vector3f::Zero();
			Eigen::Vector3f normal_accum = Eigen::Vector3f::Zero();
			int n_accum = 0;
			for(const auto& pt : cluster_cloud->points) {
				pos_accum += pt.getVector3fMap();
				normal_accum += pt.getNormalVector3fMap();
				n_accum++;
			}
			assert(n_accum > 0);
			const Eigen::Vector3f pos_mean = pos_accum / n_accum;
			const Eigen::Vector3f normal_mean = normal_accum.normalized();
			// reject ceiling clusters (don't use cluster size as signal,
			// since some of them are small)
			if(pos_accum.z() >= rframe.getHRange().second - 0.3) {
				continue;
			}

			// Accept cluster
			mini_clusters.emplace_back(&ccs, cluster_cloud);
			i_cluster++;
		}

		if(bundle.isDebugEnabled()) {
			std::default_random_engine generator;
			std::uniform_int_distribution<int> distribution(1, 255);
			pcl::PointCloud <pcl::PointXYZRGBNormal>::Ptr colored_cloud(
				new pcl::PointCloud <pcl::PointXYZRGBNormal>);
			for(const auto& indices : clusters) {
				const int r = distribution(generator);
				const int g = distribution(generator);
				const int b = distribution(generator);
				for(int ix : indices.indices) {
					auto pt = cl_world_interior->points[ix];
					pt.r = r;
					pt.g = g;
					pt.b = b;
					colored_cloud->points.push_back(pt);
				}
			}
			bundle.addDebugPointCloud("ps_clusters_" + ccs.raw_scan.getScanId(), colored_cloud);
		}
	}
	return mini_clusters;
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


MiniCluster::MiniCluster(
		CorrectedSingleScan* c_scan,
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud) :
		c_scan(c_scan), cloud(cloud),
		aabb(calculateAABB(cloud)),
		is_supported(false), stable(false) {
	assert(c_scan);
	assert(cloud);
	assert(!cloud->points.empty());

	// Approx center of gravity.
	Eigen::Vector3f accum = Eigen::Vector3f::Zero();
	for(const auto& pt : cloud->points) {
		accum += pt.getVector3fMap();
	}
	grav_center = accum / cloud->points.size();
}

AABB3f MiniCluster::calculateAABB(
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud) {
	assert(cloud);
	Eigen::Vector3f vmin(1e3, 1e3, 1e3);
	Eigen::Vector3f vmax = -vmin;
	for(const auto& pt : cloud->points) {
		vmin = vmin.cwiseMin(pt.getVector3fMap());
		vmax = vmax.cwiseMax(pt.getVector3fMap());
	}
	return AABB3f(vmin, vmax);
}


MCLinker::MCLinker(
		SceneAssetBundle& bundle,
		const RoomFrame& rframe,
		const std::vector<MiniCluster>& raw_mcs) :
		mcs(raw_mcs), rframe(rframe), bundle(bundle) {
	INFO("Linking MiniCulsters N=", (int)mcs.size());

	// Pre-calculate pairwise properties.
	INFO("Calculating MiniCluster pairwise props.");
	const float dist_cutoff = 0.3;  // above this thresh, two MiniClusters MUST NOT CONNECT
	Eigen::MatrixXf pc_min_dist(mcs.size(), mcs.size());
	for(const MCId i : boost::irange(0, (int)mcs.size())) {
		const auto enl_aabb = mcs[i].aabb.enlarged(dist_cutoff);
		for(const MCId j : boost::irange(0, (int)mcs.size())) {
			if(i == j) {
				pc_min_dist(i, j) = 0;
				continue;
			} else if(i > j) {
				pc_min_dist(i, j) = pc_min_dist(j, i);
				continue;
			}

			if(enl_aabb.overlap(mcs[j].aabb)) {
				float d_min = 1e3;
				for(const auto& pi : mcs[i].cloud->points) {
					for(const auto& pj : mcs[j].cloud->points) {
						d_min = std::min(d_min,
							(pi.getVector3fMap() - pj.getVector3fMap()).norm());
					}
				}
				pc_min_dist(i, j) = d_min;
			} else {
				pc_min_dist(i, j) = dist_cutoff;
			}
		}
	}
	cluster_dist = pc_min_dist;
	if(bundle.isDebugEnabled()) {
		Json::Value root;
		for(const MCId i : boost::irange(0, (int)mcs.size())) {
			Json::Value row;
			for(const MCId j : boost::irange(0, (int)mcs.size())) {
				row.append(pc_min_dist(i, j));
			}
			root.append(row);
		}
		std::ofstream of(bundle.reservePath("mc_dist.json"));
		of << Json::FastWriter().write(root);
	}
}

std::vector<std::set<int>> MCLinker::getResult() {
	const MCId floor_id = -1;

	// Cluster by distance.
	const float cluster_thresh = 0.05;
	std::set<MCId> ids;
	for(const MCId id : boost::irange(0, (int)mcs.size())) {
		ids.insert(id);
	}
	std::map<MCId, std::set<MCId>> adj;
	for(const MCId i : ids) {
		for(const MCId j : ids) {
			if(cluster_dist(i, j) < cluster_thresh) {
				adj[i].insert(j);
			}
		}
	}
	std::vector<std::set<MCId>> ccs = getCC(ids, adj);
	INFO("#CCs: ", (int)ccs.size());

	// unstable variations:
	// supported (unstable due to CoG offset) ->
	// * search nearby conn
	// non-supported ->
	// * floor extension (add mesh)
	// * nearby conn
	while(true) {
		// Partition CCs by stability.
		std::vector<std::set<MCId>> stable_ccs;
		std::vector<std::set<MCId>> unstable_ccs;
		for(const auto& cc : ccs) {
			if(isStable(cc, 0.05)) {
				for(const auto& id : cc) {
					mcs[id].stable = true;
				}
				stable_ccs.push_back(cc);
			} else {
				unstable_ccs.push_back(cc);
			}
		}
		assert(stable_ccs.size() + unstable_ccs.size() == ccs.size());
		INFO("#Stable CC:", (int)stable_ccs.size(), "#All CC", (int)ccs.size());
		if(unstable_ccs.size() == 0) {
			break;
		}

		// Try to connect most natural unconnected CC.
		// <improve ccs>
		float action_cost = 1e10;
		std::pair<std::set<int>, std::set<int>> action;  // unstable, stable
		for(const auto& cc : unstable_ccs) {
			float min_dist_to_stable = 1e10;
			std::set<MCId> min_stable;
			for(const auto cc_stable : stable_ccs) {
				const float dist = distanceBetween(cc, cc_stable);
				if(dist >= min_dist_to_stable) {
					continue;
				}
				min_dist_to_stable = dist;
				min_stable = cc_stable;
			}
			assert(!min_stable.empty());  // something must be closest.

			// Does linking the two make them stable?
			const std::set<MCId> merge_hypothesis = join(cc, min_stable);
			if(!isStable(merge_hypothesis, 0.05)) {
				continue;
			}
			// Register new action.
			if(min_dist_to_stable < action_cost) {
				action_cost = min_dist_to_stable;
				action = std::make_pair(cc, min_stable);
			}
		}
		if(action_cost > 1000) {
			INFO("Action exhausted");
			break;
		}
		INFO("Taking join action w/ cost=", action_cost);
		ccs.erase(std::find(ccs.begin(), ccs.end(), action.first));
		ccs.erase(std::find(ccs.begin(), ccs.end(), action.second));
		ccs.push_back(join(action.first, action.second));
	}

	const auto groups = ccs;

	if(bundle.isDebugEnabled()) {
		Json::Value root;
		// clusters
		for(const int mc_id : boost::irange(0, (int)mcs.size())) {
			const auto& mc = mcs[mc_id];
			Json::Value mc_entry;

			Json::Value cloud;
			for(const auto& pt : mc.cloud->points) {
				Json::Value p;
				p["x"] = pt.x;
				p["y"] = pt.y;
				p["z"] = pt.z;
				p["r"] = pt.r;
				p["g"] = pt.g;
				p["b"] = pt.b;
				p["nx"] = pt.normal_x;
				p["ny"] = pt.normal_y;
				p["nz"] = pt.normal_z;
				cloud.append(p);
			}
			mc_entry["is_supported"] = mc.is_supported;
			mc_entry["support_z"] = mc.support_z;
			for(const auto& pt : mc.support_polygon) {
				Json::Value p;
				p["x"] = pt.x();
				p["y"] = pt.y();
				mc_entry["support_polygon"].append(p);
			}
			mc_entry["grav_center"]["x"] = mc.grav_center.x();
			mc_entry["grav_center"]["y"] = mc.grav_center.y();
			mc_entry["grav_center"]["z"] = mc.grav_center.z();
			mc_entry["stable"] = mc.stable;

			mc_entry["cloud"] = cloud;
			mc_entry["id"] = mc_id;
			root["clusters"].append(mc_entry);
		}
		// edges
		/*
		for(const auto& edge : parent) {
			Json::Value e;
			e.append(edge.first);
			e.append(edge.second);
			root["edges"].append(e);
		}
		for(const auto& edge : merging) {
			Json::Value e;
			e.append(edge.first);
			e.append(edge.second);
			root["merging"].append(e);
		}
		*/
		// groups
		for(const auto& group : groups) {
			Json::Value e;
			for(const auto id : group) {
				e.append(id);
			}
			root["groups"].append(e);
		}
		// rframe
		root["rframe"]["z0"] = rframe.getHRange().first;
		root["rframe"]["z1"] = rframe.getHRange().second;

		std::ofstream of(bundle.reservePath("link_mc.json"));
		of << Json::FastWriter().write(root);
	}

	// Filter stable CCS.
	std::vector<std::set<MCId>> stable_ccs;
	for(const auto& cc : ccs) {
		if(isStable(cc, 0.05)) {
			stable_ccs.push_back(cc);
		}
	}
	return stable_ccs;
}

auto MCLinker::join(
		const std::set<MCId>& a, const std::set<MCId>& b) -> std::set<MCId> {
	std::set<MCId> result = a;
	result.insert(b.begin(), b.end());
	return result;
}

bool MCLinker::isStable(const std::set<MCId>& cls,
		float disturbance) const {
	assert(disturbance >= 0);

	const auto support = getSupportPolygon(cls);
	if(!support) {
		return false;
	}

	// Check if cluster is stable.
	// stable == CG falls within polygon
	const std::vector<Eigen::Vector2f> noises = {
		{0, 0},
		{1, 0},
		{-1, 0},
		{0, 1},
		{0, -1}
	};
	const Eigen::Vector3f cog = getCG(cls);
	bool stable = true;
	for(const auto& noise : noises) {
		const Eigen::Vector2f cog_w_n = cog.head<2>() + noise * disturbance;
		const Point_2 cog_w_n_cgal(cog_w_n.x(), cog_w_n.y());
		stable &= (support->bounded_side(cog_w_n_cgal) == CGAL::ON_BOUNDED_SIDE);
	}
	return stable;
}

boost::optional<MCLinker::Polygon_2> MCLinker::getSupportPolygon(
		const std::set<MCId>& cl) const {
	const float z_floor = rframe.getHRange().first;
	const float z_thresh = 0.2;
	const float bond_radius = 0.02;

	// Get all points near floor.
	std::vector<Point_2> support_points;
	for(const auto& id : cl) {
		auto& mc = mcs[id];
		// Cull completely floating cluster.
		if(std::abs(mc.aabb.getMin().z() - z_floor) > z_thresh) {
			continue;
		}
		for(const auto& pt : mc.cloud->points) {
			if(std::abs(pt.z - mc.aabb.getMin().z()) < bond_radius) {
				support_points.emplace_back(pt.x, pt.y);
			}
		}
	}
	// Too few support points: no support polygon.
	if(support_points.size() < 3) {
		return boost::none;
	}
	std::vector<Point_2> support_vertices;
	CGAL::ch_graham_andrew(
		support_points.cbegin(), support_points.cend(),
		std::back_inserter(support_vertices));

	return MCLinker::Polygon_2(
		support_vertices.begin(), support_vertices.end());
}

float MCLinker::distanceBetween(MCId cl0, MCId cl1) const {
	return cluster_dist(cl0, cl1);
}

float MCLinker::distanceBetween(
		const std::set<MCId>& cls0,
		const std::set<MCId>& cls1) const {
	assert(!cls0.empty());
	assert(!cls1.empty());
	float dist = 1e10;
	for(const auto& cl0 : cls0) {
		for(const auto& cl1 : cls1) {
			dist = std::min(dist, cluster_dist(cl0, cl1));
		}
	}
	return dist;
}

Eigen::Vector3f MCLinker::getCG(const std::set<MCId>& cl) const {
	assert(!cl.empty());
	Eigen::Vector3f cog_accum = Eigen::Vector3f::Zero();
	float n_accum = 0;
	for(const auto& id : cl) {
		cog_accum += mcs[id].grav_center * mcs[id].cloud->points.size();
		n_accum += mcs[id].cloud->points.size();
	}
	return cog_accum / n_accum;
}

void recognizeScene(SceneAssetBundle& bundle,
		const std::vector<SingleScan>& scans,
		const Json::Value& hint) {
	assert(!scans.empty());

	INFO("Merging points in multiple scans");
	const AlignedScans scans_aligned(bundle, scans, hint);
	const auto points_merged = scans_aligned.getMergedPointsNormal();
	bundle.addDebugPointCloud("points_merged", points_merged);
	INFO("# of points after merge:", (int)points_merged->points.size());

	INFO("Approximating boundary shape by an extruded polygon");
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

	std::vector<MiniCluster> mcs;
	auto scans_with_pose = scans_aligned.getScansWithPose();  // required for lending scan pointer to MiniCluster
	for(auto& ccs : scans_with_pose) {
		const auto mcs_per_scan = splitEachScan(bundle, ccs, rframe);
		mcs.insert(mcs.end(), mcs_per_scan.begin(), mcs_per_scan.end());
	}
	INFO("#MiniCluster", (int)mcs.size());

	const auto extrusion_mesh = ExtrudedPolygonMesh(
		rframe.wall_polygon, room_hrange);

	// DEPRECATED: legacy merge->filter->split pipeline
	INFO("Splitting inside/outside");
	auto points_inout = splitInOut(points_merged, room_polygon);
	auto points_inside = points_inout.first;
	auto points_outside = points_inout.second;
	bundle.addDebugPointCloud("points_outside", points_outside);

	INFO("Linking miniclusters");
	const auto groups = MCLinker(bundle, rframe, mcs).getResult();

	INFO("Creating assets");
	const auto boundary = recognizeBoundary(
		bundle, rframe,
		scans_aligned, extrusion_mesh,
		cast<pcl::PointXYZRGBNormal, pcl::PointXYZRGB>(points_inside));
	bundle.setFloorLevel(rframe.getHRange().first);
	for(const auto& pos : boundary.second) {
		bundle.addPointLight(pos);
	}
	int i_group = 0;
	for(const auto& group : groups) {
		INFO("Processing group", i_group);
		bundle.addInteriorObject(createInteriorObject(
			bundle, mcs, group, std::to_string(i_group)));
		i_group++;
	}
	bundle.setInteriorBoundary(
		InteriorBoundary(
			boundary.first,
			rframe.wall_polygon,
			rframe.getHRange()));
}

InteriorObject createInteriorObject(
		SceneAssetBundle& bundle,
		const std::vector<MiniCluster>& mcs,
		const std::set<int>& group,
		const std::string& debug_id) {
	// Generate render proxy.
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
		new pcl::PointCloud<pcl::PointXYZRGB>);
	Eigen::Vector3f vmin(1e10, 1e10, 1e10);
	Eigen::Vector3f vmax = -vmin;
	for(const auto& mc_id : group) {
		for(const auto& pt : mcs[mc_id].cloud->points) {
			pcl::PointXYZRGB pn;
			pn.getVector3fMap() = pt.getVector3fMap();
			pn.r = pt.r;
			pn.g = pt.g;
			pn.b = pt.b;
			cloud->points.push_back(pn);
			assert(std::isfinite(pn.x));
			assert(std::isfinite(pn.y));
			assert(std::isfinite(pn.z));

			// Get AABB.
			vmin = vmin.cwiseMin(pt.getVector3fMap());
			vmax = vmax.cwiseMax(pt.getVector3fMap());
		}
	}
	pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
	kdtree.setInputCloud(cloud);

	auto field = [&kdtree](const Eigen::Vector3f& pos) {
		const float sigma = 0.02;

		pcl::PointXYZRGB query;
		query.getVector3fMap() = pos;

		std::vector<int> result_ixs;
		std::vector<float> result_sq_dists;
		const int n_result = kdtree.radiusSearch(
			query, sigma * 3,
			result_ixs, result_sq_dists);

		float v = 0;
		for(int i : boost::irange(0, n_result)) {
			v += std::exp(-result_sq_dists[i] / std::pow(sigma, 2));
		}
		return v;
	};

	auto color_field = [&kdtree, &cloud](const Eigen::Vector3f& pos) -> Eigen::Vector3f {
		const float sigma = 0.02;

		pcl::PointXYZRGB query;
		query.getVector3fMap() = pos;

		std::vector<int> result_ixs;
		std::vector<float> result_sq_dists;
		const int n_result = kdtree.radiusSearch(
			query, sigma * 3,
			result_ixs, result_sq_dists);

		float weight_accum = 0;
		Eigen::Vector3f rgb_accum = Eigen::Vector3f::Zero();
		for(int i : boost::irange(0, n_result)) {
			const float weight =
				std::exp(-result_sq_dists[i] / std::pow(sigma, 2));
			const auto& pt = cloud->points[result_ixs[i]];
			rgb_accum += weight *
				Eigen::Vector3f(pt.r, pt.g, pt.b);
			weight_accum += weight;
		}
		if(weight_accum < 1e-3) {
			return Eigen::Vector3f::Zero();
		} else {
			return rgb_accum / weight_accum;
		}
	};

	TriangleMesh<std::nullptr_t> simple_mesh;
	{
		// default triangulation for Surface_mesher
		using Tr = CGAL::Surface_mesh_default_triangulation_3;
		// c2t3
		using C2t3 = CGAL::Complex_2_in_triangulation_3<Tr>;
		using GT = Tr::Geom_traits;
		using Sphere_3 = GT::Sphere_3;
		using Point_3 = GT::Point_3;
		using FT = GT::FT;
		using Function = std::function<FT(const Point_3&)>;
		using Surface_3 = CGAL::Implicit_surface_3<GT, Function>;

		// Get point narest to the AABB center to specify proper
		// sphere bounds for CGAL.
		pcl::PointXYZRGB query;
		query.getVector3fMap() = (vmin + vmax) / 2;
		std::vector<int> pointIdxNKNSearch(1);
		std::vector<float> pointNKNSquaredDistance(1);
		kdtree.nearestKSearch(query, 1, pointIdxNKNSearch, pointNKNSquaredDistance);

		const auto center = cloud->points[pointIdxNKNSearch[0]].getVector3fMap();
		const float radius = (vmax - center).cwiseMax(center - vmin).norm();

		const auto sh_function = [&](const Point_3& pt) -> FT {
			const float thresh = 0.5;
			return thresh - field(Eigen::Vector3f(pt.x(), pt.y(), pt.z()));
		};

		Tr tr;
		C2t3 c2t3(tr);

		// defining the surface
		Surface_3 surface(
			sh_function,
			Sphere_3(
				Point_3(center.x(), center.y(), center.z()),
				radius * radius * 1.1));  // 1.1 is a safety margin

		// defining meshing criteria
		CGAL::Surface_mesh_default_criteria_3<Tr> criteria(
			30.,  // angular bound
			0.02,  // radius bound
			0.02); // distance bound

		// meshing surface
		CGAL::make_surface_mesh(c2t3, surface, criteria, CGAL::Manifold_with_boundary_tag());
		INFO("CGAL meshing: #vert=", (int)tr.number_of_vertices());

		CGAL::Polyhedron_3<CGAL::Epick> polyh;
		const bool sane = CGAL::output_surface_facets_to_polyhedron(c2t3, polyh);
		if(!sane) {
			WARN("Non-orientable");
		}

		// Simplify
		const double compression_ratio = 0.3;
		CGAL::Surface_mesh_simplification::Count_ratio_stop_predicate<
			CGAL::Polyhedron_3<CGAL::Epick>> stop(compression_ratio);

		//CGAL::Polyhedron_3<CGAL::Simple_cartesian<double>> polyh2 = polyh;

		CGAL::Surface_mesh_simplification::edge_collapse(polyh,
			stop,
			CGAL::vertex_index_map( boost::get(CGAL::vertex_external_index, polyh))
				.edge_index_map( boost::get(CGAL::edge_external_index, polyh  )));

		simple_mesh = surfaceToMesh(polyh);
	}

	const int tex_size = 512;
	TexturedMesh tm;
	tm.mesh = mapSecond(assignUV(simple_mesh));

	// Create RGB texture via XYZ texture.
	const Eigen::Vector3f invalid_pos(1e3, 1e3, 1e3);
	const auto pos_map = getPositionMapInUV(tm.mesh, tex_size,
		invalid_pos);

	cv::Mat diffuse(tex_size, tex_size, CV_8UC3);
	for(const int y : boost::irange(0, tex_size)) {
		for(const int x : boost::irange(0, tex_size)) {
			const auto pos_raw = pos_map.at<cv::Vec3f>(y, x);
			const auto pos = Eigen::Vector3f(pos_raw[0], pos_raw[1], pos_raw[2]);
			if(pos == invalid_pos) {
				// don't bother looking up, because field lookup is slow.
				diffuse.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0);
			} else {
				const auto col = color_field(pos);  // RGB
				diffuse.at<cv::Vec3b>(y, x) =
					cv::Vec3b(col(2), col(1), col(0));  // BGR
			}
		}
	}
	tm.diffuse = diffuse;

	// Generate collisions.
	std::vector<OBB3f> collisions;
	for(const auto& mc_id : group) {
		// TODO: finer collision.
		assert(mcs[mc_id].aabb.getVolume() > 0);
		collisions.push_back(OBB3f(mcs[mc_id].aabb));
	}
	if(bundle.isDebugEnabled()) {
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(
			new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		for(const auto& mc_id : group) {
			for(const auto& pt : mcs[mc_id].cloud->points) {
				cloud->points.push_back(pt);
			}
		}
		bundle.addDebugPointCloud("group_" + debug_id, cloud);
	}
	return InteriorObject(tm, collisions);
}

std::pair<TexturedMesh, std::vector<Eigen::Vector3f>>
	recognizeBoundary(
		SceneAssetBundle& bundle,
		const RoomFrame& rframe,
		const AlignedScans& scans_aligned,
		const ExtrudedPolygonMesh& ext_mesh,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_inside) {
	// Create boundary mesh (textured).
	auto boundary_tex = bakeBoundaryTexture(
		scans_aligned, ext_mesh.getMesh(), 0.4);
	auto tex_mesh = boundary_tex.first;
	const auto tex_xyz = boundary_tex.second;

	////
	// Cleanup floor mess.
	////
	// Create floor mask in UV via XYZ map.
	assert(tex_mesh.diffuse.rows == tex_mesh.diffuse.cols);
	const int tex_size = tex_mesh.diffuse.cols;
	const auto pos_map = getPositionMapInUV(tex_mesh.mesh, tex_size, Eigen::Vector3f(0, 0, 1e10));
	cv::Mat floor_mask(tex_size, tex_size, CV_8U);
	for(const int y : boost::irange(0, tex_size)) {
		for(const int x : boost::irange(0, tex_size)) {
			const bool is_floor =
				(std::abs(pos_map.at<cv::Vec3f>(y, x)[2] - rframe.getHRange().first) < 1e-3);
			floor_mask.at<uint8_t>(y, x) = is_floor ? 255 : 0;
		}
	}
	if(bundle.isDebugEnabled()) {
		cv::imwrite(
			bundle.reservePath("debug_floor.png"),
			tex_mesh.diffuse);
		cv::imwrite(
			bundle.reservePath("debug_floor_xyz_scan.png"),
			tex_xyz * 10 + 127);
		cv::imwrite(
			bundle.reservePath("debug_floor_xyz_model.png"),
			pos_map * 10 + 127);
		cv::imwrite(
			bundle.reservePath("debug_floor_region.png"),
			floor_mask);
	}
	// Create error mask.
	cv::Mat dist_error(tex_size, tex_size, CV_32F);
	for(const int y : boost::irange(0, tex_size)) {
		for(const int x : boost::irange(0, tex_size)) {
			dist_error.at<float>(y, x) =
				cv::norm(pos_map.at<cv::Vec3f>(y, x) - tex_xyz.at<cv::Vec3f>(y, x));
		}
	}
	if(bundle.isDebugEnabled()) {
		cv::imwrite(
			bundle.reservePath("debug_floor_dist_error.png"),
			dist_error * 100);
	}

	const float error_thresh = 0.3;

	// Guess true good region by grabcut.
	const cv::Vec3b color_non_floor(0, 0, 0);  // BG
	const cv::Vec3b color_bad_floor(0, 0, 255);  // prob BG
	const cv::Vec3b color_floor(0, 255, 0); // prob FG
	cv::Mat visualize(tex_size, tex_size, CV_8UC3);
	cv::Mat grabcut_mask(tex_size, tex_size, CV_8U);
	for(const int y : boost::irange(0, tex_size)) {
		for(const int x : boost::irange(0, tex_size)) {
			const bool pr_bad = dist_error.at<float>(y, x) > error_thresh;

			// For debug visualization & grabcut mask. Allow this in non-debug
			// to keep logic clean.
			auto& vis_px = visualize.at<cv::Vec3b>(y, x);
			auto& gb_px = grabcut_mask.at<uint8_t>(y, x);
			if(!floor_mask.at<uint8_t>(y, x)) {
				vis_px = color_non_floor;
				gb_px = cv::GC_BGD;
			} else if(pr_bad) {
				vis_px = color_bad_floor;
				gb_px = cv::GC_BGD;
			} else {
				vis_px = color_floor;
				gb_px = cv::GC_PR_FGD;
			}
		}
	}
	if(bundle.isDebugEnabled()) {
		cv::imwrite(
			bundle.reservePath("debug_floor_mask_pre.png"),
			visualize);
	}
	cv::Mat tmp_background;
	cv::Mat tmp_foregroud;
	cv::grabCut(tex_mesh.diffuse, grabcut_mask, cv::Rect(),
		tmp_background, tmp_foregroud, 5, cv::GC_INIT_WITH_MASK);
	if(bundle.isDebugEnabled()) {
		for(const int y : boost::irange(0, tex_size)) {
			for(const int x : boost::irange(0, tex_size)) {
				const auto gb_px = grabcut_mask.at<uint8_t>(y, x);
				auto& vis_px = visualize.at<cv::Vec3b>(y, x);
				if(gb_px == cv::GC_PR_BGD) {
					vis_px = color_bad_floor;
				} else if(gb_px == cv::GC_PR_FGD) {
					vis_px = color_floor;
				}
				// other part shouldn't change by grabcut
			}
		}
		cv::imwrite(
			bundle.reservePath("debug_floor_mask_post.png"),
			visualize);
	}

	// Inpaint bad region.
	cv::Mat inpaint_mask(tex_size, tex_size, CV_8U);
	for(const int y : boost::irange(0, tex_size)) {
		for(const int x : boost::irange(0, tex_size)) {
			const bool is_bad =
				grabcut_mask.at<uint8_t>(y, x) != cv::GC_PR_FGD;
			const bool is_floor = floor_mask.at<uint8_t>(y, x);
			inpaint_mask.at<uint8_t>(y, x) =
				(is_bad & is_floor) ? 255 : 0;
		}
	}
	const float inpaint_radius = 5;
	cv::Mat tex_rgb_filled;
	cv::inpaint(tex_mesh.diffuse, inpaint_mask, tex_rgb_filled,
		inpaint_radius, cv::INPAINT_TELEA);
	tex_mesh.diffuse = tex_rgb_filled;

	return std::make_pair(
		tex_mesh,
		recognizeLights(bundle, rframe, scans_aligned, cloud_inside));
}


std::pair<TexturedMesh, cv::Mat> bakeBoundaryTexture(
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

	const TriangleMesh<Eigen::Vector2f> shape = mapSecond(assignUV(shape_wo_uv));

	INFO("Baking texture to mesh with #tri=", (int)shape.triangles.size());
	const auto c_scans = scans.getScansWithPose();
	const auto& c_scan = c_scans[0];

	const Eigen::Vector3f invalid_value(1000, 1000, 1000);
	cv::Mat mapping(tex_size, tex_size, CV_32FC2);
	mapping = cv::Scalar(0, 0);

	const Eigen::Affine3f world_to_local = c_scan.local_to_world.inverse();
	const cv::Mat pos_map = getPositionMapInUV(shape, tex_size, invalid_value);
	for(const int y : boost::irange(0, tex_size)) {
		for(const int x : boost::irange(0, tex_size)) {
			const auto pos_raw = pos_map.at<cv::Vec3f>(y, x);
			const auto pos_world = Eigen::Vector3f(pos_raw[0], pos_raw[1], pos_raw[2]);
			if(pos_world == invalid_value) {
				continue;
			}
			const auto tpr = c_scan.raw_scan.toThetaPhiR(
				world_to_local * pos_world);
			const float er_x = c_scan.raw_scan.er_rgb.cols * std::get<1>(tpr) / (2 * pi);
			const float er_y = c_scan.raw_scan.er_rgb.rows * std::get<0>(tpr) / pi;
			mapping.at<cv::Vec2f>(y, x) = cv::Vec2f(er_x, er_y);
		}
	}
	cv::Mat diffuse;
	cv::remap(c_scan.raw_scan.er_rgb, diffuse, mapping, cv::Mat(),
		cv::INTER_LINEAR);
	cv::Mat xyz;
	cv::remap(c_scan.getXYZMap(), xyz, mapping, cv::Mat(), cv::INTER_LINEAR);

	// pack everything.
	TexturedMesh tm;
	tm.diffuse = diffuse;
	tm.mesh = shape;
	return std::make_pair(tm, xyz);
}

}  // namespace

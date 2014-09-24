#include <array>
#include <iostream>
#include <fstream>
#include <limits>
#include <map>
#include <random>

#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/point_generators_3.h>
#include <CGAL/algorithm.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/convex_hull_3.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <logging.h>
#include <server/recon_server.h>
#include <visual/cloud_base.h>
#include <visual/cloud_baker.h>
#include <visual/cloud_conversion.h>
#include <visual/mapping.h>
#include <visual/marching_cubes.h>
#include <visual/scene_recognizer.h>
#include <visual/shape_fitter.h>
#include <visual/texture_conversion.h>

/* Bunch of examples on how to use codes */

void testMeshIO() {
	INFO("creating metaball");
	const auto mesh_n = visual::extractIsosurface(
		3, [](Eigen::Vector3f p) {
			return (
				1 / (0.01 + p.norm()) +
				1 / (0.01 + (p - Eigen::Vector3f(0.9,0.9,0.9)).norm())
				);
		}, std::make_pair(
			Eigen::Vector3f(-3, -3, -3),
			Eigen::Vector3f(3, 3, 3)),
		0.1);
	std::ofstream test("test.ply");
	mesh_n.serializePLY(test);

	const auto mesh_n_uv = visual::assignUV(mesh_n);
	const auto mesh_uv = visual::mapSecond(mesh_n_uv);
	cv::imwrite("uv.png", visual::visualizeUVMap(mesh_uv));
	cv::imwrite("uv_3d.png", visual::bake3DTexture(mesh_uv,
		[](Eigen::Vector3f p) {
			return p;
		}));

	std::ofstream test_uv("test_uv.obj");
	std::ofstream test_mat("test_uv.mtl");
	mesh_uv.serializeObjWithUv(test_uv, "test_uv.mtl");
	visual::writeObjMaterial(test_mat, "uv_3d.png");
}

std::string guessSceneName(const std::string& scan_path) {
	std::vector<std::string> components;
	boost::split(components, scan_path, boost::is_any_of("/"));
	if(components.size() >= 1 && components.back() != "") {
		return components.back();
	} else if(components.size() >= 2 && components[components.size() - 2] != "") {
		return components[components.size() - 2];
	} else {
		return "result_bundle";
	}
}


void test_segmentation(
		std::string dir_path,
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_org) {
	using boost::filesystem::path;
	using K = CGAL::Exact_predicates_inexact_constructions_kernel;
	using Point_3 = K::Point_3;
	using Polyhedron_3 = CGAL::Polyhedron_3<K>;


	pcl::search::Search<pcl::PointXYZRGB>::Ptr tree =
		boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB>> (new pcl::search::KdTree<pcl::PointXYZRGB>);
	
	/// pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);

	// decompose to XYZRGB + Normal
	auto cloud = visual::cloud_base::cast<pcl::PointXYZRGBNormal, pcl::PointXYZRGB>(cloud_org);
	auto normals = visual::cloud_base::cast<pcl::PointXYZRGBNormal, pcl::Normal>(cloud_org);


	INFO("Doing EC");
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
	ec.setClusterTolerance (0.02); // 2cm
	ec.setMinClusterSize (100);
	ec.setMaxClusterSize (100000);  // 100000: most small objects / 500000: everything incl. tabgles
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud);

	std::vector <pcl::PointIndices> clusters;
	ec.extract (clusters);

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

		// dump
		const std::string name = "poly_" + std::to_string(i_cluster) + ".ply";

		std::ofstream debug_poly((dir_path / path(name)).string());
		mesh.serializePLY(debug_poly);

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

	// serialize
	visual::TriangleMesh<Eigen::Vector3f> mesh;
	for(const auto& pt : colored_cloud->points) {
		mesh.vertices.push_back(std::make_pair(
			pt.getVector3fMap(),
			Eigen::Vector3f(pt.r, pt.g, pt.b)));
	}

	std::ofstream debug_points_file((dir_path / path("debug_second_cluster.ply")).string());
	mesh.serializePLYWithRgb(debug_points_file);
}

void secondPass(std::string dir_path) {
	using boost::filesystem::path;

	INFO("Loading a scan from", dir_path);
	std::ifstream f_input((path(dir_path) / path("debug_shapes.json")).string());
	if(!f_input.is_open()) {
		throw std::runtime_error("Cloudn't open debug_shapes.json");
	}
	Json::Value cloud_json;
	Json::Reader().parse(f_input, cloud_json);

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

	test_segmentation(dir_path, cloud);
}


int main(int argc, char** argv) {
	using boost::program_options::notify;
	using boost::program_options::options_description;
	using boost::program_options::parse_command_line;
	using boost::program_options::store;
	using boost::program_options::value;
	using boost::program_options::variables_map;

	options_description desc("Reconstruct 3D scene from scans.");
	desc.add_options()
		("help", "show this message")
		("test", "do experimental stuff")
		("second", value<std::string>(), "do second pass")
		("convert", value<std::vector<std::string>>()->multitoken(), "convert given scans");

	variables_map vars;
	store(parse_command_line(argc, argv, desc), vars);
	notify(vars);

	if(vars.count("help") > 0) {
		std::cout << desc << std::endl;
		return 0;
	} else if(vars.count("test") > 0) {
		testMeshIO();
		return 0;
	} else if(vars.count("convert") > 0) {
		const auto dir_paths = vars["convert"].as<std::vector<std::string>>();
		if(dir_paths.empty()) {
			ERROR("Need one or more scans to proceed");
			return 1;
		}
		INFO("Loading scans, #scans=", (int)dir_paths.size());
		std::vector<visual::SingleScan> scans;
		// HACK ALERT!
		// TODO: remove this table
		// Make Y+ north.
		std::map<std::string, float> pre_rot = {
			{"scan-20140801-18:41-gakusei-large", 0.2 * 3.14},
			{"test-20140801-15:24-gakusei-table", 0.4 * 3.14},
			{"scan-20140801-18:44", -0.7 * 3.14},
			{"scan-20140801-18:47", 1.2 * 3.14},
			{"scan-20140801-18:50-ocha-2", -0.6 * 3.14},
			{"scan-20140801-18:54-gakusei-small-1", 0.4 * 3.14},
			{"scan-20140801-18:57-gakusei-small-2", -0.4 * 3.14},
			{"scan-20140801-19:00-gakusei-small-3", 0.2 * 3.14}
		};
		for(const auto& dir_path : dir_paths) {
			auto it = pre_rot.find(guessSceneName(dir_path));
			float pr = 0;
			if(it != pre_rot.end()) {
				WARN("Using hard-coded pre-rotation table", it->second);
				pr = it->second;
			}
			scans.emplace_back(dir_path, pr);
		}

		INFO("Converting to a scene");
		visual::SceneAssetBundle bundle(guessSceneName(dir_paths.front()));
		visual::scene_recognizer::recognizeScene(bundle, scans);
		return 0;
	} else if(vars.count("second") > 0) {
		const std::string dir_path = vars["second"].as<std::string>();
		secondPass(dir_path);
		return 0;
	} else {
		INFO("Launching HTTP server");
		server::ReconServer server;
		server.launch();
		while(true) {
			std::string dummy;
			std::cin >> dummy;
		}
		return 0;
	}
}

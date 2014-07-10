#include "cloud_baker.h"

#include <array>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <set>

#include <boost/filesystem.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/one_bit_color_map.hpp>
#include <boost/graph/stoer_wagner_min_cut.hpp>
#include <boost/graph/boykov_kolmogorov_max_flow.hpp>
#include <boost/range/irange.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <analyzer/voxel_traversal.h>
#include <logging.h>
#include <range2.h>
#include <visual/cloud_conversion.h>
#include <visual/mapping.h>
#include <visual/marching_cubes.h>
#include <visual/scene_converter.h>
#include <visual/texture_conversion.h>
#include <visual/voxel_conversion.h>

namespace visual {

using Tuple3i = std::tuple<int, int, int>;


VoxelDescription::VoxelDescription() : average_image_color(0, 0, 0), guess(false) {
}


Voxelizer::Voxelizer(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float voxel_size) :
	cloud(cloud), voxel_size(voxel_size) {
}

std::map<Tuple3i, VoxelDescription> Voxelizer::getVoxelsDetailed() const {
	std::map<Tuple3i, VoxelDescription> vx;
	return vx;
}

std::map<Tuple3i, VoxelDescription> Voxelizer::getVoxelsDetailedWithoutGuess() const {

	// known to be filled
	std::map<Tuple3i, bool> voxels;
	std::map<Tuple3i, Eigen::Vector3f> voxels_accum;
	std::map<Tuple3i, int> voxels_count;
	for(const auto& pt : cloud->points) {
		if(!std::isfinite(pt.x)) {
			continue;
		}

		auto ix = pt.getVector3fMap() / voxel_size;
		auto key = std::make_tuple(
			static_cast<int>(std::floor(ix.x())),
			static_cast<int>(std::floor(ix.y())),
			static_cast<int>(std::floor(ix.z())));

		voxels[key] = true;
		if(voxels_accum.find(key) == voxels_accum.end()) {
			voxels_accum[key] = Eigen::Vector3f(pt.r, pt.g, pt.b);
			voxels_count[key] = 1;
		} else {
			voxels_accum[key] += Eigen::Vector3f(pt.r, pt.g, pt.b);
			voxels_count[key] += 1;
		}
	}

	// TODO: remove and use frame.camera_pos (need to consolidate
	// frame.camera_pos semantics)
	const auto& camera_origin = Eigen::Vector3f::Zero();

	std::map<Tuple3i, bool> voxels_empty;
	for(const auto& pair_filled : voxels) {
		// cast ray from camera
		const auto pos = Eigen::Vector3f(
			std::get<0>(pair_filled.first) + 0.5,
			std::get<1>(pair_filled.first) + 0.5,
			std::get<2>(pair_filled.first) + 0.5) * voxel_size;

		const auto dir = (pos - camera_origin).normalized();

		// traverse until hit.
		VoxelTraversal traversal(voxel_size, camera_origin, dir);
		for(int i : boost::irange(0, 100)) {
			const auto key = traversal.next();

			// Hit wall.
			if(voxels.find(key) != voxels.end()) {
				break;
			}

			voxels_empty[key] = true;
		}
	}

	std::map<Tuple3i, VoxelDescription> voxel_merged;
	for(const auto& pair_filled : voxels) {
		VoxelDescription desc;
		desc.state = VoxelState::OCCUPIED;
		desc.average_image_color =
		voxels_accum[pair_filled.first] / voxels_count[pair_filled.first];
		voxel_merged[pair_filled.first] = desc;
	}
	for(const auto& pair_empty : voxels_empty) {
		VoxelDescription desc;
		desc.state = VoxelState::EMPTY;
		voxel_merged[pair_empty.first] = desc;
	}
	return voxel_merged;
}




TriangleMesh<std::nullptr_t> createBox(
		Eigen::Vector3f center,Eigen::Vector3f half_dx,
		Eigen::Vector3f half_dy, Eigen::Vector3f half_dz) {
	TriangleMesh<std::nullptr_t> box;
	for(int i : boost::irange(0, 8)) {
		const Eigen::Vector3f vertex_pos = center +
			((i & 0b001) ? 1 : -1) * half_dx +
			((i & 0b010) ? 1 : -1) * half_dy +
			((i & 0b100) ? 1 : -1) * half_dz;
		box.vertices.push_back(std::make_pair(vertex_pos, nullptr));
	}

	// Create inward-facing triangles. (CCW is positive direction)
	// Draw a cube with 000-111 to understand this.
	box.triangles = {
		// X-
		std::make_tuple(0, 4, 2),
		std::make_tuple(6, 2, 4),
		// X+
		std::make_tuple(5, 1, 7),
		std::make_tuple(3, 7, 1),
		// Y-
		std::make_tuple(0, 1, 4),
		std::make_tuple(5, 4, 1),
		// Y+
		std::make_tuple(6, 7, 2),
		std::make_tuple(3, 2, 7),
		// Z-
		std::make_tuple(0, 2, 1),
		std::make_tuple(3, 1, 2),
		// Z+
		std::make_tuple(6, 4, 7),
		std::make_tuple(5, 7, 4)
	};

	return box;
}

TriangleMesh<std::nullptr_t> createBox(
	Eigen::Vector3f center, float half_size) {
	return createBox(center,
		Eigen::Vector3f::UnitX() * half_size,
		Eigen::Vector3f::UnitY() * half_size,
		Eigen::Vector3f::UnitZ() * half_size);
}

TriangleMesh<std::nullptr_t> CloudBaker::extractSurface(
		const DenseVoxel<bool>& dv,
		const Eigen::Vector3i& imin,
		const float voxel_size) {

	TriangleMesh<std::nullptr_t> mesh;
	for(const auto& i : range3(dv.shape())) {
		const bool inside =
			(Eigen::Vector3i::Zero().array() < i.array()).all() &&
			(i.array() < (dv.shape() - Eigen::Vector3i(1, 1, 1)).array()).all();

		// Don't output non-surface-facing cubes.
		// TODO: ideally, we should operate at quads level, not cube level.
		bool can_skip = true;
		if(inside) {
			for(int axis : boost::irange(0, 3)) {
				Eigen::Vector3i delta = Eigen::Vector3i::Zero();
				delta(axis) += 1;

				if(dv[i + delta] != dv[i]) {
					can_skip = false;
					break;
				}
				if(dv[i - delta] != dv[i]) {
					can_skip = false;
					break;
				}
			}
		} else {
			can_skip = false;
		}

		if(!can_skip && dv[i]) {
			const Eigen::Vector3f pos = (imin + i).cast<float>() * voxel_size;
			mesh.merge(createBox(pos + Eigen::Vector3f::Ones() * (0.5 * voxel_size), voxel_size));
		}
	}
	return mesh;
}

CloudBaker::CloudBaker(const Json::Value& cloud_json) {
	loadFromJson(cloud_json);

	// Approximate exterior by OBB.
	const auto cloud_colorless = decolor(*cloud);
	TriangleMesh<std::nullptr_t> mesh = OBBFitter(cloud_colorless).extract();
	exterior_mesh = mapSecond(assignUV(mesh));

	// Voxelize
	// Goal:
	// Label all voxels as:
	// * EMPTY (air)
	// * EXTERIOR (static building part)
	// * OBJECTS

	const float voxel_size = 0.05;

	// Voxelize point cloud.
	Voxelizer voxelizer(cloud, voxel_size);
	const auto vxs = voxelizer.getVoxelsDetailedWithoutGuess();
	int num_occupied = 0;
	int num_empty = 0;
	for(const auto& vx : vxs) {
		if(vx.second.state == VoxelState::OCCUPIED) {
			num_occupied++;
		} else if(vx.second.state == VoxelState::EMPTY) {
			num_empty++;
		}
	}
	INFO("Voxel Stat occupied", num_occupied, "empty", num_empty);

	// Calculate min & max point.
	const Eigen::Vector3i margin(20, 20, 20);
	Eigen::Vector3i imin(1000, 1000, 1000), imax(-1000, -1000, -1000);
	for(const auto& vx : vxs) {
		const Eigen::Vector3i i(
			std::get<0>(vx.first), std::get<1>(vx.first), std::get<2>(vx.first));
		imin = imin.cwiseMin(i);
		imax = imax.cwiseMax(i);
	}
	imin -= margin;
	imax += margin;
	INFO("Voxel range:", showVec3i(imin), showVec3i(imax));

	// Densify voxel.
	DenseVoxel<RoomVoxel> dv(imax - imin + Eigen::Vector3i(1, 1, 1));
	for(const auto& i : range3(dv.shape())) {
		dv[i] = RoomVoxel::UNKNOWN;
	}
	for(const auto& vx : vxs) {
		const auto i = margin + Eigen::Vector3i(
			std::get<0>(vx.first), std::get<1>(vx.first), std::get<2>(vx.first));

		RoomVoxel label;
		if(vx.second.state == VoxelState::OCCUPIED) {
			label = RoomVoxel::INTERIOR;
		} else if(vx.second.state == VoxelState::EMPTY) {
			label = RoomVoxel::EMPTY;
		}
		dv[i - imin] = label;
	}

	// Fill exterior.
	const auto exterior_voxel = meshToVoxel(
		exterior_mesh, imin.cast<float>() * voxel_size, voxel_size, dv.shape());
	assert(dv.shape() == exterior_voxel.shape());
	for(const auto& i : range3(dv.shape())) {
		if(exterior_voxel[i]) {
			//dv[i] = RoomVoxel::EXTERIOR;
		}
	}

	logVoxelCounts(dv);
	const auto dv_before = dv.clone();
	estimateUnknownCells(dv);
	logVoxelCounts(dv);

	// Dump debug mesh.
	for(const auto state : RoomVoxelString) {
		DenseVoxel<bool> target(dv.shape());
		for(const auto& i : range3(dv.shape())) {
			target[i] = (dv[i] == state.first);
		}
		std::ofstream f_debug("debug_voxel_" + state.second + ".ply");
		extractSurface(target, imin, voxel_size).serializePLY(f_debug);
	}

	// Output delta.
	DenseVoxel<bool> dv_delta(dv.shape());
	for(const auto& i : range3(dv.shape())) {
		dv_delta[i] = (dv[i] == RoomVoxel::INTERIOR && dv_before[i] == RoomVoxel::UNKNOWN);
	}
	std::ofstream f_debug("debug_voxel_delta.ply");
	extractSurface(dv_delta, imin, voxel_size).serializePLY(f_debug);
}

void CloudBaker::logVoxelCounts(const DenseVoxel<RoomVoxel>& dv) {
	std::map<RoomVoxel, int> count;
	for(const auto& state : RoomVoxelString) {
		count[state.first] = 0;
	}
	for(const auto& i : range3(dv.shape())) {
		count[dv[i]] += 1;
	}

	Json::Value stat;
	for(const auto& type_count : count) {
		stat[RoomVoxelString.find(type_count.first)->second] = type_count.second;
	}
	INFO("Voxel composition", stat);
}

void CloudBaker::estimateUnknownCells(DenseVoxel<RoomVoxel>& dv) {
	// Use very common technique.
	// * Create 2 virtual vertex corresponding to 2 labels.
	// * Connect known label vertices with one of the two label vertices, with infinite capacity.
	// Mincut represents two labled regions.
	using Traits = boost::adjacency_list_traits<boost::vecS, boost::vecS, boost::directedS>;
	using Graph = boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS,
		boost::property<boost::vertex_name_t, std::string,
		boost::property<boost::vertex_index_t, long,
		boost::property<boost::vertex_color_t, boost::default_color_type,
		boost::property<boost::vertex_distance_t, long,
		boost::property<boost::vertex_predecessor_t, Traits::edge_descriptor>>>>>,

		boost::property<boost::edge_capacity_t, long,
		boost::property<boost::edge_residual_capacity_t, long,
		boost::property<boost::edge_reverse_t, Traits::edge_descriptor>>>>;

	// Create vertex index - position mapping.
	const int num_real_vertices = dv.shape().prod();
	const int num_vertices = num_real_vertices + 2;

	const int vertex_source = num_real_vertices;  // EMPTY
	const int vertex_sink = num_real_vertices + 1;  // other labels

	const Eigen::Vector3i strides(1, dv.shape()(0), dv.shape()(0) * dv.shape()(1));
	auto pos_to_vertex = [&](const Eigen::Vector3i& pos) -> int {
		return pos.dot(strides);
	};
	auto vertex_to_pos = [&](int v) -> Eigen::Vector3i {
		return Eigen::Vector3i(
			(v / strides(0)) % strides(1),
			(v / dv.shape()(0)) % dv.shape()(1),
			v / (dv.shape()(0) * dv.shape()(1)));
	};

	// Setup convenience functions.
	Graph g(num_vertices);
	auto reverse_edge = get(boost::edge_reverse, g);
	auto residual_capacity = get(boost::edge_residual_capacity, g);
	auto capacity = get(boost::edge_capacity, g);

	auto create_edge = [&](int u, int v, int weight_uv, int weight_vu) {
		auto e_uv = boost::add_edge(u, v, g).first;
		auto e_vu = boost::add_edge(v, u, g).first;

		reverse_edge[e_uv] = e_vu;
		reverse_edge[e_vu] = e_uv;

		capacity[e_uv] = weight_uv;
		residual_capacity[e_uv] = weight_uv;

		capacity[e_vu] = weight_vu;
		residual_capacity[e_vu] = weight_vu;
	};

	// Add edges to represent known labels.
	for(const auto& pos : range3(dv.shape())) {
		if(dv[pos] == RoomVoxel::UNKNOWN) {
			continue;
		}

		if(dv[pos] == RoomVoxel::EMPTY) {
			create_edge(vertex_source, pos_to_vertex(pos), 2, 0);
		} else {
			create_edge(pos_to_vertex(pos), vertex_sink, 10000, 0);
		}
	}

	// Add edges to represent voxel-voxel cost term.
	for(const auto& pos : range3(dv.shape())) {
		// Create edges toward X+, Y+, Z+ directions.
		for(int axis : boost::irange(0, 3)) {
			Eigen::Vector3i pos_to = pos;
			pos_to(axis) += 1;
			// Ignore if destination vertex is out of bounds.
			if(!(pos_to.array() < dv.shape().array()).all()) {
				continue;
			}

			const int transition_cost = (axis == 1) ? 1 : 20;
			create_edge(
				pos_to_vertex(pos), pos_to_vertex(pos_to),
				transition_cost, transition_cost);
		}
	}

	const int maxflow = boost::boykov_kolmogorov_max_flow(g, vertex_source, vertex_sink);
	INFO("Voxel labeling cost", maxflow);

	// Collect vertices that fell on source-side by depth first search within
	// mincut boundary.
	std::set<int> source_side;
	std::vector<int> frontier = {vertex_source};
	while(!frontier.empty()) {
		const int v = frontier.back();
		frontier.pop_back();
		if(source_side.find(v) != source_side.end()) {
			continue;
		}
		source_side.insert(v);

		auto edges_its = boost::out_edges(v, g);
		for(auto it_edge = edges_its.first; it_edge != edges_its.second; it_edge++) {
			// maxed out -> cannot cross this edge (because its one of mincut edges)
			if(residual_capacity[*it_edge] != 0) {
				frontier.push_back(boost::target(*it_edge, g));
			}
		}
	}
	source_side.erase(vertex_source);
	DEBUG("source-side voxels", (int)source_side.size());

	// Collect exterior cells by flooding from a voxel in a corner.
	/*
	std::set<int> ext_side;
	{
		std::vector<int> frontier = {pos_to_vertex(Eigen::Vector3i::Zero())};
		while(!frontier.empty()) {
			const int v = frontier.back();
			frontier.pop_back();
			if(ext_side.find(v) != ext_side.end()) {
				continue;
			}
			ext_side.insert(v);

			auto edges_its = boost::out_edges(v, g);
			for(auto it_edge = edges_its.first; it_edge != edges_its.second; it_edge++) {
				// Don't traverse if target is not UNKNOWN.
				if(dv[vertex_to_pos(boost::target(*it_edge, g))] == RoomVoxel::UNKNOWN) {
					frontier.push_back(boost::target(*it_edge, g));
				}
			}
		}
	}
	*/

	// Re-label based on vertices.
	for(const auto& pos : range3(dv.shape())) {
		if(dv[pos] != RoomVoxel::UNKNOWN) {
			continue;
		}
		/*
		if(ext_side.find(pos_to_vertex(pos)) != ext_side.end()) {
			dv[pos] = RoomVoxel::EXTERIOR;
			continue;
		}
		*/

		if(source_side.find(pos_to_vertex(pos)) != source_side.end()) {
			dv[pos] = RoomVoxel::INTERIOR;
		} else {
			dv[pos] = RoomVoxel::EMPTY;
		}
	}

}

Json::Value CloudBaker::showVec3i(const Eigen::Vector3i& v) {
	Json::Value m;
	m["x"] = v.x();
	m["y"] = v.y();
	m["z"] = v.z();
	return m;
}

void CloudBaker::loadFromJson(const Json::Value& cloud_json) {
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

TexturedMesh CloudBaker::generateRoomMesh() {
	const auto& mesh_uv = exterior_mesh;

	// Project points to the surface and draw circles onto texture.
	const int tex_size = 2048;
	const float thresh_distance = 0.5;
	cv::Mat diffuse(tex_size, tex_size, CV_8UC3);
	diffuse = cv::Scalar(0, 0, 0);
	for(const auto& point : cloud->points) {
		const Eigen::Vector3f pos = point.getVector3fMap();
		const auto dist_and_uv = nearestCoordinate(mesh_uv, pos);
		// Ignore points too far from exterior.
		if(dist_and_uv.first < thresh_distance) {
			const cv::Scalar color(point.b, point.g, point.r);
			cv::circle(
				diffuse, eigenToCV(swapY(dist_and_uv.second) * tex_size), 1,
				color, -1);
		}
	}
	fillHoles(diffuse, cv::Vec3b(0, 0, 0), 5);

	TexturedMesh tm;
	tm.diffuse = diffuse;
	tm.mesh = mesh_uv;
	return tm;
}

void CloudBaker::fillHoles(cv::Mat& image, const cv::Vec3b undefined, int iteration) {
	const Eigen::Vector2i imageMin(0, 0);
	const Eigen::Vector2i imageMax(image.cols, image.rows);
	for(int step : boost::irange(0, iteration)) {
		bool propagation_happened = false;
		for(auto pos : range2(imageMin, imageMax)) {
			auto& current_pixel = image.at<cv::Vec3b>(pos.y(), pos.x());
			if(current_pixel != undefined) {
				continue;
			}
			// Copy defined neighbor color.
			// (Search range contains itself, but it's ok because it's undefined thus ignored)
			for(auto pos_search : range2(
				(pos - Eigen::Vector2i(1, 1)).cwiseMax(imageMin),
				(pos + Eigen::Vector2i(2, 2)).cwiseMin(imageMax))) {
				const auto candidate = image.at<cv::Vec3b>(pos_search.y(), pos_search.x());
				if(candidate != undefined) {
					current_pixel = candidate;
					propagation_happened = true;
					break;
				}
			}
		}
		// all pixels are filled, or image == undefined
		if(!propagation_happened) {
			break;
		}
	}
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CloudBaker::decolor(const pcl::PointCloud<pcl::PointXYZRGB>& cloud) {
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

#include "triangulation.h"

#include <list>

#include <boost/range/irange.hpp>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_face_base_with_info_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Polygon_2.h>

namespace recon {

struct FaceInfo2 {
	FaceInfo2() {}
	int nesting_level;
	bool in_domain(){
		return nesting_level%2 == 1;
	}
};

using K = CGAL::Exact_predicates_inexact_constructions_kernel;
using Vb = CGAL::Triangulation_vertex_base_with_info_2<int, K>;
using Fbb = CGAL::Triangulation_face_base_with_info_2<FaceInfo2,K>;
using Fb = CGAL::Constrained_triangulation_face_base_2<K,Fbb>;
using TDS = CGAL::Triangulation_data_structure_2<Vb,Fb>;
using Itag = CGAL::Exact_predicates_tag;
using CDT = CGAL::Constrained_Delaunay_triangulation_2<K, TDS, Itag>;
using Point = CDT::Point;
using Polygon_2 = CGAL::Polygon_2<K>;

void  mark_domains(CDT& ct,
		CDT::Face_handle start,
		int index,
		std::list<CDT::Edge>& border) {
	if(start->info().nesting_level != -1){
		return;
	}
	std::list<CDT::Face_handle> queue;
	queue.push_back(start);
	while(!queue.empty()) {
		CDT::Face_handle fh = queue.front();
		queue.pop_front();
		if(fh->info().nesting_level != -1) {
			continue;
		}
		fh->info().nesting_level = index;
		for(int i = 0; i < 3; i++) {
			CDT::Edge e(fh,i);
			CDT::Face_handle n = fh->neighbor(i);
			if(n->info().nesting_level == -1){
					if(ct.is_constrained(e)) {
						border.push_back(e);
					}
					else {
						queue.push_back(n);
					}
			}
		}
	}
}

//explore set of facets connected with non constrained edges,
//and attribute to each such set a nesting level.
//We start from facets incident to the infinite vertex, with a nesting
//level of 0. Then we recursively consider the non-explored facets incident 
//to constrained edges bounding the former set and increase the nesting level by 1.
//Facets in the domain are those with an odd nesting level.
void mark_domains(CDT& cdt) {
	for(CDT::All_faces_iterator it = cdt.all_faces_begin(); it != cdt.all_faces_end(); ++it) {
		it->info().nesting_level = -1;
	}
	std::list<CDT::Edge> border;
	mark_domains(cdt, cdt.infinite_face(), 0, border);
	while(! border.empty()){
		CDT::Edge e = border.front();
		border.pop_front();
		CDT::Face_handle n = e.first->neighbor(e.second);
		if(n->info().nesting_level == -1){
			mark_domains(cdt, n, e.first->info().nesting_level+1, border);
		}
	}
}

void insert_polygon(CDT& cdt, const std::vector<Point>& polygon) {
	const int n = polygon.size();
	assert(n >= 3);
	// Insert all vertices with their indicies as info().
	std::vector<CDT::Vertex_handle> vertex_handles;
	for(const int ix : boost::irange(0, n)) {
		CDT::Vertex_handle v_handle = cdt.insert(polygon[ix]);
		v_handle->info() = ix;
		vertex_handles.push_back(v_handle);
	}
	// Now insert edges as constraints.
	for(const int ix : boost::irange(0, n)) {
		cdt.insert_constraint(
			vertex_handles[ix], vertex_handles[(ix + 1) % n]);
	}
}

// Use CGAL to triangulate a polygon.
// cf. https://stackoverflow.com/questions/17680321/retrive-vertices-from-cgals-delaunay-constrained-triangulation
std::vector<std::array<int, 3>> triangulatePolygon(
		const std::vector<Eigen::Vector2f>& points_eigen) {
	assert(points_eigen.size() >= 3);

	// Setup triangulation.
	std::vector<Point> poly;
	for(const auto& pt : points_eigen) {
		poly.emplace_back(pt(0), pt(1));
	}
	CDT cdt;
	insert_polygon(cdt, poly);
	mark_domains(cdt);

	// Retrieve data.
	std::vector<std::array<int, 3>> tris;
	for(auto fit = cdt.finite_faces_begin(); fit != cdt.finite_faces_end(); fit++) {
		if(!fit->info().in_domain()) {
			continue;
		}
		tris.push_back({{
			static_cast<int>(fit->vertex(0)->info()),
			static_cast<int>(fit->vertex(1)->info()),
			static_cast<int>(fit->vertex(2)->info())
			}});
	}
	assert(tris.size() == points_eigen.size() - 2);
	return tris;
}

}  // namespace

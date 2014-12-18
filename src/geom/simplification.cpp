#include "simplification.h"

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>

// Adaptor for Polyhedron_3
#include <CGAL/Surface_mesh_simplification/HalfedgeGraph_Polyhedron_3.h>

// Simplification function
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>

// Stop-condition policy
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_stop_predicate.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_ratio_stop_predicate.h>

#include <logging.h>

typedef CGAL::Simple_cartesian<double> Kernel;
typedef CGAL::Polyhedron_3<Kernel> Surface; 
typedef Surface::HalfedgeDS HalfedgeDS;
typedef Kernel::Point_3 Point;

namespace SMS = CGAL::Surface_mesh_simplification ;

//using namespace sgf;

namespace recon {


template < class HDS>
class geometry_to_surface_op :  public CGAL::Modifier_base<HDS> {
protected:
	const TriangleMesh<std::nullptr_t>& mesh;

public:
	typedef HDS Halfedge_data_structure;
public:
	geometry_to_surface_op( const TriangleMesh<std::nullptr_t>& mesh) : mesh(mesh) {
	}

	void operator()(HDS& hds) {
		CGAL::Polyhedron_incremental_builder_3<HalfedgeDS> builder( hds, true );
		builder.begin_surface(mesh.vertices.size(), mesh.triangles.size());

		for(const auto& vert : mesh.vertices) {
			builder.add_vertex(Point(
				vert.first.x(), vert.first.y(), vert.first.z()));
		}
		for(const auto& tri : mesh.triangles) {
			builder.begin_facet();
			for(const int vix : tri) {
				builder.add_vertex_to_facet(vix);
			}
			builder.end_facet();
		}
		if ( builder.check_unconnected_vertices() ) {
			builder.remove_unconnected_vertices();
		}
		builder.end_surface();
	}
};


void meshToSurface(const TriangleMesh<std::nullptr_t>& mesh, Surface &s ) {
	geometry_to_surface_op<Surface::HalfedgeDS> gen(mesh);
	s.delegate(gen);
}

TriangleMesh<std::nullptr_t> surfaceToMesh(const Surface& s) {
	TriangleMesh<std::nullptr_t> result;

	for ( Surface::Facet_const_iterator fit = s.facets_begin(); fit != s.facets_end(); ++fit) {
		if (!fit->is_triangle() ) {
			WARN("Skipping non-trianglular facet");
			continue;
		}
		const int vix_offset = result.vertices.size();
		int tick = 0;

		Surface::Halfedge_around_facet_const_circulator hit( fit->facet_begin() ), hend( hit );
		do {
			const Point p = hit->vertex()->point();
			if ( tick < 3 ) {
				result.vertices.emplace_back(
					Eigen::Vector3f(p.x(), p.y(), p.z()),
					nullptr);
				tick++;
			} else {
				WARN("We've got facets with more than 3 vertices even though the facet reported to be trianglular...");
				assert(false);
			}
		} while( ++hit != hend );

		result.triangles.push_back({{
			vix_offset + 0,
			vix_offset + 1,
			vix_offset + 2
		}});
	}

	return result;
}



TriangleMesh<std::nullptr_t> simplifyMesh(
		const TriangleMesh<std::nullptr_t>& mesh,
		float compression_ratio) {
	assert(0 < compression_ratio && compression_ratio <= 1);

	Surface surface;
	meshToSurface(mesh, surface);
	SMS::Count_ratio_stop_predicate<Surface> stop(compression_ratio);

	SMS::edge_collapse(surface,
		stop,
		CGAL::vertex_index_map( boost::get(CGAL::vertex_external_index,surface))
			.edge_index_map( boost::get(CGAL::edge_external_index,surface  )));

	return surfaceToMesh(surface);
}


}  // namespace

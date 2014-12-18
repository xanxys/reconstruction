#include "simplification.h"
#if 0

// Simplification function
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>

// Stop-condition policy
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_stop_predicate.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_ratio_stop_predicate.h>

#include <logging.h>

typedef CGAL::Simple_cartesian<float> Kernel;
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

#endif
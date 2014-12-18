// quick hack to integrate CGAL mesh simplification
// read http://www.idevgames.com/forums/thread-2169-page-2.html
#pragma once

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>

#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>


#include <logging.h>
#include <visual/triangle_mesh.h>

namespace recon {

template<typename Kernel>
TriangleMesh<std::nullptr_t> surfaceToMesh(
		const CGAL::Polyhedron_3<Kernel>& s) {
	using Surface = CGAL::Polyhedron_3<Kernel>;
	TriangleMesh<std::nullptr_t> result;

	for (typename Surface::Facet_const_iterator fit = s.facets_begin(); fit != s.facets_end(); ++fit) {
		if (!fit->is_triangle() ) {
			WARN("Skipping non-trianglular facet");
			continue;
		}
		const int vix_offset = result.vertices.size();
		int tick = 0;

		typename Surface::Halfedge_around_facet_const_circulator hit( fit->facet_begin() ), hend( hit );
		do {
			const typename Kernel::Point_3 p = hit->vertex()->point();
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

/*
TriangleMesh<std::nullptr_t> simplifyMesh(
    const TriangleMesh<std::nullptr_t>& mesh, float compression_ratio);
*/

}  // namespace


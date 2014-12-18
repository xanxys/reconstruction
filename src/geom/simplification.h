// quick hack to integrate CGAL mesh simplification
// read http://www.idevgames.com/forums/thread-2169-page-2.html
#pragma once

#include <visual/triangle_mesh.h>

namespace recon {

TriangleMesh<std::nullptr_t> simplifyMesh(
    const TriangleMesh<std::nullptr_t>& mesh, float compression_ratio);

}  // namespace


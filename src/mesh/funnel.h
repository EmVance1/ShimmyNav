#pragma once
#include "mesh.h"


namespace nav {

struct CrossInfo {
    size_t next_index;
    size_t neighbor_index;
};


Path edge_to_edge(const Mesh& mesh, std::vector<CrossInfo>&& path, Vector2f begin, Vector2f end);
Path funnel(const Mesh& mesh, std::vector<CrossInfo>&& path, Vector2f begin, Vector2f end);
// IndexedPath funnel_indexed(const Mesh& mesh, std::vector<CrossInfo>&& path, IndexedPoint begin, IndexedPoint end);

}

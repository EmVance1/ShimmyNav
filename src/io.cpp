#include "lib.h"
#include <fstream>


namespace nav {

void NavMesh::write_file(const std::string& filename, float scale) const {
    auto f = std::ofstream(filename, std::ios::binary);
    const auto tri_count = triangles.size();
    f.write((char*)&tri_count, sizeof(usize));
    for (const auto& tri : triangles) {
        f.write((char*)&tri, sizeof(Triangle));
    }
    const auto vert_count = vertices.size();
    f.write((char*)&vert_count, sizeof(usize));
    for (const auto& v : vertices) {
        const auto v2 = v / scale;
        f.write((char*)&v2, sizeof(Vector2f));
    }
    const auto edge_count = edges.size();
    f.write((char*)&edge_count, sizeof(usize));
    for (const auto& e : edges) {
        const Edge neg = { SIZE_MAX, Vector2f{}, 0, 0 };
        if (e.size() > 0) { f.write((char*)&e[0], sizeof(Edge)); } else { f.write((char*)&neg, sizeof(Edge)); }
        if (e.size() > 1) { f.write((char*)&e[1], sizeof(Edge)); } else { f.write((char*)&neg, sizeof(Edge)); }
        if (e.size() > 2) { f.write((char*)&e[2], sizeof(Edge)); } else { f.write((char*)&neg, sizeof(Edge)); }
    }
}


NavMesh NavMesh::read_file(const std::string& filename, float scale) {
    auto f = std::ifstream(filename, std::ios::binary);
    auto result = NavMesh();
    usize tri_count = 0;
    f.read((char*)&tri_count, sizeof(usize));
    result.triangles.reserve(tri_count);
    for (usize i = 0; i < tri_count; i++) {
        auto tri = Triangle();
        f.read((char*)&tri, sizeof(Triangle));
        result.triangles.push_back(tri);
    }
    usize vert_count = 0;
    f.read((char*)&vert_count, sizeof(usize));
    result.vertices.reserve(vert_count);
    for (usize i = 0; i < vert_count; i++) {
        Vector2f v;
        f.read((char*)&v, sizeof(Vector2f));
        v = v * scale;
        result.vertices.push_back(v);
    }
    usize edge_count = 0;
    f.read((char*)&edge_count, sizeof(usize));
    result.edges.reserve(edge_count);
    for (usize i = 0; i < edge_count; i++) {
        auto& edge = result.edges.emplace_back();
        Edge e;
        f.read((char*)&e, sizeof(Edge));
        if (e.index != SIZE_MAX) { edge.push_back(e); }
        f.read((char*)&e, sizeof(Edge));
        if (e.index != SIZE_MAX) { edge.push_back(e); }
        f.read((char*)&e, sizeof(Edge));
        if (e.index != SIZE_MAX) { edge.push_back(e); }
    }
    return result;
}

}


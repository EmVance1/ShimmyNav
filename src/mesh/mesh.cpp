#include "mesh.h"
#include <fstream>


namespace nav {

#ifdef _WIN32
#define PATH_NORM(p) p
#else
#define PATH_NORM(p) p.u8string()
#endif


std::optional<size_t> Mesh::get_triangle(Vector2f p, float error) const {
    for (usize i = 0; i < triangles.size(); i++) {
        if (triangles[i].contains(vertices.data(), p)) {
            return i;
        }
    }
    if (error < 0.0001f) { return {}; }
    for (usize i = 0; i < triangles.size(); i++) {
        if (triangles[i].contains_with_error(vertices.data(), p, error)) {
            return i;
        }
    }
    return {};
}


void Mesh::write_file(const std::filesystem::path& filename, float scale) const {
    auto f = std::ofstream(PATH_NORM(filename), std::ios::binary);
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
        for (size_t i = 0; i < 3; i++) {
            if (e.size() > i) {
                const auto _e = Mesh::Edge{ e[i].index, e[i].center / scale, e[i].a, e[i].b };
                f.write((char*)&_e, sizeof(Edge));
            } else {
                f.write((char*)&neg, sizeof(Edge));
            }
        }
    }
}


Mesh Mesh::read_file(const std::filesystem::path& filename, float scale) {
    auto f = std::ifstream(PATH_NORM(filename), std::ios::binary);
    auto result = Mesh();
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
        v *= scale;
        result.vertices.push_back(v);
    }
    usize edge_count = 0;
    f.read((char*)&edge_count, sizeof(usize));
    result.edges.reserve(edge_count);
    for (usize i = 0; i < edge_count; i++) {
        auto& edge = result.edges.emplace_back();
        Edge e;
        for (usize j = 0; j < 3; j++) {
            f.read((char*)&e, sizeof(Edge));
            e.center *= scale;
            if (e.index != SIZE_MAX) { edge.push_back(e); }
        }
    }
    return result;
}

}

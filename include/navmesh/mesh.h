#pragma once
#include "shapes.h"
#include <optional>
#include <filesystem>


namespace nav {

struct Mesh {
    struct Edge {
        usize index;
        Vector2f center;
        usize a;
        usize b;
    };
    std::vector<Vector2f> vertices;
    std::vector<Triangle> triangles;
    std::vector<std::vector<Edge>> edges;

    void write_file(const std::filesystem::path& filename, f32 scale = 1.f) const;
    static Mesh read_file(const std::filesystem::path& filename, f32 scale = 1.f);

    std::optional<size_t> get_triangle(Vector2f p, f32 error = 0.f) const;

    Path pathfind(Vector2f begin, Vector2f end) const;
    IndexedPath pathfind_indexed(Vector2f begin, Vector2f end) const;
};

}

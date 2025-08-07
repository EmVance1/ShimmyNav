#include "lib.h"
#include <CDT/CDT.h>
#include "gen_internal.h"
#include "simplify.h"

#include "bench.h"


namespace nav {

struct Edge {
    usize a;
    usize b;
};


static bool collinear(Vector2i a, Vector2i b, Vector2i c) {
    return (b.y - a.y) * (c.x - b.x) == (c.y - b.y) * (b.x - a.x);
}

// static bool collinear(Vector2f a, Vector2f b, Vector2f c) {
//     return ((b.y - a.y) * (c.x - b.x) - (c.y - b.y) * (b.x - a.x)) < 0.001f;
// }


static std::pair<size_t, size_t> shared_edge(const CDT::Triangle& a, const CDT::Triangle& b) {
    isize v1 = -1;
    isize v2 = -1;
    for (size_t i = 0; i < 3; i++) {
        for (size_t j = 0; j < 3; j++) {
            if (a.vertices[i] == b.vertices[j]) {
                if (v1 == -1) {
                    v1 = i;
                } else {
                    v2 = i;
                }
            }
        }
    }
    if ((v2 + 1) % 3 == v1) {
        std::swap(v2, v1);
    }
    if (v1 == -1 || v2 == -1) { throw "no shared edge"; }
    return { a.vertices[(size_t)v1], a.vertices[(size_t)v2] };
}


Mesh generate_delauney(
        const uint8_t* grid,
        size_t width,
        size_t height,
        size_t stride,
        Predicate P,
        Method method,
        float epsilon)
{
    // BENCH_BEGIN;

    auto polys = method == Method::MARCHING_SQUARES ?
        marching_squares(grid, width, height, stride, P) :
        floodfill(grid, width, height, stride, P);

    // BENCH_STEP("polygon construction");

    auto verts = std::vector<Vector2i>();
    auto edges = std::vector<Edge>();

    size_t offset = 0;
    for (auto& p : polys) {
        auto dp = douglas_peucker(p, epsilon);
        if (dp.front() == dp.back()) { dp.pop_back(); }
        if (collinear(dp[dp.size()-1], dp[0], dp[1])) { dp.erase(dp.begin()); }
        else if (collinear(dp[dp.size()-2], dp[dp.size()-1], dp[0])) { dp.pop_back(); }

        size_t i = 0;
        for (const auto& v : dp) {
            verts.push_back(v);
            edges.push_back(Edge{ offset + i, offset + ((i + 1) % (dp.size())) });
            i++;
        }

        offset += dp.size();
    }

    // BENCH_STEP("simplify & prepare");

    auto cdt = CDT::Triangulation<float>();
    cdt.insertVertices(
        verts.begin(),
        verts.end(),
        [](const Vector2i& p){ return (f32)p.x; },
        [](const Vector2i& p){ return (f32)p.y; }
    );
    cdt.insertEdges(
        edges.begin(),
        edges.end(),
        [](const Edge& e){ return e.a; },
        [](const Edge& e){ return e.b; }
    );
    cdt.eraseOuterTrianglesAndHoles();

    // BENCH_STEP("triangulation");

    auto mesh = Mesh();
    for (const auto& vert : cdt.vertices) {
        mesh.vertices.push_back(Vector2f{ vert.x * 0.1f, vert.y * 0.1f });
    }
    for (const auto& tri : cdt.triangles) {
        mesh.triangles.push_back(Triangle{
            tri.vertices[0],
            tri.vertices[1],
            tri.vertices[2],
        });
        auto ns = std::vector<Mesh::Edge>();
        if (tri.neighbors[0] != CDT::noNeighbor) {
            const auto [_v1, _v2] = shared_edge(tri, cdt.triangles[tri.neighbors[0]]);
            const auto v1 = Vector2f{ cdt.vertices[_v1].x, cdt.vertices[_v1].y } * 0.1f;
            const auto v2 = Vector2f{ cdt.vertices[_v2].x, cdt.vertices[_v2].y } * 0.1f;
            ns.push_back(Mesh::Edge{
                tri.neighbors[0],
                v1 + (v2 - v1) / 2,
                _v1, _v2,
            });
        }
        if (tri.neighbors[1] != CDT::noNeighbor) {
            const auto [_v1, _v2] = shared_edge(tri, cdt.triangles[tri.neighbors[1]]);
            const auto v1 = Vector2f{ cdt.vertices[_v1].x, cdt.vertices[_v1].y } * 0.1f;
            const auto v2 = Vector2f{ cdt.vertices[_v2].x, cdt.vertices[_v2].y } * 0.1f;
            ns.push_back(Mesh::Edge{
                tri.neighbors[1],
                v1 + (v2 - v1) / 2,
                _v1, _v2,
            });
        }
        if (tri.neighbors[2] != CDT::noNeighbor) {
            const auto [_v1, _v2] = shared_edge(tri, cdt.triangles[tri.neighbors[2]]);
            const auto v1 = Vector2f{ cdt.vertices[_v1].x, cdt.vertices[_v1].y } * 0.1f;
            const auto v2 = Vector2f{ cdt.vertices[_v2].x, cdt.vertices[_v2].y } * 0.1f;
            ns.push_back(Mesh::Edge{
                tri.neighbors[2],
                v1 + (v2 - v1) / 2,
                _v1, _v2,
            });
        }

        mesh.edges.push_back(ns);
    }

    // BENCH_STEP("data extraction");

    return mesh;
}

Mesh generate_delauney(
        const uint8_t* grid,
        size_t width,
        size_t height,
        size_t stride,
        size_t index,
        Method method,
        float epsilon)
{
    BENCH_BEGIN;

    const auto len = width * height;

    auto polys = method == Method::MARCHING_SQUARES ?
        marching_squares(grid, width, height, stride, index) :
            len > 400 * 400 ?
                floodfill_threaded(grid, width, height, stride, index) :
                floodfill(grid, width, height, stride, index);

    BENCH_STEP("polygon construction");

    auto verts = std::vector<CDT::V2d<f32>>();
    auto edges = std::vector<CDT::Edge>();

    u32 offset = 0;
    for (auto& p : polys) {
        auto dp = douglas_peucker(p, epsilon);
        if (dp.front() == dp.back()) { dp.pop_back(); }
        if (collinear(dp[dp.size()-1], dp[0], dp[1])) { dp.erase(dp.begin()); }
        else if (collinear(dp[dp.size()-2], dp[dp.size()-1], dp[0])) { dp.pop_back(); }

        verts.reserve(verts.size() + dp.size());
        edges.reserve(edges.size() + dp.size());
        u32 i = 0;
        for (const auto& v : dp) {
            verts.push_back(CDT::V2d<f32>((f32)v.x, (f32)v.y));
            edges.push_back(CDT::Edge(offset + i, offset + ((i + 1) % (dp.size()))));
            i++;
        }

        offset += dp.size();
    }

    BENCH_STEP("simplify & prepare");

    auto cdt = CDT::Triangulation<float>();
    cdt.insertVertices(verts);
    cdt.insertEdges(edges);
    cdt.eraseOuterTrianglesAndHoles();

    BENCH_STEP("triangulation");

    auto mesh = Mesh();
    mesh.vertices.reserve(cdt.vertices.size());
    mesh.triangles.reserve(cdt.triangles.size());
    mesh.edges.reserve(cdt.triangles.size());

    for (const auto& vert : cdt.vertices) {
        mesh.vertices.push_back(Vector2f{ vert.x * 0.1f, vert.y * 0.1f });
    }
    for (const auto& tri : cdt.triangles) {
        mesh.triangles.push_back(Triangle{
            tri.vertices[0],
            tri.vertices[1],
            tri.vertices[2],
        });
        auto ns = std::vector<Mesh::Edge>();
        if (tri.neighbors[0] != CDT::noNeighbor) {
            const auto [_v1, _v2] = shared_edge(tri, cdt.triangles[tri.neighbors[0]]);
            const auto v1 = Vector2f{ cdt.vertices[_v1].x, cdt.vertices[_v1].y } * 0.1f;
            const auto v2 = Vector2f{ cdt.vertices[_v2].x, cdt.vertices[_v2].y } * 0.1f;
            ns.push_back(Mesh::Edge{
                tri.neighbors[0],
                v1 + (v2 - v1) / 2,
                _v1, _v2,
            });
        }
        if (tri.neighbors[1] != CDT::noNeighbor) {
            const auto [_v1, _v2] = shared_edge(tri, cdt.triangles[tri.neighbors[1]]);
            const auto v1 = Vector2f{ cdt.vertices[_v1].x, cdt.vertices[_v1].y } * 0.1f;
            const auto v2 = Vector2f{ cdt.vertices[_v2].x, cdt.vertices[_v2].y } * 0.1f;
            ns.push_back(Mesh::Edge{
                tri.neighbors[1],
                v1 + (v2 - v1) / 2,
                _v1, _v2,
            });
        }
        if (tri.neighbors[2] != CDT::noNeighbor) {
            const auto [_v1, _v2] = shared_edge(tri, cdt.triangles[tri.neighbors[2]]);
            const auto v1 = Vector2f{ cdt.vertices[_v1].x, cdt.vertices[_v1].y } * 0.1f;
            const auto v2 = Vector2f{ cdt.vertices[_v2].x, cdt.vertices[_v2].y } * 0.1f;
            ns.push_back(Mesh::Edge{
                tri.neighbors[2],
                v1 + (v2 - v1) / 2,
                _v1, _v2,
            });
        }

        mesh.edges.push_back(ns);
    }

    BENCH_STEP("data extraction");

    return mesh;
}


#define PI 3.1415f
#define TAU (2.f * PI)

Mesh generate_from_shapes(
        const std::vector<std::vector<Vector2f>>& polys,
        const std::vector<FloatCircle>& circles,
        const std::vector<Vector2f>& fillers,
        u32 circle_res,
        float epsilon
    )
{
    auto verts = std::vector<CDT::V2d<f32>>();
    auto edges = std::vector<CDT::Edge>();
    auto cons = std::vector<CDT::Edge>();

    u32 offset = 0;
    for (auto& p : polys) {
        auto dp = douglas_peucker(p, epsilon);
        verts.reserve(verts.size() + dp.size());
        edges.reserve(edges.size() + dp.size());
        u32 i = 0;
        for (const auto& v : dp) {
            verts.push_back(CDT::V2d<f32>((f32)v.x, (f32)v.y));
            edges.push_back(CDT::Edge(offset + i, offset + ((i + 1) % (dp.size()))));
            i++;
        }

        offset += dp.size();
    }

    const auto theta = TAU / (f32)circle_res;

    for (const auto c : circles) {
        for (u32 i = 0; i < circle_res; i++) {
            const auto v = Vector2f{
                c.pos.x + std::cos(theta * (f32)i) * c.radius,
                c.pos.y + std::sin(theta * (f32)i) * c.radius,
            };
            verts.push_back(CDT::V2d<f32>((f32)v.x, (f32)v.y));
            edges.push_back(CDT::Edge(offset + i, offset + ((i + 1) % (circle_res))));
        }

        offset += circle_res;
    }

    for (const auto p : fillers) {
        verts.push_back(CDT::V2d<f32>((f32)p.x, (f32)p.y));
    }

    auto cdt = CDT::Triangulation<float>();
    cdt.insertVertices(verts);
    cdt.insertEdges(edges);
    cdt.eraseOuterTriangles();

    auto mesh = Mesh();
    mesh.vertices.reserve(cdt.vertices.size());
    mesh.triangles.reserve(cdt.triangles.size());
    mesh.edges.reserve(cdt.triangles.size());

    for (const auto& vert : cdt.vertices) {
        mesh.vertices.push_back(Vector2f{ vert.x, vert.y });
    }
    for (const auto& tri : cdt.triangles) {
        mesh.triangles.push_back(Triangle{
            tri.vertices[0],
            tri.vertices[1],
            tri.vertices[2],
        });
        auto ns = std::vector<Mesh::Edge>();
        if (tri.neighbors[0] != CDT::noNeighbor) {
            const auto [_v1, _v2] = shared_edge(tri, cdt.triangles[tri.neighbors[0]]);
            const auto v1 = Vector2f{ cdt.vertices[_v1].x, cdt.vertices[_v1].y };
            const auto v2 = Vector2f{ cdt.vertices[_v2].x, cdt.vertices[_v2].y };
            ns.push_back(Mesh::Edge{
                tri.neighbors[0],
                v1 + (v2 - v1) / 2,
                _v1, _v2,
            });
        }
        if (tri.neighbors[1] != CDT::noNeighbor) {
            const auto [_v1, _v2] = shared_edge(tri, cdt.triangles[tri.neighbors[1]]);
            const auto v1 = Vector2f{ cdt.vertices[_v1].x, cdt.vertices[_v1].y };
            const auto v2 = Vector2f{ cdt.vertices[_v2].x, cdt.vertices[_v2].y };
            ns.push_back(Mesh::Edge{
                tri.neighbors[1],
                v1 + (v2 - v1) / 2,
                _v1, _v2,
            });
        }
        if (tri.neighbors[2] != CDT::noNeighbor) {
            const auto [_v1, _v2] = shared_edge(tri, cdt.triangles[tri.neighbors[2]]);
            const auto v1 = Vector2f{ cdt.vertices[_v1].x, cdt.vertices[_v1].y };
            const auto v2 = Vector2f{ cdt.vertices[_v2].x, cdt.vertices[_v2].y };
            ns.push_back(Mesh::Edge{
                tri.neighbors[2],
                v1 + (v2 - v1) / 2,
                _v1, _v2,
            });
        }

        mesh.edges.push_back(ns);
    }

    return mesh;
}

}


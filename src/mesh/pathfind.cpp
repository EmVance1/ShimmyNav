#include "lib.h"
#include <queue>
#include <unordered_map>
#include "funnel.h"


namespace nav {

static float Euclidean(Vector2f a, Vector2f b) { const auto d = Vector2f(b-a); return std::sqrt(d.x * d.x + d.y * d.y); }
static float Chebyshev(Vector2f a, Vector2f b) { const auto d = Vector2f(b-a); return std::max(std::abs(d.x), std::abs(d.y)); }
const auto H = Chebyshev;


struct AStarTuple {
    size_t id;
    size_t parent;
    Vector2f pos;
    float g_cost = std::numeric_limits<float>::infinity();
    float f_cost = std::numeric_limits<float>::infinity();
};
bool operator< (const AStarTuple& a, const AStarTuple& b) { return a.f_cost <  b.f_cost; }
bool operator> (const AStarTuple& a, const AStarTuple& b) { return a.f_cost >  b.f_cost; }
bool operator<=(const AStarTuple& a, const AStarTuple& b) { return a.f_cost <= b.f_cost; }
bool operator>=(const AStarTuple& a, const AStarTuple& b) { return a.f_cost >= b.f_cost; }
bool operator==(const AStarTuple& a, const AStarTuple& b) { return a.id == b.id && a.parent == b.parent && a.g_cost == b.g_cost && a.f_cost == b.f_cost; }
bool operator!=(const AStarTuple& a, const AStarTuple& b) { return !(a == b); }

template<typename T>
using LowPrioQueue = std::priority_queue<T, std::vector<T>, std::greater<T>>;
template<typename T>
using HighPrioQueue = std::priority_queue<T, std::vector<T>, std::less<T>>;


// finds i such that B is the ith neighbor of A
static size_t get_neighbor_index(const Mesh& mesh, size_t a, size_t b) {
    const auto& edges = mesh.edges[a];
    size_t i = 0;
    for (const auto& e : edges) {
        if (e.index == b) {
            return i;
        }
        i++;
    }
    return SIZE_MAX;
}

Path Mesh::pathfind(Vector2f begin, Vector2f end) const {
    const auto begin_idx = get_triangle(begin, 0.05f);
    const auto end_idx = get_triangle(end, 0.f);
    if (!begin_idx.has_value()) { return {}; }
    if (!end_idx.has_value()) { return {}; }
    if (begin_idx == end_idx) { return { begin, end }; }

    auto queue = LowPrioQueue<AStarTuple>();
    queue.push(AStarTuple{ *begin_idx, *begin_idx, begin, 0, H(begin, end) });

    auto lut = std::unordered_map<size_t, AStarTuple>();
    lut[*begin_idx] = AStarTuple{ *begin_idx, *begin_idx, begin, 0, H(begin, end) };

    while (!queue.empty()) {
        const auto current = queue.top();
        queue.pop();

        if (current.id == *end_idx) {
            if (lut.find(*end_idx) != lut.end()) {
                auto cur = *end_idx;
                auto cur_pos = end;
                auto total_path = std::vector<CrossInfo>{ CrossInfo{ cur, SIZE_MAX } };
                auto it = lut.find(cur);
                while (it != lut.end() && (it->second.parent != cur)) {
                    const auto n = get_neighbor_index(*this, it->second.parent, cur);
                    cur = it->second.parent;
                    cur_pos = lut.at(cur).pos;
                    total_path.insert(total_path.begin(), CrossInfo{ cur, n });
                    it = lut.find(cur);
                }
                return funnel(*this, std::move(total_path), begin, end);
            } else {
                return {};
            }
        }

        const auto c_g_cost = lut[current.id].g_cost;
        for (size_t i = 0; i < edges[current.id].size(); i++) {
            const auto n_id = edges[current.id][i].index;
            const auto dist = Euclidean(lut[current.id].pos, edges[current.id][i].center);
            const auto g_cost_tentative = c_g_cost + dist * triangles[edges[current.id][i].index].weight;
            if (lut.find(n_id) == lut.end()) {
                lut[n_id] = AStarTuple{
                    n_id,
                    n_id,
                    edges[current.id][i].center,
                    std::numeric_limits<float>::infinity(),
                    std::numeric_limits<float>::infinity(),
                };
            }
            const auto g_cost_neighbor = lut[n_id].g_cost;
            if (g_cost_tentative < g_cost_neighbor) {
                lut[n_id] = AStarTuple{
                    n_id,
                    current.id,
                    edges[current.id][i].center,
                    g_cost_tentative,
                    g_cost_tentative + H(edges[current.id][i].center, end),
                };
                queue.push(lut[n_id]);
            }
        }
    }

    return {};
}

/*
IndexedPath Mesh::pathfind_indexed(Vector2f begin, Vector2f end) const {
    const auto begin_idx = get_triangle(begin, 0.05f);
    const auto end_idx = get_triangle(end, 0.f);
    if (!begin_idx.has_value()) { return {}; }
    if (!end_idx.has_value()) { return {}; }
    if (begin_idx == end_idx) { return { IndexedPoint{ begin, *begin_idx }, IndexedPoint{ end, *end_idx } }; }

    auto queue = LowPrioQueue<AStarTuple>();
    queue.push(AStarTuple{ *begin_idx, *begin_idx, begin, 0, H(begin, end) });

    auto lut = std::unordered_map<size_t, AStarTuple>();
    lut[*begin_idx] = AStarTuple{ *begin_idx, *begin_idx, begin, 0, H(begin, end) };

    while (!queue.empty()) {
        const auto current = queue.top();
        queue.pop();

        if (current.id == *end_idx) {
            if (lut.find(*end_idx) != lut.end()) {
                auto cur = *end_idx;
                auto cur_pos = end;
                auto total_path = std::vector<CrossInfo>{ CrossInfo{ cur, SIZE_MAX } };
                auto it = lut.find(cur);
                while (it != lut.end() && (it->second.parent != cur)) {
                    const auto n = get_neighbor_index(*this, it->second.parent, cur);
                    cur = it->second.parent;
                    cur_pos = lut.at(cur).pos;
                    total_path.insert(total_path.begin(), CrossInfo{ cur, n });
                    it = lut.find(cur);
                }
                return funnel_indexed(*this, std::move(total_path), IndexedPoint{ begin, *begin_idx }, IndexedPoint{ end, *end_idx } });
            } else {
                return {};
            }
        }

        const auto c_g_cost = lut[current.id].g_cost;
        for (size_t i = 0; i < edges[current.id].size(); i++) {
            const auto n_id = edges[current.id][i].index;
            const auto dist = Euclidean(lut[current.id].pos, edges[current.id][i].center);
            const auto g_cost_tentative = c_g_cost + dist * triangles[edges[current.id][i].index].weight;
            if (lut.find(n_id) == lut.end()) {
                lut[n_id] = AStarTuple{
                    n_id,
                    n_id,
                    edges[current.id][i].center,
                    std::numeric_limits<float>::infinity(),
                    std::numeric_limits<float>::infinity(),
                };
            }
            const auto g_cost_neighbor = lut[n_id].g_cost;
            if (g_cost_tentative < g_cost_neighbor) {
                lut[n_id] = AStarTuple{
                    n_id,
                    current.id,
                    edges[current.id][i].center,
                    g_cost_tentative,
                    g_cost_tentative + H(edges[current.id][i].center, end),
                };
                queue.push(lut[n_id]);
            }
        }
    }

    return {};
}
*/

}

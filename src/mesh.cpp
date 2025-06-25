#include "lib.h"
#include <queue>
#include <unordered_map>


namespace nav {

static float Euclidean(Vector2f a, Vector2f b) { const auto d = Vector2f(b-a); return std::sqrt(d.x * d.x + d.y * d.y); }
static float Chebyshev(Vector2f a, Vector2f b) { const auto d = Vector2f(b-a); return std::max(std::abs(d.x), std::abs(d.y)); }
const auto H = Chebyshev;


std::optional<size_t> NavMesh::get_triangle(Vector2f p, float error) const {
    size_t i = 0;
    for (const auto& tri : triangles) {
        if (tri.contains(vertices.data(), p)) {
            return i;
        }
        i++;
    }
    if (error == 0.f) { return {}; }
    i = 0;
    for (const auto& tri : triangles) {
        if (tri.contains_with_error(vertices.data(), p, error)) {
            return i;
        }
        i++;
    }
    return {};
}


struct AStarTuple {
    size_t id;
    size_t parent;
    Vector2f pos;
    float g_cost = std::numeric_limits<float>::infinity();
    float f_cost = std::numeric_limits<float>::infinity();
};
bool operator<(const AStarTuple& a, const AStarTuple& b) { return a.f_cost < b.f_cost; }
bool operator>(const AStarTuple& a, const AStarTuple& b) { return a.f_cost > b.f_cost; }
bool operator<=(const AStarTuple& a, const AStarTuple& b) { return a.f_cost <= b.f_cost; }
bool operator>=(const AStarTuple& a, const AStarTuple& b) { return a.f_cost >= b.f_cost; }
bool operator==(const AStarTuple& a, const AStarTuple& b) { return a.id == b.id && a.parent == b.parent && a.g_cost == b.g_cost && a.f_cost == b.f_cost; }
bool operator!=(const AStarTuple& a, const AStarTuple& b) { return !(a == b); }

template<typename T>
using LowPrioQueue = std::priority_queue<T, std::vector<T>, std::greater<T>>;
template<typename T>
using HighPrioQueue = std::priority_queue<T, std::vector<T>, std::less<T>>;

struct CrossInfo {
    size_t next_index;
    size_t neighbor_index;
};


// (lhs <-- rhs) forms positive (CCW) angle arc
// (lhs --> rhs) forms negative (CW)  angle arc
static bool pos_angle(Vector2f lhs, Vector2f rhs) {
    return rhs.perp_cw().dot(lhs) < 0;
}


struct FunnelVertex {
    usize vertex_index;
    usize list_index;
    Vector2f pos;
};

static std::vector<Vector2f> funnel(const NavMesh& mesh, std::vector<CrossInfo>&& path, Vector2f begin, Vector2f end) {
    if (path.size() == 2 && path[0].next_index == path[1].next_index) {
        return { begin, end };
    }

    auto list_l = std::vector<FunnelVertex>();
    auto list_r = std::vector<FunnelVertex>();
    for (CrossInfo i : path) {
        if (i.neighbor_index != SIZE_MAX) {
            const auto a = mesh.edges[i.next_index][i.neighbor_index].a;
            const auto b = mesh.edges[i.next_index][i.neighbor_index].b;
            list_l.push_back({ a, list_l.size(), mesh.vertices[a] });
            list_r.push_back({ b, list_r.size(), mesh.vertices[b] });
        }
    }
    list_l.push_back({ SIZE_MAX, list_l.size(), end });
    list_r.push_back({ SIZE_MAX, list_r.size(), end });

    auto result = std::vector<Vector2f>{ begin };
    auto root = begin;
    auto arm_l = list_l.front();
    auto arm_r = list_r.front();
    usize idx_l = 0;
    usize idx_r = 0;

    while (true) {
        idx_l++;
        if (idx_l == list_l.size()) {
            result.push_back(end);
            return result;
        } else {
            const auto pos_new = list_l[idx_l].pos;
            const auto pos_old = arm_l.pos;
            if (!pos_angle(pos_old - root, pos_new - root)) {
                const auto pos_right = arm_r.pos;
                if (pos_angle(pos_new - root, pos_right - root)) {
                    root = pos_right;
                    result.push_back(root);
                    idx_r = arm_r.list_index+1;
                    arm_r = list_r[idx_r];
                    idx_l = idx_r;
                    arm_l = list_l[idx_l];
                } else {
                    arm_l = list_l[idx_l];
                }
            }
        }

        idx_r++;
        if (idx_r == list_r.size()) {
            result.push_back(end);
            return result;
        } else {
            const auto pos_new = list_r[idx_r].pos;
            const auto pos_old = arm_r.pos;
            if (!pos_angle(pos_new - root, pos_old - root)) {
                const auto pos_left = arm_l.pos;
                if (pos_angle(pos_left - root, pos_new - root)) {
                    root = pos_left;
                    result.push_back(root);
                    idx_l = arm_l.list_index+1;
                    arm_l = list_l[idx_l];
                    idx_r = idx_l;
                    arm_r = list_r[idx_r];
                } else {
                    arm_r = list_r[idx_r];
                }
            }
        }
    }

    return {};
}


// finds i such that B is the ith neighbor of A
static size_t get_neighbor_index(const NavMesh& mesh, size_t a, size_t b) {
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


std::vector<Vector2f> NavMesh::pathfind(Vector2f begin, Vector2f end) const {
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
            const auto g_cost_tentative = c_g_cost + dist;
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


}

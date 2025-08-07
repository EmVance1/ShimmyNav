#include "funnel.h"


namespace nav {

Path edge_to_edge(const Mesh& mesh, std::vector<CrossInfo>&& path, Vector2f begin, Vector2f end) {
    if (path.size() == 2 && path[0].next_index == path[1].next_index) {
        return { begin, end };
    }

    auto result = std::vector<Vector2f>();
    result.push_back(begin);
    for (CrossInfo i : path) {
        if (i.neighbor_index != SIZE_MAX) {
            result.push_back(mesh.edges[i.next_index][i.neighbor_index].center);
        }
    }
    result.push_back(end);
    return result;
}


struct FunnelVertex {
    usize vertex_index;
    usize list_index;
    Vector2f pos;
};

// (lhs <-- rhs) forms positive (CCW) angle arc
// (lhs --> rhs) forms negative (CW)  angle arc
static bool pos_angle(Vector2f lhs, Vector2f rhs) {
    return rhs.perp_cw().dot(lhs) < 0;
}

Path funnel(const Mesh& mesh, std::vector<CrossInfo>&& path, Vector2f begin, Vector2f end) {
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
        if (++idx_l == list_l.size()) {
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
                    idx_r = arm_r.list_index + 1;
                    arm_r = list_r[idx_r];
                    idx_l = idx_r;
                }
                arm_l = list_l[idx_l];
            }
        }

        if (++idx_r == list_r.size()) {
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
                    idx_l = arm_l.list_index + 1;
                    arm_l = list_l[idx_l];
                    idx_r = idx_l;
                }
                arm_r = list_r[idx_r];
            }
        }
    }

    return {};
}

/*
IndexedPath funnel_indexed(const Mesh& mesh, std::vector<CrossInfo>&& path, IndexedPoint begin, IndexedPoint end) {
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
        if (++idx_l == list_l.size()) {
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
                }
                arm_l = list_l[idx_l];
            }
        }

        if (++idx_r == list_r.size()) {
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
                }
                arm_r = list_r[idx_r];
            }
        }
    }

    return {};
}
*/

}

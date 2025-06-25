#include "lib.h"
#include <cmath>
#include "slice.h"


namespace nav {

template<typename T>
static f32 perp_dist(Vector2<T> l0, Vector2<T> l1, Vector2<T> p) {
    const T num = (l1.y - l0.y) * p.x - (l1.x - l0.x) * p.y + l1.x * l0.y - l1.y * l0.x;
    const T den = (l1.y - l0.y) * (l1.y - l0.y) + (l1.x - l0.x) * (l1.x - l0.x);
    return std::abs((f32)num) / std::sqrt((f32)den);
}

template<typename T>
static f32 euclid_dist(Vector2<T> p0, Vector2<T> p1) {
    return std::sqrt((f32)((p0.x - p1.x) * (p0.x - p1.x) + (p0.y - p1.y) * (p0.y - p1.y)));
}


static void douglas_peucker_impl(Slice<Vector2i> segment, f32 epsilon, std::vector<Vector2i>& result) {
    const usize end = segment.len - 1;
    if (segment.len == 1) { result.push_back(segment[0]); return; }
    if (segment.len == 2) { result.push_back(segment[0]); result.push_back(segment[end]); return; }

    f32 dmax = 0;
    usize index = 0;
    if (segment[0] == segment[end]) {
        for (usize i = 1; i < end; i++) {
            const f32 d = euclid_dist(segment[0], segment[i]);
            if (d > dmax) {
                dmax = d;
                index = i;
            }
        }
    } else {
        for (usize i = 1; i < end; i++) {
            const f32 d = perp_dist(segment[0], segment[end], segment[i]);
            if (d > dmax) {
                dmax = d;
                index = i;
            }
        }
    }

    if (dmax > epsilon) {
        douglas_peucker_impl(segment.slice(0, index+1), epsilon, result);
        result.pop_back();
        douglas_peucker_impl(segment.slice(index, end+1), epsilon, result);
        return;
    } else {
        result.push_back(segment[0]);
        result.push_back(segment[end]);
        return;
    }
}

static void douglas_peucker_impl(Slice<Vector2f> segment, f32 epsilon, std::vector<Vector2f>& result) {
    const usize end = segment.len - 1;
    if (segment.len == 1) { result.push_back(segment[0]); return; }
    if (segment.len == 2) { result.push_back(segment[0]); result.push_back(segment[end]); return; }

    f32 dmax = 0;
    usize index = 0;
    if (segment[0].cmp(segment[end]) < 0.001f) {
        for (usize i = 1; i < end; i++) {
            const f32 d = euclid_dist(segment[0], segment[i]);
            if (d > dmax) {
                dmax = d;
                index = i;
            }
        }
    } else {
        for (usize i = 1; i < end; i++) {
            const f32 d = perp_dist(segment[0], segment[end], segment[i]);
            if (d > dmax) {
                dmax = d;
                index = i;
            }
        }
    }

    if (dmax > epsilon) {
        douglas_peucker_impl(segment.slice(0, index+1), epsilon, result);
        result.pop_back();
        douglas_peucker_impl(segment.slice(index, end+1), epsilon, result);
        return;
    } else {
        result.push_back(segment[0]);
        result.push_back(segment[end]);
        return;
    }
}


std::vector<Vector2i> douglas_peucker(const std::vector<Vector2i>& segment, f32 epsilon) {
    auto result = std::vector<Vector2i>();
    douglas_peucker_impl(Slice<Vector2i>::from_vec(segment), epsilon, result);
    return result;
}

std::vector<Vector2f> douglas_peucker(const std::vector<Vector2f>& segment, f32 epsilon) {
    auto result = std::vector<Vector2f>();
    douglas_peucker_impl(Slice<Vector2f>::from_vec(segment), epsilon, result);
    return result;
}

}


#include "lib.h"
#include "gen_internal.h"
#include <unordered_map>
#include <BS_thread_pool.hpp>

#include "bench.h"

namespace nav {

struct Vector2iHash {
    constexpr size_t operator()(const Vector2i& vec) const {
        const size_t p1 = 73856093;
        const size_t p2 = 19349663;
        return ((size_t)vec.x * p1) ^ ((size_t)vec.y * p2);
    }
};

template<typename T>
struct Vector2Cmp {
    constexpr bool operator()(const Vector2<T>& lhs, const Vector2<T>& rhs) const {
        if (lhs.y > rhs.y) { return false; }
        if (lhs.y < rhs.y) { return true; }
        if (lhs.x < rhs.x) { return true; }
        return false;
    }
};


struct Contour {
    uint8_t count;
    Vector2i a;
    Vector2i b;
    Vector2i c;
    Vector2i d;

    constexpr Contour() : count(0) {}
    constexpr Contour(Vector2i _a, Vector2i _b) : count(2), a(_a), b(_b) {}
    constexpr Contour(Vector2i _a, Vector2i _b, Vector2i _c, Vector2i _d) : count(4), a(_a), b(_b), c(_c), d(_d) {}
};


constexpr static Contour MS_LUT[] = {
    Contour(),
    Contour({ 0,  5 },  { 5,  10 }),
    Contour({ 5,  10 }, { 10, 5 }),
    Contour({ 0,  5 },  { 10, 5 }),
    Contour({ 10, 5 },  { 5,  0 }),
    Contour({ 0,  5 },  { 5,  0 }, { 10, 5 }, { 5, 10 }),
    Contour({ 5,  10 }, { 5,  0 }),
    Contour({ 0,  5 },  { 5,  0 }),
    Contour({ 5,  0 },  { 0,  5 }),
    Contour({ 5,  0 },  { 5,  10 }),
    Contour({ 5,  0 },  { 10, 5 }, { 5, 10 }, { 0, 5 }),
    Contour({ 5,  0 },  { 10, 5 }),
    Contour({ 10, 5 },  { 0,  5 }),
    Contour({ 10, 5 },  { 5,  10 }),
    Contour({ 5, 10 },  { 0,  5 }),
    Contour(),
};


using EdgeMap = std::unordered_map<Vector2i, Vector2i, Vector2iHash>;

static std::vector<Polygon> concat_edges(EdgeMap& map) {
    auto shapes = std::vector<Polygon>();

    while (!map.empty()) {
        auto& chain = shapes.emplace_back();
        chain.push_back(map.begin()->first);
        chain.push_back(map.begin()->second);
        map.erase(map.begin());
        while (true) {
            const auto it = map.find(chain.back());
            if (it == map.end()) {
                break;
            } else if (it->second == chain.front()) {
                chain.push_back(it->second);
                map.erase(it);
                break;
            } else {
                chain.push_back(it->second);
                map.erase(it);
            }
        }
    }

    return shapes;
}


constexpr static u8 wall_lookup(const u8* grid, isize x, isize y, usize width, usize height, usize stride, Predicate P) {
    if (x == -1 || y == -1) { return true; }
    if (x == (isize)width || y == (isize)height) { return true; }
    return (u8)P(grid + ((y * width + x) * stride));
}

constexpr static u8 wall_lookup(const u8* grid, isize x, isize y, usize width, usize height, usize stride, size_t index) {
    if (x == -1 || y == -1) { return true; }
    if (x == (isize)width || y == (isize)height) { return true; }
    return grid[(y * width + x) * stride + index];
}


std::vector<Polygon> marching_squares(const uint8_t* data, size_t width, size_t height, size_t stride, Predicate P) {
    auto edges = EdgeMap();

    for (usize i = 0; i < (width + 2) * (height + 2); i++) {
        const isize x = (isize)i % (isize)(width + 2) - 1;
        const isize y = (isize)i / (isize)(width + 2) - 1;

        const u8 bits = (u8)(wall_lookup(data, x,   y,   width, height, stride, P) << 3) |
                        (u8)(wall_lookup(data, x+1, y,   width, height, stride, P) << 2) |
                        (u8)(wall_lookup(data, x+1, y+1, width, height, stride, P) << 1) |
                        (u8)(wall_lookup(data, x,   y+1, width, height, stride, P));
        const auto con = MS_LUT[bits];
        if (con.count > 0) {
            const auto a = Vector2i{ 10*(i32)x + con.a.x, 10*(i32)y + con.a.y };
            const auto b = Vector2i{ 10*(i32)x + con.b.x, 10*(i32)y + con.b.y };
            edges.emplace(a, b);
        }
        if (con.count > 2) {
            const auto c = Vector2i{ 10*(i32)x + con.c.x, 10*(i32)y + con.c.y };
            const auto d = Vector2i{ 10*(i32)x + con.d.x, 10*(i32)y + con.d.y };
            edges.emplace(c, d);
        }
    }

    return concat_edges(edges);
}

std::vector<Polygon> marching_squares(const uint8_t* data, size_t width, size_t height, size_t stride, size_t index) {
    auto edges = EdgeMap();

    for (usize i = 0; i < (width + 2) * (height + 2); i++) {
        const isize x = (isize)i % (isize)(width + 2) - 1;
        const isize y = (isize)i / (isize)(width + 2) - 1;

        const u8 bits = (u8)(wall_lookup(data, x,   y,   width, height, stride, index) << 3) |
                        (u8)(wall_lookup(data, x+1, y,   width, height, stride, index) << 2) |
                        (u8)(wall_lookup(data, x+1, y+1, width, height, stride, index) << 1) |
                        (u8)(wall_lookup(data, x,   y+1, width, height, stride, index));
        const auto con = MS_LUT[bits];
        if (con.count > 0) {
            const auto a = Vector2i{ 10*(i32)x + con.a.x, 10*(i32)y + con.a.y };
            const auto b = Vector2i{ 10*(i32)x + con.b.x, 10*(i32)y + con.b.y };
            edges.emplace(a, b);
        }
        if (con.count > 2) {
            const auto c = Vector2i{ 10*(i32)x + con.c.x, 10*(i32)y + con.c.y };
            const auto d = Vector2i{ 10*(i32)x + con.d.x, 10*(i32)y + con.d.y };
            edges.emplace(c, d);
        }
    }

    return concat_edges(edges);
}


std::vector<Polygon> floodfill(const uint8_t* data, size_t width, size_t height, size_t stride, Predicate P) {
    auto result = EdgeMap();
    u8 left = 0;
    u8 right = 0;

    for (usize i = 0; i < width * height; i++) {
        const isize x = i % width;
        const isize y = i / width;
        if (x == 0 || !right) {
            left = 0;
            right = !wall_lookup(data, x, y, width, height, stride, P);
        }

        if (right) {
            if (wall_lookup(data, x, y-1, width, height, stride, P)) {
                result.emplace(Vector2i{ (i32)x*10 + 10, (i32)y*10 },      Vector2i{ (i32)x*10,      (i32)y*10 });
            }
            if (!left) {
                result.emplace(Vector2i{ (i32)x*10,      (i32)y*10 },      Vector2i{ (i32)x*10,      (i32)y*10 + 10 });
            }
            if (wall_lookup(data, x, y+1, width, height, stride, P)) {
                result.emplace(Vector2i{ (i32)x*10,      (i32)y*10 + 10 }, Vector2i{ (i32)x*10 + 10, (i32)y*10 + 10 });
            }
            if (wall_lookup(data, x+1, y, width, height, stride, P)) {
                result.emplace(Vector2i{ (i32)x*10 + 10, (i32)y*10 + 10 }, Vector2i{ (i32)x*10 + 10, (i32)y*10 });
                right = 0;
            } else {
                right = 1;
            }
            left = 1;
        }
    }

    return concat_edges(result);
}

std::vector<Polygon> floodfill(const u8* data, size_t width, size_t height, size_t stride, size_t index) {
    auto result = EdgeMap();

    for (usize i = 0; i < width * height; i++) {
        const isize x = i % width;
        const isize y = i / width;

        if (!data[(y * width + x) * stride + index]) {
            /*
            if (y == 0               || data[((y-1) * width + x) * stride + index]) {
                const auto it = result.find(Vector2i{ (i32)x*10, (i32)y*10 });
                if (it != result.end() && it->second.y == y*10) {
                    const auto v = it->second;
                    result.erase(it);
                    result.emplace(Vector2i{ (i32)x*10 + 10, (i32)y*10 }, v);
                } else {
                    result.emplace(Vector2i{ (i32)x*10 + 10, (i32)y*10 }, Vector2i{ (i32)x*10,      (i32)y*10 });
                }
            }
            if (x == 0               || data[(y * width + (x-1)) * stride + index]) {
                const auto it = result.find(Vector2i{ (i32)x*10, (i32)y*10 - 10 });
                if (it != result.end() && it->second.x == x*10) {
                    it->second = Vector2i{ (i32)x*10, (i32)y*10 + 10 };
                } else {
                    result.emplace(Vector2i{ (i32)x*10, (i32)y*10 }, Vector2i{ (i32)x*10, (i32)y*10 + 10 });
                }
            }
            if (y == (isize)height-1 || data[((y+1) * width + x) * stride + index]) {
                const auto it = result.find(Vector2i{ (i32)x*10 - 10, (i32)y*10 + 10 });
                if (it != result.end() && it->second.y == y*10 + 10) {
                    it->second = Vector2i{ (i32)x*10 + 10, (i32)y*10 + 10 };
                } else {
                    result.emplace(Vector2i{ (i32)x*10, (i32)y*10 + 10 }, Vector2i{ (i32)x*10 + 10, (i32)y*10 + 10 });
                }
            }
            if (x == (isize)width-1  || data[(y * width + (x+1)) * stride + index]) {
                const auto it = result.find(Vector2i{ (i32)x*10 + 10, (i32)y*10 });
                if (it != result.end() && it->second.x == x*10 + 10) {
                    const auto v = it->second;
                    result.erase(it);
                    result.emplace(Vector2i{ (i32)x*10 + 10, (i32)y*10 + 10 }, v);
                } else {
                    result.emplace(Vector2i{ (i32)x*10 + 10, (i32)y*10 + 10 }, Vector2i{ (i32)x*10 + 10, (i32)y*10 });
                }
            }
            */
            if (y == 0               || data[((y-1) * width + x) * stride + index]) {
                result.emplace(Vector2i{ (i32)x*10 + 10, (i32)y*10 },      Vector2i{ (i32)x*10,      (i32)y*10 });
            }
            if (x == 0               || data[(y * width + (x-1)) * stride + index]) {
                result.emplace(Vector2i{ (i32)x*10,      (i32)y*10 },      Vector2i{ (i32)x*10,      (i32)y*10 + 10 });
            }
            if (y == (isize)height-1 || data[((y+1) * width + x) * stride + index]) {
                result.emplace(Vector2i{ (i32)x*10,      (i32)y*10 + 10 }, Vector2i{ (i32)x*10 + 10, (i32)y*10 + 10 });
            }
            if (x == (isize)width-1  || data[(y * width + (x+1)) * stride + index]) {
                result.emplace(Vector2i{ (i32)x*10 + 10, (i32)y*10 + 10 }, Vector2i{ (i32)x*10 + 10, (i32)y*10 });
            }
        }
    }

    // std::cout << "==============" << result.size() << "\n";

    return concat_edges(result);
}


std::vector<Polygon> floodfill_threaded(const u8* data, size_t width, size_t height, size_t stride, size_t index) {
    auto pool = BS::thread_pool();

    const auto count = 8;
    const auto len = width * height;
    const auto block = ((isize)len / count);

    auto future = pool.submit_sequence(0, count, [=](size_t i){
        auto res = EdgeMap();
        const auto begin = (isize)i * block;

        for (isize j = begin; j < begin + block && j < (isize)len; j++) {
            const isize x = j % width;
            const isize y = j / width;
            if (!data[(y * width + x) * stride + index]) {
                /*
                if (y == 0               || data[((y-1) * width + x) * stride + index]) {
                    const auto it = res.find(Vector2i{ (i32)x*10, (i32)y*10 });
                    if (it != res.end() && it->second.y == y*10) {
                        const auto v = it->second;
                        res.erase(it);
                        res.emplace(Vector2i{ (i32)x*10 + 10, (i32)y*10 }, v);
                    } else {
                        res.emplace(Vector2i{ (i32)x*10 + 10, (i32)y*10 }, Vector2i{ (i32)x*10,      (i32)y*10 });
                    }
                }
                if (x == 0               || data[(y * width + (x-1)) * stride + index]) {
                    const auto it = res.find(Vector2i{ (i32)x*10, (i32)y*10 - 10 });
                    if (it != res.end() && it->second == Vector2i{ (i32)x*10, (i32)y*10 }) {
                        it->second = Vector2i{ (i32)x*10, (i32)y*10 + 10 };
                    } else {
                        res.emplace(Vector2i{ (i32)x*10, (i32)y*10 }, Vector2i{ (i32)x*10, (i32)y*10 + 10 });
                    }
                }
                if (y == (isize)height-1 || data[((y+1) * width + x) * stride + index]) {
                    const auto it = res.find(Vector2i{ (i32)x*10 - 10, (i32)y*10 + 10 });
                    if (it != res.end() && it->second == Vector2i{ (i32)x*10, (i32)y*10 + 10 }) {
                        it->second = Vector2i{ (i32)x*10 + 10, (i32)y*10 + 10 };
                    } else {
                        res.emplace(Vector2i{ (i32)x*10, (i32)y*10 + 10 }, Vector2i{ (i32)x*10 + 10, (i32)y*10 + 10 });
                    }
                }
                if (x == (isize)width-1  || data[(y * width + (x+1)) * stride + index]) {
                    const auto it = res.find(Vector2i{ (i32)x*10 + 10, (i32)y*10 });
                    if (it != res.end() && it->second.x == x*10 + 10) {
                        const auto v = it->second;
                        res.erase(it);
                        res.emplace(Vector2i{ (i32)x*10 + 10, (i32)y*10 + 10 }, v);
                    } else {
                        res.emplace(Vector2i{ (i32)x*10 + 10, (i32)y*10 + 10 }, Vector2i{ (i32)x*10 + 10, (i32)y*10 });
                    }
                }
                */
                if (y == 0               || data[((y-1) * width + x) * stride + index]) {
                    res.emplace(Vector2i{ (i32)x*10 + 10, (i32)y*10 },      Vector2i{ (i32)x*10,      (i32)y*10 });
                }
                if (x == 0               || data[(y * width + (x-1)) * stride + index]) {
                    res.emplace(Vector2i{ (i32)x*10,      (i32)y*10 },      Vector2i{ (i32)x*10,      (i32)y*10 + 10 });
                }
                if (y == (isize)height-1 || data[((y+1) * width + x) * stride + index]) {
                    res.emplace(Vector2i{ (i32)x*10,      (i32)y*10 + 10 }, Vector2i{ (i32)x*10 + 10, (i32)y*10 + 10 });
                }
                if (x == (isize)width-1  || data[(y * width + (x+1)) * stride + index]) {
                    res.emplace(Vector2i{ (i32)x*10 + 10, (i32)y*10 + 10 }, Vector2i{ (i32)x*10 + 10, (i32)y*10 });
                }
            }
        }

        return res;
    });

    auto result = EdgeMap();
    for (const auto& m : future.get()) {
        result.insert(m.begin(), m.end());
    }

    // std::cout << "==============" << result.size() << "\n";

    return concat_edges(result);
}

}

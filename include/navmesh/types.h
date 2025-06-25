#pragma once
#include <cmath>
#include <cstdint>
#include <vector>
#include <string>
#include <optional>


namespace nav {

typedef int8_t  i8;
typedef int16_t i16;
typedef int32_t i32;
typedef int64_t i64;

typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

typedef float  flt32_t;
typedef double flt64_t;

typedef flt32_t f32;
typedef flt64_t f64;

typedef u64 usize;
typedef i64 isize;


template<typename T>
struct Vector2 {
    T x = 0, y = 0;

    constexpr f32 dot(Vector2 rhs) const { return x * rhs.x + y * rhs.y; }
    constexpr f32 length_squared() const { return x * x + y * y; }
    f32 length() const { return std::sqrt(x * x + y * y); }
    constexpr Vector2 perp_cw()   const { return Vector2{ y, -x }; }
    constexpr Vector2 perp_ccw()  const { return Vector2{ -y, x }; }
    Vector2 normalise() const { const auto l = length(); return (*this) * (1.f / l); }
    constexpr T cmp(Vector2 rhs) const { const auto a = x - rhs.x; const auto b = y - rhs.y; return a > b ? a : b; }
    f32 angle(Vector2 rhs) const { const auto a = dot(rhs); const auto b = length() * rhs.length(); return std::acos(a / b); }

    constexpr Vector2& operator+=(Vector2<T> rhs) { x += rhs.x; y += rhs.y; return *this; }
    constexpr Vector2& operator-=(Vector2<T> rhs) { x -= rhs.x; y -= rhs.y; return *this; }
    constexpr Vector2& operator*=(f32 rhs) { x *= rhs; y *= rhs; return *this; }
    constexpr Vector2& operator/=(f32 rhs) { x /= rhs; y /= rhs; return *this; }
};
using Vector2i = Vector2<i32>;
using Vector2f = Vector2<f32>;

template<typename T> constexpr Vector2<T> operator+(Vector2<T> lhs, Vector2<T> rhs) { return Vector2<T>{ lhs.x + rhs.x, lhs.y + rhs.y }; }
template<typename T> constexpr Vector2<T> operator-(Vector2<T> lhs, Vector2<T> rhs) { return Vector2<T>{ lhs.x - rhs.x, lhs.y - rhs.y }; }
template<typename T> constexpr Vector2<T> operator*(Vector2<T> lhs, f32 rhs) { return Vector2<T>{ lhs.x * rhs, lhs.y * rhs }; }
template<typename T> constexpr Vector2<T> operator/(Vector2<T> lhs, f32 rhs) { return Vector2<T>{ lhs.x / rhs, lhs.y / rhs }; }
template<typename T> constexpr Vector2<T> operator*(f32 lhs, Vector2<T> rhs) { return Vector2<T>{ rhs.x * lhs, rhs.y * lhs }; }
template<typename T> constexpr Vector2<T> operator/(f32 lhs, Vector2<T> rhs) { return Vector2<T>{ rhs.x / lhs, rhs.y / lhs }; }
template<typename T> constexpr bool operator==(Vector2<T> lhs, Vector2<T> rhs) { return lhs.x == rhs.x && lhs.y == rhs.y; }
template<typename T> constexpr bool operator!=(Vector2<T> lhs, Vector2<T> rhs) { return !(lhs == rhs); }


template<typename T>
struct Circle {
    Vector2<T> pos;
    T radius = 0;
};
using IntCircle = Circle<i32>;
using FloatCircle = Circle<f32>;


struct Triangle {
    usize A;
    usize B;
    usize C;

    constexpr Vector2f centroid(const Vector2f* vertices) const;
    constexpr Vector2f circumcenter(const Vector2f* vertices) const;
    constexpr bool contains(const Vector2f* vertices, Vector2f p, bool with_corners = true) const;
    constexpr bool contains_with_error(const Vector2f* vertices, Vector2f p, f32 error) const;
};


using Path = std::vector<Vector2f>;
// using Polygon = std::vector<Vector2f>;

struct NavMesh {
    struct Edge {
        usize index;
        Vector2f center;
        usize a;
        usize b;
    };
    std::vector<Vector2f> vertices;
    std::vector<Triangle> triangles;
    std::vector<std::vector<Edge>> edges;

    void write_file(const std::string& filename, f32 scale = 1.f) const;
    static NavMesh read_file(const std::string& filename, f32 scale = 1.f);

    std::optional<size_t> get_triangle(Vector2f p, f32 error = 0.f) const;

    Path pathfind(Vector2f begin, Vector2f end) const;
};

}

#pragma once
#include <cmath>
#include <vector>
#include "core.h"


namespace nav {

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
template<typename T> constexpr bool operator==(Vector2<T> lhs, Vector2<T> rhs) { return lhs.x == rhs.x && lhs.y == rhs.y; }
template<typename T> constexpr bool operator!=(Vector2<T> lhs, Vector2<T> rhs) { return !(lhs == rhs); }


using Polygon = std::vector<Vector2i>;

template<typename T>
struct Circle {
    Vector2<T> pos;
    T radius = 0;

    constexpr bool contains(Vector2f p) const {
        return (pos - p).length_squared() <= radius * radius;
    }
};
using IntCircle = Circle<i32>;
using FloatCircle = Circle<f32>;


struct Triangle {
    usize A;
    usize B;
    usize C;
    f32 weight = 1.f;

    constexpr Vector2f centroid(const Vector2f* vertices) const {
        const auto a = vertices[A];
        const auto b = vertices[B];
        const auto c = vertices[C];

        const auto m1 = a + (b - a) / 2.f;
        const auto m2 = b + (c - b) / 2.f;
        const auto v1 = c - m1;
        const auto v2 = a - m2;

        const auto den = v1.x * -v2.y - v1.y * -v2.x;
        // if (std::abs(den) < 0.00001f) { throw std::exception("bad triangle"); }

        const auto k = m2.x - m1.x;
        const auto l = m2.y - m1.y;
        const auto l1 = (v2.x * l - v2.y * k) / den;

        return m1 + v1 * l1;
    }

    constexpr Vector2f circumcenter(const Vector2f* vertices) const {
        const auto a = vertices[A];
        const auto b = vertices[B];
        const auto c = vertices[C];

        const auto m1 = a + (b - a) / 2.f;
        const auto m2 = b + (c - b) / 2.f;
        const auto v1 = (b - a).perp_ccw();
        const auto v2 = (c - b).perp_ccw();

        const auto den = v1.x * -v2.y - v1.y * -v2.x;
        // if (std::abs(den) < 0.00001f) { throw std::exception("bad triangle"); }

        const auto k = m2.x - m1.x;
        const auto l = m2.y - m1.y;
        const auto l1 = (v2.x * l - v2.y * k) / den;

        return m1 + v1 * l1;
    }


    constexpr static f32 sign(Vector2f a, Vector2f b, Vector2f c) {
        return (float)((a.x - c.x) * (b.y - c.y) - (b.x - c.x) * (a.y - c.y));
    }

    constexpr bool contains(const Vector2f* vertices, Vector2f p, bool with_corners = true) const {
        const auto a = vertices[A];
        const auto b = vertices[B];
        const auto c = vertices[C];

        if (!with_corners && (p == a || p == b || p == c)) { return false; }

        const auto d1 = sign(p, a, b);
        const auto d2 = sign(p, b, c);
        const auto d3 = sign(p, c, a);

        const auto has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
        const auto has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);

        return !(has_neg && has_pos);
    }

    constexpr bool contains_with_error(const Vector2f* vertices, Vector2f p, f32 error) const {
        const auto center = centroid(vertices);

        const auto a = center + (vertices[A] - center) * (1.f + error);
        const auto b = center + (vertices[B] - center) * (1.f + error);
        const auto c = center + (vertices[C] - center) * (1.f + error);

        const auto d1 = sign(p, a, b);
        const auto d2 = sign(p, b, c);
        const auto d3 = sign(p, c, a);

        const auto has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
        const auto has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);

        return !(has_neg && has_pos);
    }
};


using Path = std::vector<Vector2f>;
struct IndexedPoint {
    Vector2f point;
    size_t index;
};
using IndexedPath = std::vector<IndexedPoint>;

}

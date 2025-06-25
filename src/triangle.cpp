#include "types.h"


namespace nav {

constexpr Vector2f Triangle::centroid(const Vector2f* vertices) const {
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

constexpr Vector2f Triangle::circumcenter(const Vector2f* vertices) const {
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

constexpr bool Triangle::contains(const Vector2f* vertices, Vector2f p, bool with_corners) const {
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

constexpr bool Triangle::contains_with_error(const Vector2f* vertices, Vector2f p, f32 error) const {
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

}

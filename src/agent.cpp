#include "agent.h"


namespace nav {

Agent::Agent(const nav::Mesh* mesh) : p_mesh(mesh) {}


void Agent::set_speed(float speed) {
    m_speed = speed;
}

float Agent::get_speed() const {
    return m_speed;
}


bool Agent::set_position(const Vector2f pos) {
    if (!p_mesh->get_triangle(pos, 0.05f).has_value()) { return false; }
    m_position = pos;
    m_path.clear();
    m_path_index = 0;
    m_path_prog = 0;
    return true;
}

const Vector2f Agent::get_position() const {
    return m_position;
}


bool Agent::set_target_position(const Vector2f goal) {
    m_path = p_mesh->pathfind(m_position, goal);
    if (m_path.empty()) { return false; }
    m_path_index = 0;
    m_path_prog = 0;
    return true;
}

Vector2f Agent::get_target_position() const {
    if (m_path.empty()) {
        return m_position;
    } else {
        return m_path.back();
    }
}


static std::optional<float> ray_circle_intersect_nearest(nav::Vector2f p, nav::Vector2f d, nav::FloatCircle c) {
    const float x = c.pos.dot(c.pos) + p.dot(p) - 2 * c.pos.dot(p) - c.radius * c.radius;
    const float y = 2 * d.dot(c.pos - p);
    const float z = d.dot(d);
    const float disc = y * y - 4 * x * z;
    if (disc < 0) { return {}; }
    const float root = std::sqrt(disc);
    const float t1 = (y + root) / (2 * z);
    if (std::abs(disc) < 0.0001f) {
        return t1 < 0 ? std::optional<float>{} : t1;
    }
    const float t2 = (y - root) / (2 * z);
    if (t1 < 0 && t2 < 0) { return {}; }
    if (t1 < 0) { return t2; }
    if (t2 < 0) { return t1; }
    return t1 < t2 ? t1 : t2;
}

void Agent::trim_path_radial(float dist) {
    if (dist == 0.f || m_path.empty()) { return; }
    const auto last = m_path.back();

    for (size_t j = 0; j < m_path.size(); j++) {
        const auto i = m_path.size() - 1;
        const auto d1 = (m_path[i] - last).length_squared();

        if (d1 <= dist * dist) {
            const auto d2 = (m_path[i-1] - last).length_squared();
            if (d2 <= dist * dist) {
                m_path.pop_back();
            } else {
                const auto pos = m_path[i-1];
                const auto dir = (m_path[i] - m_path[i-1]).normalise();
                const auto circle = FloatCircle{ last, dist };
                m_path[i] = pos + dir * ray_circle_intersect_nearest(pos, dir, circle).value();
                return;
            }
        }
    }
}

void Agent::trim_path_walked(float dist) {
    if (dist == 0.f || m_path.empty()) { return; }
    auto total = 0.f;
    for (int i = (int)m_path.size() - 2; i >= 0; i--) {
        total += (m_path[i] - m_path[i+1]).length();
        if (total < dist) {
            m_path.pop_back();
        } else {
            break;
        }
    }
}

void Agent::clamp_path_radial(float dist) {
    if (m_path.empty()) { return; }

    printf("BROKEN; FIX OR DONT USE\n");
    throw std::exception();

    const auto first = m_path.front();

    for (size_t j = 0; j < m_path.size(); j++) {
        const auto i = m_path.size() - 1;
        const auto d1 = (m_path[i] - first).length_squared();

        if (d1 >= dist * dist) {
            const auto d2 = (m_path[i-1] - first).length_squared();
            if (d2 >= dist * dist) {
                m_path.pop_back();
            } else {
                const auto pos = m_path[i-1];
                const auto dir = (m_path[i] - m_path[i-1]).normalise();
                const auto circle = nav::FloatCircle{ first, dist };
                m_path[i] = pos + dir * ray_circle_intersect_nearest(pos, dir, circle).value();
                return;
            }
        }
    }
}

void Agent::clamp_path_walked(float dist) {
    if (m_path.empty()) { return; }
    auto total = 0.f;
    auto count = 1ULL;
    auto step = 0.f;
    for (size_t i = 0; i < m_path.size()-1 && total < dist; i++) {
        step = (m_path[i+1] - m_path[i]).length();
        total += step;
        count++;
    }
    if (total <= dist) { return; }
    if (count < 2) { m_path.clear(); return; }
    m_path.resize(count);
    const auto pos = m_path[m_path.size() - 2];
    const auto dir = (m_path[m_path.size() - 1] - pos).normalise();
    m_path.back() = pos + dir * (step - (total - dist));
}


const nav::Path& Agent::get_active_path() const {
    return m_path;
}

float Agent::get_active_path_length() const {
    float result = 0.f;
    for (size_t i = 0; i < m_path.size() - 1; i++) {
        result += (m_path[i] - m_path[i+1]).length();
    }
    return result;
}


size_t Agent::get_current_index() const {
    return m_path_index;
}

size_t Agent::get_inverse_index() const {
    return m_path.size() - 1 - m_path_index;
}


bool Agent::is_moving() const {
    return !(m_path.empty() || (m_path_index == m_path.size() - 1) || m_override_stop);
}

void Agent::pause() {
    m_override_stop = true;
}

void Agent::stop()  {
    m_override_stop = true; m_path.clear(); m_path_index = 0; m_path_prog = 0;
}

void Agent::start() {
    m_override_stop = false;
}


void Agent::update(float deltatime) {
    if (!is_moving()) { return; }

    if (m_path[m_path_index + 1] == m_position) {
        if (++m_path_index == m_path.size() - 1) { return; }
    }
    const auto diff = m_path[m_path_index + 1] - m_position;
    const auto dist = diff.length();
    const auto actual = m_speed * deltatime * 60.f;
    if (dist < actual) {
        m_path_index++;
        m_position = m_path[m_path_index];
        const auto dir = diff * (1.f / dist);
        m_position += dir * (actual - dist);
    } else {
        const auto dir = diff * (1.f / dist);
        m_position += dir * actual;
    }
}

}

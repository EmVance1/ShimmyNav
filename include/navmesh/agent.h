#pragma once
#include "mesh.h"


namespace nav {

class Agent {
public:
    inline static float max_penalty = 100000.f;

private:
    const nav::Mesh* p_mesh = nullptr;
    Vector2f m_position;
    float m_speed = 1.0f;

    nav::Path m_path;
    size_t m_path_index = 0;
    float m_path_prog = 0.f;

    bool m_override_stop = false;

private:
    Agent(const nav::Mesh* mesh);

public:
    void set_speed(float speed);
    float get_speed() const;

    bool set_position(Vector2f pos);
    const Vector2f get_position() const;

    bool set_target_position(Vector2f goal);
    Vector2f get_target_position() const;

    void trim_path_radial(float dist);
    void trim_path_walked(float dist);

    void clamp_path_radial(float max);
    void clamp_path_walked(float max);

    const nav::Path& get_active_path() const;
    float get_active_path_length() const;

    size_t get_current_index() const;
    size_t get_inverse_index() const;

    bool is_moving() const;
    void pause();
    void stop();
    void start();

    void update(float deltatime);
};

}


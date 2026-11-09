#include "chase.hpp"

using namespace cgp;

namespace {

float const INF = 1e30f;

void set_path(agent_structure& a, std::vector<vec2> const& raw, std::vector<vec2> const& smooth)
{
    a.path_raw = raw;
    a.path = smooth;
    a.path_index = 0;
    a.path_version++;
}

void clear_path(agent_structure& a)
{
    set_path(a, std::vector<vec2>(), std::vector<vec2>());
}

// Move the agent along its polyline and smoothly turn it toward the direction of motion
void agent_advance(agent_structure& a, float dt, float speed_scale)
{
    a.moving = false;
    // Destination reached or null speed
    if (a.path_index >= a.path.size() || speed_scale <= 0.0f)
        return;

    // Move forward towards the path
    float remaining = a.speed * speed_scale * dt;
    while (remaining > 0.0f && a.path_index < a.path.size()) {
        vec2 const d = a.path[a.path_index] - a.position;
        float const L = norm(d);

        // Last steps
        if (L <= remaining) {
            a.position = a.path[a.path_index];
            remaining -= L;
            a.path_index++;
        }
        // Move forward normally
        else {
            a.position = a.position + d * (remaining / L);
            remaining = 0.0f;
        }
    }

    a.moving = true;

    if (a.path_index >= a.path.size())
        return;

    // Aim at a point slightly ahead on the path (certainly more realistic)
    size_t const look = std::min(a.path_index + 2, a.path.size() - 1);
    vec2 const dir = a.path[look] - a.position;

    // Update direction angle
    if (norm(dir) > 1e-4f) {
        float const target = std::atan2(dir.x, dir.y);
        float const diff = std::remainder(target - a.angle, 2.0f * Pi);
        float const step = a.max_turn * dt;
        a.angle += std::clamp(diff, -step, step);
    }
}

void tom_replan(chase_structure& c, nav_grid_structure const& g)
{
    // Target Jerry if visible, otherwise target the exit of the hole where he disappeared
    vec2 target = c.jerry.position;

    if (!c.jerry.visible) {
        if (c.hole_current < 0 || c.hole_current >= int(g.hole_exit.size()))
            return;

        target = g.hole_exit[c.hole_current];
    }

    int goal = g.cell_of(target);

    if (!g.is_free(goal % g.nx, goal / g.nx, false))
        goal = nav_nearest_free_cell(g, target, false);

    int start = g.cell_of(c.tom.position);

    if (!g.is_free(start % g.nx, start / g.nx, false))
        start = nav_nearest_free_cell(g, c.tom.position, false);

    // No path possible
    if (goal < 0 || start < 0) {
        clear_path(c.tom);
        return;
    }

    // A* from Tom to the target
    nav_search_result const res = nav_search(g, start, goal, false);
    std::vector<vec2> raw = nav_extract_path(g, res, goal);

    // No path found
    if (raw.empty()) {
        clear_path(c.tom);
        return;
    }

    raw[0] = c.tom.position;
    set_path(c.tom, raw, nav_smooth_path(g, raw, false));
}

void jerry_replan(chase_structure& c, nav_grid_structure const& g)
{
    // TODO: implement
}

void update_jerry(chase_structure& c, nav_grid_structure const& g, float dt)
{
    // TODO: implement
}

void update_tom(chase_structure& c, nav_grid_structure const& g, float dt)
{
    c.replan_timer_tom -= dt;
    if (c.replan_timer_tom <= 0.0f) {
        c.replan_timer_tom = c.param.replan_period;
        tom_replan(c, g);
    }

    // Tom can never catch Jerry: he stops when too close
    float speed_scale = 1.0f;
    if (c.jerry.visible && norm(c.tom.position - c.jerry.position) < c.param.catch_gap)
        speed_scale = 0.0f;

    agent_advance(c.tom, dt, speed_scale);
}

void place_character(character_structure& character, agent_structure const& a, float floor_y)
{
    effect_place_root(character.animated_model.skeleton, vec3(a.position.x, floor_y, a.position.y), a.angle + a.angle_offset);
}

}

void chase_initialize(chase_structure& c, nav_grid_structure const& g)
{
    chase_parameters const param = c.param;
    c = chase_structure();
    c.param = param; // copy back params

    vec2 jerry_start, tom_start;
    // Should always happen as we have three mouse holes
    if (g.hole_exit.size() >= 2) {
        jerry_start = g.hole_exit[0];
        tom_start = g.hole_exit[1];
    }
    else {
        // Too lazy to code the else condition
        return;
    }

    c.jerry.position = jerry_start;
    c.tom.position = tom_start;
    c.tom.speed = param.tom_speed;
    c.jerry.speed = param.jerry_speed;
}

void chase_update(chase_structure& c, nav_grid_structure const& g, float dt)
{
    dt = std::min(dt, 0.05f);
    c.tom.speed = c.param.tom_speed;
    c.jerry.speed = c.param.jerry_speed;
    c.cooldown = std::max(0.0f, c.cooldown - dt);

    // update_jerry(c, g, dt);
    update_tom(c, g, dt);
}

void chase_apply(chase_structure& c, nav_grid_structure const& g, character_structure& tom, character_structure& jerry)
{
    place_character(tom, c.tom, g.floor_y);
    place_character(jerry, c.jerry, g.floor_y);

    // Freeze the run cycle when the agent does not move
    tom.timer.scale = c.tom.moving ? 1.0f : 0.0f;
    jerry.timer.scale = c.jerry.moving ? 1.0f : 0.0f;
}

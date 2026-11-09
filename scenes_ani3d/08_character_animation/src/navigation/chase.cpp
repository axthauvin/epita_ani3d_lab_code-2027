#include "chase.hpp"

using namespace cgp;

namespace {

float const INF = 1e30f;

void set_path(agent_structure& a, std::vector<vec2> const& raw, std::vector<vec2> const& smooth)
{
    a.path_raw = raw;
    a.path = smooth;
    a.path_index = 0;
    ++a.path_version;
}

void clear_path(agent_structure& a)
{
    set_path(a, std::vector<vec2>(), std::vector<vec2>());
}

// Move the agent along its polyline and smoothly turn it toward the direction of motion
void agent_advance(agent_structure& a, float dt, float speed_scale)
{
    // TODO: implement
}

void tom_replan(chase_structure& c, nav_grid_structure const& g)
{
    // TODO: implement
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
    // TODO: implement
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
    // TODO: implement

    // update_jerry(c, g, dt);
    // update_tom(c, g, dt);
}

void chase_apply(chase_structure& c, nav_grid_structure const& g, character_structure& tom, character_structure& jerry)
{
    place_character(tom, c.tom, g.floor_y);
    place_character(jerry, c.jerry, g.floor_y);

    // Freeze the run cycle when the agent does not move
    tom.timer.scale = c.tom.moving ? 1.0f : 0.0f;
    jerry.timer.scale = c.jerry.moving ? 1.0f : 0.0f;
}

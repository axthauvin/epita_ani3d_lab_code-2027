#pragma once

#include "cgp/cgp.hpp"

#include "../effects/effects.hpp"
#include "navigation.hpp"

struct agent_structure {
    cgp::vec2 position = {0.0f, 0.0f}; // (x, z)
    float angle = 0.0f;                // rotation around the vertical axis
    float speed = 1.5f;
    float max_turn = 6.0f;             // max angular speed (rad/s)
    float angle_offset = 0.0f;         // correction if the model does not face +z by default
};

struct chase_parameters {
    float tom_speed = 1.5f;
    float jerry_speed = 1.65f;
};

struct chase_structure {
    agent_structure tom;
    agent_structure jerry;
    chase_parameters param;
};

// Reset positions and state
void chase_initialize(chase_structure& chase, nav_grid_structure const& grid);

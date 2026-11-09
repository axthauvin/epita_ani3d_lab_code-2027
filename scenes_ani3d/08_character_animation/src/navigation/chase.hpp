#pragma once

#include "cgp/cgp.hpp"

#include "../effects/effects.hpp"
#include "navigation.hpp"

struct agent_structure {
    cgp::vec2 position = {0.0f, 0.0f};  // (x, z)
    float angle = 0.0f;                 // rotation around the vertical axis
    float speed = 1.5f;
    float max_turn = 6.0f;              // max angular speed (rad/s)
    float angle_offset = 0.0f;          // correction if the model does not face +z by default
    bool visible = true;
    bool moving = false;

    // Path finding
    std::vector<cgp::vec2> path;        // smoothed path followed
    std::vector<cgp::vec2> path_raw;    // raw A* cells
    size_t path_index = 0;
    int path_version = 0;               // incremented at each path change (debug refresh)
};

struct chase_parameters {
    float tom_speed = 4.0f;
    float jerry_speed = 4.15f;
    float catch_gap = 0.5f;             // Tom never gets closer than this to Jerry
    float replan_period = 0.3f;
    float safety_time = 0.3f;           // Jerry must reach a cell this much earlier than Tom
    float hidden_duration = 2.0f;       // time spent inside the hole
    float hole_cooldown = 3.0f;         // Jerry cannot re-enter the hole he just left
    float hole_bonus = 3.0f;
    float hole_bonus_danger = 6.0f;     // bonus when Tom is close
    float danger_distance = 4.0f;
    float far_distance = 12.0f;         // cap of Tom's distance in the goal score
    float goal_hysteresis = 1.0f;
};

enum class jerry_state { running, hidden };

struct chase_structure {
    agent_structure tom;
    agent_structure jerry;
    chase_parameters param;

    jerry_state state = jerry_state::running;
    float replan_timer_tom = 0.0f;
    float replan_timer_jerry = 0.15f;
    float hidden_timer = 0.0f;
    float cooldown = 0.0f;
    int hole_current = -1; // hole Jerry is currently inside
    int hole_last = -1;    // hole Jerry just left
    int jerry_goal = -1;
    cgp::vec2 jerry_goal_pos = {0.0f, 0.0f};
};

// Reset positions and state
void chase_initialize(chase_structure& chase, nav_grid_structure const& grid);

// Update the AI and move the agents
void chase_update(chase_structure& chase, nav_grid_structure const& grid, float dt);

// Write the agent positions into the skeletons
void chase_apply(chase_structure& chase, nav_grid_structure const& grid, character_structure& tom, character_structure& jerry);

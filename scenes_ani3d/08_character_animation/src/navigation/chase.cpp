#include "chase.hpp"

using namespace cgp;

namespace {

float constexpr INF = 1e30f;

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

void jerry_follow_path(chase_structure& c, nav_grid_structure const& g, nav_search_result const& from_jerry, int goal)
{
    std::vector<vec2> raw = nav_extract_path(g, from_jerry, goal);
    if (raw.empty()) {
        clear_path(c.jerry);
        c.jerry_goal = -1;
        return;
    }
    raw[0] = c.jerry.position;
    set_path(c.jerry, raw, nav_smooth_path(g, raw, true));
    c.jerry_goal = goal;
    c.jerry_goal_pos = g.cell_center(goal);
}

void jerry_replan(chase_structure& c, nav_grid_structure const& g)
{
    chase_parameters const& p = c.param;
    agent_structure& jerry = c.jerry;
    agent_structure const& tom = c.tom;

    int const N = g.nx * g.nz;
    int const tom_cell = g.cell_of(tom.position);
    int const jerry_cell = g.cell_of(jerry.position);
    float const vt = tom.speed;
    float const vj = jerry.speed;
    bool const locked = c.cooldown > 0.0f;

    if (c.jerry_goal >= 0 && norm(c.jerry_goal_pos - jerry.position) < 0.3f)
        c.jerry_goal = -1;

    // Time map of Tom (holes are forbidden)
    nav_search_result const from_tom = nav_search(g, tom_cell, -1, false);

    // Remember the area around Jerry as visited
    int const R = std::max(1, int(1.0f / g.cell_size));
    int const ji = jerry_cell % g.nx, jj = jerry_cell / g.nx;
    for (int dj = -R; dj <= R; dj++)
        for (int di = -R; di <= R; di++)
            if (g.inside(ji + di, jj + dj))
                c.last_visit[g.index(ji + di, jj + dj)] = c.time;

    auto is_locked_hole = [&](int cell) { return locked && g.hole_id[cell] == c.hole_last; };

    // A cell is safe if Jerry gets there clearly before Tom
    auto safe = [&](int cell, float d) {
        if (is_locked_hole(cell))
            return false;
        return d / vj + p.safety_time < from_tom.dist[cell] / vt;
    };

    // Goal score: far from Tom, not too far from Jerry, bonus for holes
    auto score = [&](nav_search_result const& r, int cell) {
        float s = p.far_weight * std::min(from_tom.dist[cell], p.far_distance) - 0.25f * r.dist[cell];
        // For exploration, favor places not visited for a long time
        s += p.explore_weight * std::min(c.time - c.last_visit[cell], p.explore_cap) / p.explore_cap;
        return s;
    };

    auto pick = [&](nav_search_result const& r, float& best) {
        int best_cell = -1;
        best = -INF;

        for (int k = 0; k < N; k++) {
            // Skip invalid cells
            if (k == jerry_cell || r.dist[k] >= 1e29f || is_locked_hole(k))
                continue;

            // Fix horrible edge case (Jerry's trajectory shakes)
            // The goal has to be far enough away
            if (g.hole_id[k] < 0 && r.dist[k] < 1.0f)
                continue;

            float const s = score(r, k);

            if (s > best) {
                best = s;
                best_cell = k;
            }
        }

        return best_cell;
    };

    // Dijkstra from Jerry restricted to safe cells: every reached cell has a safe path
    nav_search_result from_jerry = nav_search(g, jerry_cell, -1, true, safe);
    float best_score = -INF;
    int goal = pick(from_jerry, best_score);

    // Panic mode: no safe cell, relax the safety constraint
    if (goal < 0) {
        from_jerry = nav_search(g, jerry_cell, -1, true, [&](int cell, float) { return !is_locked_hole(cell); });
        goal = pick(from_jerry, best_score);
    }
    if (goal < 0) {
        clear_path(jerry);
        c.jerry_goal = -1;
        return;
    }

    jerry_follow_path(c, g, from_jerry, goal);
}

void jerry_go_to_nearest_hole(chase_structure& c, nav_grid_structure const& g)
{
    nav_search_result const from_jerry = nav_search(g, g.cell_of(c.jerry.position), -1, true);

    int goal = -1;

    for (int k = 0; k < g.nx * g.nz; k++) {
        if (g.hole_id[k] >= 0 && from_jerry.dist[k] < 1e29f) {
            if (goal == -1)
                goal = k;
            else if (from_jerry.dist[k] < from_jerry.dist[goal])
                goal = k;

        }
    }

    if (goal < 0) {
        clear_path(c.jerry);
        c.jerry_goal = -1;
        return;
    }

    jerry_follow_path(c, g, from_jerry, goal);
}

void update_jerry(chase_structure& c, nav_grid_structure const& g, float dt)
{
    chase_parameters const& p = c.param;
    agent_structure& jerry = c.jerry;

    if (c.state == jerry_state::hidden) {
        c.hidden_timer -= dt;
        c.hidden_elapsed += dt;
        bool const tom_done = (c.phase == tom_phase::looking && c.phase_timer >= p.looking_duration);
        if (c.hidden_timer > 0.0f || !(tom_done || c.hidden_elapsed > p.hidden_timeout))
            return;

        // Reappear from the hole farthest from Tom (other than the one he entered)
        int best = -1;
        float best_d = -1.0f;

        for (int h = 0; h < int(g.hole_exit.size()); ++h) {
            if (h == c.hole_current)
                continue;

            float const d = norm(g.hole_exit[h] - c.tom.position);
            if (d > best_d) {
                best_d = d;
                best = h;
            }
        }

        if (best < 0)
            best = c.hole_current;

        jerry.position = g.hole_exit[best];
        vec2 const dir = g.hole_exit[best] - g.hole_center[best];

        if (norm(dir) > 1e-4f)
            jerry.angle = std::atan2(dir.x, dir.y);

        jerry.visible = true;
        c.state = jerry_state::running;
        c.hole_last = best;
        c.hole_current = -1;
        c.cooldown = p.hole_cooldown;
        c.jerry_goal = -1;
        c.replan_timer_jerry = 0.0f;
        c.phase = tom_phase::chasing;
        c.phase_timer = 0.0f;
        c.replan_timer_tom = 0.0f;
        c.jerry_dancing = false;
        clear_path(jerry);
        return;
    }

    // Enter a hole if Jerry stands on a hole cell
    int const h = g.hole_id[g.cell_of(jerry.position)];
    if (h >= 0 && !(c.cooldown > 0.0f && h == c.hole_last)) {
        c.state = jerry_state::hidden;
        c.hidden_timer = p.hidden_duration;
        c.hole_current = h;
        jerry.visible = false;
        jerry.moving = false;
        c.jerry_goal = -1;
        c.hidden_elapsed = 0.0f;
        c.jerry_dancing = false;
        c.force_hole = false;
        clear_path(jerry);
        return;
    }

    // Jerry dances while Tom is far
    float const d_tom = norm(c.tom.position - jerry.position);

    if (c.force_hole)
        c.jerry_dancing = false;
    else if (!c.jerry_dancing && d_tom > p.dance_distance)
        c.jerry_dancing = true;
    else if (c.jerry_dancing && d_tom < 0.7f * p.dance_distance) // with hystereris
        c.jerry_dancing = false;

    jerry.anim = c.jerry_dancing ? p.anim_dance : p.anim_run;

    if (c.jerry_dancing) {
        jerry.moving = false;
        return;
    }

    c.replan_timer_jerry -= dt;
    if (c.replan_timer_jerry <= 0.0f) {
        c.replan_timer_jerry = p.replan_period;
        if (c.force_hole)
            jerry_go_to_nearest_hole(c, g);
        else
            jerry_replan(c, g);
    }
    agent_advance(jerry, dt, 1.0f);
}

void update_tom(chase_structure& c, nav_grid_structure const& g, float dt)
{
    chase_parameters const& p = c.param;
    agent_structure& tom = c.tom;
    tom.moving = false;

    // Scripted sequence after Jerry entered the hole: fall, stand up, look around
    if (c.phase == tom_phase::falling || c.phase == tom_phase::lying
        || c.phase == tom_phase::standing_up || c.phase == tom_phase::looking) {
        c.phase_timer += dt;

        if (c.phase == tom_phase::falling) {
            tom.anim = p.anim_fall;

            // Start the blend toward StandUp slightly before Death ends
            if (c.phase_timer >= p.fall_duration) {
                c.phase = tom_phase::lying;
                c.phase_timer = 0.0f;
            }
        }
        else if (c.phase == tom_phase::lying) {
            tom.anim = p.anim_fall; // keep the last pose of Death

            if (c.phase_timer >= p.lying_duration) {
                c.phase = tom_phase::standing_up;
                c.phase_timer = 0.0f;
            }
        }
        else if (c.phase == tom_phase::standing_up) {
            tom.anim = p.anim_standup;

            // Start the blend toward Looking slightly before StandUp ends
            if (c.phase_timer >= p.standup_duration - 1.2f * p.blend_time) {
                c.phase = tom_phase::looking;
                c.phase_timer = 0.0f;
            }
        }
        else
            tom.anim = p.anim_look;

        return;
    }

    // Jerry just disappeared: Tom runs to the hole entrance
    if (c.phase == tom_phase::chasing && !c.jerry.visible) {
        c.phase = tom_phase::to_hole;
        c.replan_timer_tom = 0.0f;
    }

    // Assign the animation
    tom.anim = p.anim_run;

    // Replan Tom path
    c.replan_timer_tom -= dt;
    if (c.replan_timer_tom <= 0.0f) {
        c.replan_timer_tom = p.replan_period;
        tom_replan(c, g);
    }

    // Move Tom
    float speed_scale = 1.0f;
    if (c.jerry.visible && norm(tom.position - c.jerry.position) < p.catch_gap)
        speed_scale = 0.0f;
    agent_advance(tom, dt, speed_scale);

    // Arrived at the hole: face the wall and fall
    if (c.phase == tom_phase::to_hole && c.hole_current >= 0 && tom.path_index >= tom.path.size()
        && norm(tom.position - g.hole_exit[c.hole_current]) < 0.3f) {
        vec2 const dir = g.hole_center[c.hole_current] - tom.position;

        if (norm(dir) > 1e-4f)
            tom.angle = std::atan2(dir.x, dir.y);

        c.phase = tom_phase::falling;
        c.phase_timer = 0.0f;
        tom.moving = false;
    }
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

    c.last_visit.assign(g.nx * g.nz, -c.param.explore_cap);

    c.tom.anim = param.anim_run;
    c.jerry.anim = param.anim_run;
}

void chase_update(chase_structure& c, nav_grid_structure const& g, float dt)
{
    dt = std::min(dt, 0.05f);
    c.tom.speed = c.param.tom_speed;
    c.jerry.speed = c.param.jerry_speed;
    c.cooldown = std::max(0.0f, c.cooldown - dt);
    c.time += dt;

    update_jerry(c, g, dt);
    update_tom(c, g, dt);
}

void chase_force_jerry_to_hole(chase_structure& c)
{
    if (c.state != jerry_state::running)
        return;

    c.force_hole = true;
    c.cooldown = 0.0f;
    c.replan_timer_jerry = 0.0f;
}

void chase_apply(chase_structure& c, nav_grid_structure const& g, character_structure& tom, character_structure& jerry)
{
    place_character(tom, c.tom, g.floor_y);
    place_character(jerry, c.jerry, g.floor_y);

    // Freeze the Death animation on its last pose while Tom lies on the ground
    // Maybe there is a better place to do this...
    if (c.phase == tom_phase::lying) {
        tom.timer.scale = 0.0f;
        tom.timer.t_periodic = std::max(0.0f, tom.timer.event_period - 0.02f);
    }
}

#include "chase_debug.hpp"

using namespace cgp;

namespace {

// Build one flat colored quad per cell satisfying the predicate
void build_cells(debug_mesh_structure& out, nav_grid_structure const& g, std::function<bool(int)> const& pred, vec3 const& color, float y)
{
    mesh m;
    float const s = 0.45f * g.cell_size;
    unsigned int k = 0;

    for (int c = 0; c < g.nx * g.nz; c++) {
        if (!pred(c))
            continue;

        vec2 const p = g.cell_center(c);
        m.position.push_back({p.x - s, y, p.y - s});
        m.position.push_back({p.x + s, y, p.y - s});
        m.position.push_back({p.x + s, y, p.y + s});
        m.position.push_back({p.x - s, y, p.y + s});

        for (int q = 0; q < 4; q++) {
            m.normal.push_back({0, 1, 0});
            m.color.push_back(color);
            m.uv.push_back({0, 0});
        }

        m.connectivity.push_back(uint3{k, k + 2, k + 1});
        m.connectivity.push_back(uint3{k, k + 3, k + 2});

        k += 4;
    }

    out.valid = k > 0;
    if (!out.valid)
        return;

    out.drawable.initialize_data_on_gpu(m);
    out.drawable.material.phong = {1, 0, 0, 1}; // ambient only: flat color
}

// Upload a 2D polyline as a curve at height y
void set_curve(debug_curve_structure& dc, std::vector<vec2> const& pts, float y, vec3 const& color)
{
    if (dc.valid)
        dc.curve.clear();

    dc.valid = false;

    if (pts.size() < 2)
        return;

    numarray<vec3> v;

    for (vec2 const& p : pts)
        v.push_back({p.x, y, p.y});

    dc.curve.initialize_data_on_gpu(v);
    dc.curve.color = color;
    dc.valid = true;
}

void refresh_curves(debug_curve_structure& smooth, debug_curve_structure& raw, agent_structure const& a,
                    float y, vec3 const& color_smooth, vec3 const& color_raw)
{
    if (smooth.version == a.path_version)
        return;

    smooth.version = a.path_version;
    set_curve(smooth, a.path, y + 0.04f, color_smooth);
    set_curve(raw, a.path_raw, y + 0.02f, color_raw);
}

} // namespace

void chase_debug_initialize(chase_debug_structure& d, nav_grid_structure const& g)
{
    float const y = g.floor_y + 0.02f;
    build_cells(d.grid_blocked, g, [&](int c) { return g.blocked[c] != 0; }, {1.0f, 0.2f, 0.2f}, y);
    build_cells(d.grid_hole, g, [&](int c) { return !g.blocked[c] && g.hole_id[c] >= 0; }, {0.2f, 0.4f, 1.0f}, y + 0.005f);
    build_cells(d.grid_free, g, [&](int c) { return !g.blocked[c] && g.hole_id[c] < 0; }, {0.3f, 0.9f, 0.3f}, y - 0.005f);

    d.marker.initialize_data_on_gpu(mesh_primitive_sphere());
    d.marker.material.phong = {1, 0, 0, 1};
}

void chase_debug_draw(chase_debug_structure& d, chase_structure const& c, nav_grid_structure const& g,
                      environment_generic_structure const& environment,
                      bool show_grid, bool show_free_cells, bool show_paths)
{
    if (show_grid) {
        if (d.grid_blocked.valid)
            draw(d.grid_blocked.drawable, environment);

        if (d.grid_hole.valid)
            draw(d.grid_hole.drawable, environment);

        if (show_free_cells && d.grid_free.valid)
            draw(d.grid_free.drawable, environment);
    }

    refresh_curves(d.tom_path, d.tom_raw, c.tom, g.floor_y, {1.0f, 0.0f, 0.0f}, {1.0f, 0.6f, 0.0f});
    refresh_curves(d.jerry_path, d.jerry_raw, c.jerry, g.floor_y, {0.0f, 0.7f, 0.0f}, {0.9f, 0.9f, 0.0f});

    if (d.tom_raw.valid)
        draw(d.tom_raw.curve, environment);
    if (d.jerry_raw.valid)
        draw(d.jerry_raw.curve, environment);
    if (d.tom_path.valid)
        draw(d.tom_path.curve, environment);
    if (d.jerry_path.valid)
        draw(d.jerry_path.curve, environment);

    // Markers: holes (blue), hole exits (cyan), Jerry goal (green)
    auto draw_marker = [&](vec2 const& p, float size, vec3 const& color) {
        d.marker.model.translation = {p.x, g.floor_y + size, p.y};
        d.marker.model.scaling = size;
        d.marker.material.color = color;
        draw(d.marker, environment);
    };

    for (size_t h = 0; h < g.hole_center.size(); h++) {
        draw_marker(g.hole_center[h], 0.05f, {0.2f, 0.4f, 1.0f});
        draw_marker(g.hole_exit[h], 0.03f, {0.0f, 0.9f, 0.9f});
    }

    if (c.jerry_goal >= 0)
        draw_marker(c.jerry_goal_pos, 0.04f, {0.0f, 1.0f, 0.0f});
}

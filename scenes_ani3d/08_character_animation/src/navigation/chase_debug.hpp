#pragma once

#include "cgp/cgp.hpp"

#include "chase.hpp"
#include "navigation.hpp"

struct debug_mesh_structure {
    cgp::mesh_drawable drawable;
    bool valid = false;
};

struct debug_curve_structure {
    cgp::curve_drawable curve;
    bool valid = false;
    int version = -1;
};

struct chase_debug_structure {
    // Occupation grid
    debug_mesh_structure grid_blocked;  // red
    debug_mesh_structure grid_hole;     // blue
    debug_mesh_structure grid_free;     // green
    cgp::mesh_drawable marker;

    // Paths
    debug_curve_structure tom_path;     // red: smoothed path of Tom
    debug_curve_structure jerry_path;   // green: smoothed path of Jerry
    debug_curve_structure tom_raw;      // orange: raw A* cells of Tom
    debug_curve_structure jerry_raw;    // yellow: raw cells of Jerry
};

void chase_debug_initialize(chase_debug_structure& debug, nav_grid_structure const& grid);

void chase_debug_draw(chase_debug_structure& debug, chase_structure const& chase, nav_grid_structure const& grid,
                      cgp::environment_generic_structure const& environment,
                      bool show_grid, bool show_free_cells, bool show_paths);

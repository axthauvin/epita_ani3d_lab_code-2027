#pragma once

#include "cgp/cgp.hpp"

#include "chase.hpp"
#include "navigation.hpp"

struct debug_mesh_structure {
    cgp::mesh_drawable drawable;
    bool valid = false;
};

struct chase_debug_structure {
    // Occupation grid
    debug_mesh_structure grid_blocked;  // red
    debug_mesh_structure grid_hole;     // blue
    debug_mesh_structure grid_free;     // green
};

void chase_debug_initialize(chase_debug_structure& debug, nav_grid_structure const& grid);

void chase_debug_draw(chase_debug_structure& debug, chase_structure const& chase, nav_grid_structure const& grid,
                      cgp::environment_generic_structure const& environment,
                      bool show_grid, bool show_free_cells, bool show_paths);

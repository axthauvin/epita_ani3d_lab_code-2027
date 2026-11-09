#pragma once

#include "cgp/cgp.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <utility>
#include <vector>

// Occupancy grid on the ground plane (x, z)
// So world point (x, y, z) is stored as vec2(x, z)
struct nav_grid_structure {
    float cell_size = 0.1f;
    cgp::vec2 origin = {0.0f, 0.0f};
    int nx = 0;
    int nz = 0;
    float floor_y = 0.0f;

    // TODO: convert to vector of bool?
    std::vector<char> blocked;              // 1 if an obstacle occupies the cell
    // TODO: convert to vector of char?
    std::vector<int> hole_id;               // -1 for a normal cell, else index of the mouse hole
    std::vector<cgp::vec2> hole_center;     // center of each hole
    std::vector<cgp::vec2> hole_exit;       // free normal cell connected to each hole

    int index(int i, int j) const { return j * nx + i; }
    bool inside(int i, int j) const { return i >= 0 && j >= 0 && i < nx && j < nz; }

    // Check if a cell is free
    bool is_free(int i, int j, bool allow_holes) const {
        if (!inside(i, j))
            return false;

        int const k = index(i, j);

        if (blocked[k])
            return false;

        return allow_holes || hole_id[k] < 0;
    }

    // Get the center position of a cell
    cgp::vec2 cell_center(int k) const {
        int const i = k % nx;
        int const j = k / nx;
        return origin + cell_size * cgp::vec2(i + 0.5f, j + 0.5f);
    }

    // Get cell index from a position
    int cell_of(cgp::vec2 const& p) const {
        int const i = std::clamp(int(std::floor((p.x - origin.x) / cell_size)), 0, nx - 1);
        int const j = std::clamp(int(std::floor((p.y - origin.y) / cell_size)), 0, nz - 1);
        return index(i, j);
    }
};

struct nav_build_parameters {
    float cell_size = 0.1f;
    float height_min = 0.2f;    // obstacles lower than floor_y + height_min are ignored
    float height_max = 2.0f;    // obstacles higher than floor_y + height_max are ignored
    // TODO: margin to adjust
    float margin = 0.1f;        // distance to obstacles (radius of the characters)
};

// Build the grid from obstacle meshes (already scaled to world coordinates) and the mouse-hole boxes
void nav_grid_build(nav_grid_structure& grid,
                    std::vector<cgp::mesh> const& obstacle_meshes,
                    std::vector<std::pair<cgp::vec3, cgp::vec3>> const& hole_boxes,
                    nav_build_parameters const& param);

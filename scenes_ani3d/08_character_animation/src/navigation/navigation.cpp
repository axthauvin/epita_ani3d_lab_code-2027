#include "navigation.hpp"

#include <queue>

using namespace cgp;

namespace {

float const INF = 1e30f;

float dist_point_segment(vec2 const& p, vec2 const& a, vec2 const& b)
{
    vec2 const ab = b - a;
    float const l2 = dot(ab, ab);
    float const t = l2 > 1e-12f ? std::clamp(dot(p - a, ab) / l2, 0.0f, 1.0f) : 0.0f;
    return norm(p - (a + t * ab));
}

bool point_in_triangle(vec2 const& p, vec2 const& a, vec2 const& b, vec2 const& c)
{
    auto cross = [](vec2 const& u, vec2 const& v) { return u.x * v.y - u.y * v.x; };
    float const d1 = cross(b - a, p - a);
    float const d2 = cross(c - b, p - b);
    float const d3 = cross(a - c, p - c);
    // All cross products must have the same sign, otherwise the point is outside
    bool const has_neg = d1 < 0 || d2 < 0 || d3 < 0;
    bool const has_pos = d1 > 0 || d2 > 0 || d3 > 0;
    return !(has_neg && has_pos);
}

} // namespace

void nav_grid_build(nav_grid_structure& g,
                    std::vector<mesh> const& meshes,
                    std::vector<std::pair<vec3, vec3>> const& hole_boxes,
                    nav_build_parameters const& param)
{
    // === Bounding box of the whole house =====================================
    vec3 pmin = {1e9f, 1e9f, 1e9f};
    vec3 pmax = {-1e9f, -1e9f, -1e9f};

    auto extend = [&](vec3 const& q) {
        pmin = {std::min(pmin.x, q.x), std::min(pmin.y, q.y), std::min(pmin.z, q.z)};
        pmax = {std::max(pmax.x, q.x), std::max(pmax.y, q.y), std::max(pmax.z, q.z)};
    };

    // House meshes
    for (auto const& m : meshes)
        for (int k = 0; k < int(m.position.size()); k++)
            extend(m.position[k]);

    // Mouse holes
    for (auto const& b : hole_boxes) {
        extend(b.first);
        extend(b.second);
    }

    // === Prepare navigation grid structure ===================================
    g.cell_size = param.cell_size;
    g.origin = {pmin.x, pmin.z};
    g.nx = int((pmax.x - pmin.x) / g.cell_size) + 1;
    g.nz = int((pmax.z - pmin.z) / g.cell_size) + 1;
    g.floor_y = pmin.y;
    int const N = g.nx * g.nz;
    g.blocked.assign(N, 0);
    g.hole_id.assign(N, -1);

    // Check for obstacle between these two bounds
    float const y_lo = g.floor_y + param.height_min;
    float const y_hi = g.floor_y + param.height_max;

    // === Build navigration grid ==============================================
    // Rasterize obstacle triangles: a cell is blocked if its center is inside the projected
    // triangle, or closer than margin to one of its edges
    for (auto const& m : meshes) {
        for (int k = 0; k < int(m.connectivity.size()); k++) {
            vec3 const a = m.position[m.connectivity[k][0]];
            vec3 const b = m.position[m.connectivity[k][1]];
            vec3 const c = m.position[m.connectivity[k][2]];

            // Check if it is an obstacle candidate
            if (std::max({a.y, b.y, c.y}) < y_lo || std::min({a.y, b.y, c.y}) > y_hi)
                continue;

            // Get triangle bounds
            vec2 const a2(a.x, a.z), b2(b.x, b.z), c2(c.x, c.z);
            float const x0 = std::min({a2.x, b2.x, c2.x}) - param.margin;
            float const x1 = std::max({a2.x, b2.x, c2.x}) + param.margin;
            float const z0 = std::min({a2.y, b2.y, c2.y}) - param.margin;
            float const z1 = std::max({a2.y, b2.y, c2.y}) + param.margin;
            int const i0 = std::max(0, int(std::floor((x0 - g.origin.x) / g.cell_size)));
            int const i1 = std::min(g.nx - 1, int(std::floor((x1 - g.origin.x) / g.cell_size)));
            int const j0 = std::max(0, int(std::floor((z0 - g.origin.y) / g.cell_size)));
            int const j1 = std::min(g.nz - 1, int(std::floor((z1 - g.origin.y) / g.cell_size)));

            for (int j = j0; j <= j1; j++) {
                for (int i = i0; i <= i1; i++) {
                    int const idx = g.index(i, j);

                    if (g.blocked[idx])
                        continue; // already blocked

                    vec2 const p = g.cell_center(idx);
                    float const d = std::min({dist_point_segment(p, a2, b2), dist_point_segment(p, b2, c2), dist_point_segment(p, c2, a2)});

                    // Block if too close (margin threshold)
                    //  We do not want Tom and Jerry to run into walls with their model halfway inside.
                    if (d <= param.margin || point_in_triangle(p, a2, b2, c2))
                        g.blocked[idx] = 1;
                }
            }
        }
    }

    // === Build holes grid ====================================================
    // Mouse holes: cells of the tunnel box become hole cells (free for Jerry only)
    int const H = int(hole_boxes.size());
    g.hole_center.assign(H, vec2(0, 0));
    g.hole_exit.assign(H, vec2(0, 0));

    for (int h = 0; h < H; h++) {
        vec3 const& bmin = hole_boxes[h].first;
        vec3 const& bmax = hole_boxes[h].second;
        float const e = g.cell_size;
        g.hole_center[h] = vec2(0.5f * (bmin.x + bmax.x), 0.5f * (bmin.z + bmax.z));

        for (int idx = 0; idx < N; idx++) {
            vec2 const p = g.cell_center(idx);

            if (p.x >= bmin.x - e && p.x <= bmax.x + e && p.y >= bmin.z - e && p.y <= bmax.z + e) {
                // Mark it as free in the blocked vector, but as occupied in the hole_id vector
                g.blocked[idx] = 0;
                g.hole_id[idx] = h;
            }
        }
    }
}

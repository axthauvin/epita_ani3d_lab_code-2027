#include "navigation.hpp"

#include <queue>

using namespace cgp;

namespace {

float constexpr INF = 1e30f;

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

bool point_free(nav_grid_structure const& g, vec2 const& p, bool allow_holes)
{
    int const i = int(std::floor((p.x - g.origin.x) / g.cell_size));
    int const j = int(std::floor((p.y - g.origin.y) / g.cell_size));
    return g.is_free(i, j, allow_holes);
}

vec2 catmull_rom(vec2 const& p0, vec2 const& p1, vec2 const& p2, vec2 const& p3, float t)
{
    float const t2 = t * t;
    float const t3 = t2 * t;
    return 0.5f * (
        (2.0f * p1) + (p2 - p0) * t
        + (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * t2
        + (3.0f * p1 - p0 - 3.0f * p2 + p3) * t3
    );
}

} // namespace

int nav_nearest_free_cell(nav_grid_structure const& g, vec2 const& p, bool allow_holes)
{
    int const c = g.cell_of(p);
    int const ci = c % g.nx;
    int const cj = c / g.nx;
    int const R = std::max(g.nx, g.nz);

    // Search ring by ring around the starting cell
    for (int r = 0; r <= R; r++) {
        int best = -1;
        float best_d = INF;

        auto check = [&](int i, int j) {
            if (!g.is_free(i, j, allow_holes))
                return;

            int const k = g.index(i, j);
            float const d = norm(g.cell_center(k) - p);

            if (d < best_d) {
                best_d = d;
                best = k;
            }
        };

        if (r == 0)
            check(ci, cj);
        else {
            for (int i = ci - r; i <= ci + r; i++) {
                check(i, cj - r);
                check(i, cj + r);
            }

            for (int j = cj - r + 1; j <= cj + r - 1; j++) {
                check(ci - r, j);
                check(ci + r, j);
            }
        }

        if (best >= 0)
            return best;
    }
    return -1;
}

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
    // triangle, or closer than margin to one of its edges.
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

    // === Mark extern free cells as blocked ===================================
    // Flood fill from corners.
    // The prevents hole exits to be behind the house walls.
    std::vector<int> stack;
    int const corners[4] = {
        g.index(0, 0),
        g.index(g.nx - 1, 0),
        g.index(0, g.nz - 1),
        g.index(g.nx - 1, g.nz - 1)};

    // Mark all corners as blocked
    for (int c : corners) {
        g.blocked[c] = 1;
        stack.push_back(c);
    }

    int const di[4] = {1, -1, 0, 0};
    int const dj[4] = {0, 0, 1, -1};

    // Flood fill
    while (!stack.empty()) {
        int const cur = stack.back();
        stack.pop_back();
        int const ci = cur % g.nx;
        int const cj = cur / g.nx;

        for (int d = 0; d < 4; d++) {
            int const ni = ci + di[d];
            int const nj = cj + dj[d];
            if (!g.inside(ni, nj))
                continue;

            int const n = g.index(ni, nj);
            if (g.blocked[n])
                continue; // already blocked, fill stops here

            g.blocked[n] = 1;
            stack.push_back(n); // recursion
        }
    }

    // === Build holes grid ====================================================
    // Mouse holes: cells of the tunnel box become hole cells (free for Jerry only).
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

        // Connect the hole to the map.
        // Carve a line towards the nearest free cell (which is not a holl).
        g.hole_exit[h] = g.hole_center[h];
        int const exit_cell = nav_nearest_free_cell(g, g.hole_center[h], false);

        if (exit_cell < 0)
            continue; // no exit found

        vec2 const start = g.hole_center[h];
        vec2 const target = g.cell_center(exit_cell);
        int const n = std::max(1, int(std::ceil(norm(target - start) / (0.5f * g.cell_size))));
        g.hole_exit[h] = target;

        for (int s = 0; s <= n; s++)
        {
            vec2 const p = start + (target - start) * (float(s) / float(n));
            int const k = g.cell_of(p);

            if (g.blocked[k]) {
                g.blocked[k] = 0;
                g.hole_id[k] = h;
            }
            else if (g.hole_id[k] < 0) {
                g.hole_exit[h] = g.cell_center(k);
                break;
            }
        }
    }
}

nav_search_result nav_search(nav_grid_structure const& g, int start, int goal, bool allow_holes,
                             std::function<bool(int, float)> const& accept)
{
    int const N = g.nx * g.nz;
    nav_search_result res;
    res.dist.assign(N, INF);
    res.parent.assign(N, -1);
    if (start < 0)
        return res;

    // Euclidean heuristic for A*, zero heuristic for Dijkstra
    auto heuristic = [&](int k) {
        if (goal < 0)
            return 0.0f; // Dijkstra
        return norm(g.cell_center(k) - g.cell_center(goal)); // A*
    };

    using node = std::pair<float, int>;
    std::priority_queue<node, std::vector<node>, std::greater<node>> open;
    res.dist[start] = 0.0f;
    open.push({heuristic(start), start});

    while (!open.empty()) {
        node const top = open.top();
        open.pop();
        int const cur = top.second;
        // Distance condition
        if (top.first > res.dist[cur] + heuristic(cur) + 1e-4f)
            continue;
        if (cur == goal)
            break;

        int const ci = cur % g.nx;
        int const cj = cur / g.nx;
        for (int di = -1; di <= 1; di++) {
            for (int dj = -1; dj <= 1; dj++) {
                if (di == 0 && dj == 0)
                    continue;

                int const ni = ci + di;
                int const nj = cj + dj;

                if (!g.is_free(ni, nj, allow_holes))
                    continue;

                // Do not cut corners of obstacles
                bool const diagonal = di != 0 && dj != 0;
                if (diagonal && (!g.is_free(ci + di, cj, allow_holes) || !g.is_free(ci, cj + dj, allow_holes)))
                    continue;

                int const n = g.index(ni, nj);
                float const c = res.dist[cur] + (diagonal ? 1.4142f : 1.0f) * g.cell_size; // cost

                // Cost does not improve the path
                if (c >= res.dist[n])
                    continue;

                // The destination cell is simply invalid
                // For example: Jerry cannot access a locked hole
                if (accept && !accept(n, c))
                    continue;

                // Update new best distance
                res.dist[n] = c;
                res.parent[n] = cur;
                open.push({c + heuristic(n), n});
            }
        }
    }

    return res;
}

std::vector<vec2> nav_extract_path(nav_grid_structure const& g, nav_search_result const& res, int goal)
{
    std::vector<vec2> path;

    if (goal < 0 || res.dist[goal] >= 1e29f)
        return path;

    for (int c = goal; c != -1; c = res.parent[c])
        path.push_back(g.cell_center(c));

    std::reverse(path.begin(), path.end());
    return path;
}

bool nav_line_of_sight(nav_grid_structure const& g, vec2 const& a, vec2 const& b, bool allow_holes)
{
    // Divides the segment into small steps of half a cell,
    // then checks for each point whether the cell containing it is free.
    int const n = std::max(1, int(std::ceil(norm(b - a) / (0.5f * g.cell_size))));

    for (int s = 0; s <= n; s++) {
        vec2 const p = a + (b - a) * (float(s) / float(n));
        if (!point_free(g, p, allow_holes))
            return false;
    }

    return true;
}

std::vector<vec2> nav_smooth_path(nav_grid_structure const& g, std::vector<vec2> const& points, bool allow_holes)
{
    if (points.size() < 3)
        return points;

    // === (1) String pulling ==================================================
    // Keep only the points needed to avoid obstacles
    // Documentation: https://idm-lab.org/bib/abstracts/papers/socs20c.pdf
    std::vector<vec2> pulled;
    pulled.push_back(points[0]);
    size_t i = 0;

    while (i < points.size() - 1) {
        size_t j = points.size() - 1;

        while (j > i + 1 && !nav_line_of_sight(g, points[i], points[j], allow_holes))
            j--;

        pulled.push_back(points[j]);
        i = j;
    }

    // === (2) Catmull-Rom =====================================================
    // Sampling of the pulled points
    // Documentation: https://en.wikipedia.org/wiki/Catmull%E2%80%93Rom_spline
    std::vector<vec2> out;
    out.push_back(pulled[0]);

    for (size_t s = 0; s + 1 < pulled.size(); s++) {
        vec2 const p1 = pulled[s];
        vec2 const p2 = pulled[s + 1];
        vec2 const p0 = s > 0 ? pulled[s - 1] : p1;
        vec2 const p3 = s + 2 < pulled.size() ? pulled[s + 2] : p2;

        int const n = std::max(1, int(norm(p2 - p1) / (1.5f * g.cell_size)));
        std::vector<vec2> segment;
        bool ok = true;

        for (int k = 1; k <= n; k++) {
            vec2 const q = catmull_rom(p0, p1, p2, p3, float(k) / float(n));

            if (!point_free(g, q, allow_holes))
                ok = false;

            segment.push_back(q);
        }

        // If the curve touches an obstacle, fall back to the straight segment
        if (!ok) {
            segment.clear();

            for (int k = 1; k <= n; k++)
                segment.push_back(p1 + (p2 - p1) * (float(k) / float(n)));
        }

        out.insert(out.end(), segment.begin(), segment.end());
    }
    return out;
}

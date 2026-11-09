#include "chase.hpp"

using namespace cgp;

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

    c.jerry.position = jerry_start;
    c.tom.position = tom_start;
    c.tom.speed = param.tom_speed;
    c.jerry.speed = param.jerry_speed;
}

#include "simulation.hpp"

using namespace cgp;

void divergence_free(grid_2D<vec2> &new_velocity, grid_2D<vec2> const &velocity,
                     grid_2D<float> &divergence,
                     grid_2D<float> &gradient_field) {
  // v = projection of v0 on divergence free vector field
  //
  // v : Final vector field to be filled
  // v0: Initial vector field (non divergence free)
  // divergence: temporary buffer used to compute the divergence of v0
  // gradient_field: temporary buffer used to compute v = v0 -
  // nabla(gradient_field) TO do:
  // 1. Compute divergence of v0
  // 2. Compute gradient_field such that nabla(gradient_field)^2 = div(v0)
  // 3. Compute v = v0 - nabla(gradient_field)

  for (int x = 1; x < velocity.dimension[0] - 1; x++) {
    for (int y = 1; y < velocity.dimension[1] - 1; y++) {
      divergence(x, y) = (velocity(x + 1, y).x - velocity(x - 1, y).x +
                          velocity(x, y + 1).y - velocity(x, y - 1).y) /
                         2;
    }
  }

  constexpr int NMAX = 10;

  gradient_field.fill(0);
  for (int x = 1; x < velocity.dimension[0] - 1; x++) {
    for (int y = 1; y < velocity.dimension[1] - 1; y++) {

      for (int i = 0; i < NMAX; i++) {
        gradient_field(x, y) =
            (gradient_field(x + 1, y) + gradient_field(x - 1, y) +
             gradient_field(x, y + 1) + gradient_field(x, y - 1) -
             divergence(x, y)) /
            4;
      }
    }
  }

  for (int x = 1; x < velocity.dimension[0] - 1; x++) {
    for (int y = 1; y < velocity.dimension[1] - 1; y++) {

      vec2 q(gradient_field(x + 1, y) - gradient_field(x - 1, y),
             gradient_field(x, y + 1) - gradient_field(x, y - 1));

      new_velocity(x, y) = velocity(x, y) - q / 2;
    }
  }

  set_boundary_reflective(new_velocity);
}
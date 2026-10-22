#include "simulation.hpp"

using namespace cgp;

// Convert a density value to a pressure
float density_to_pressure(float rho, float rho0, float stiffness) {
  return stiffness * (rho - rho0);
}

float W_laplacian_viscosity(vec3 const &p_i, vec3 const &p_j, float h) {
  // To do ...
  //  Fill it with laplacian of W_viscosity

  const float d = norm(p_i - p_j);
  return 45 / (Pi * h * h * h * h * h * h) * (h - d);
}

vec3 W_gradient_pressure(vec3 const &p_i, vec3 const &p_j, float h) {
  // To do ...
  //  Fill it with gradient of W_spiky

  const float d = norm(p_i - p_j);
  const float W_spiky = -45 / (Pi * h * h * h * h * h * h) * (h - d) * (h - d);

  return W_spiky * (p_i - p_j) / d;
}

float W_density(vec3 const &p_i, const vec3 &p_j, float h) {
  float const r = norm(p_i - p_j);
  assert_cgp_no_msg(r <= h);
  return 315.0 / (64.0 * 3.14159f * std::pow(h, 9)) *
         std::pow(h * h - r * r, 3.0f);
}

void update_density(numarray<particle_element> &particles, float h, float m) {
  // To do: Compute the density value (particles[i].rho) at each particle
  // position
  //  rho_i = \sum_j m W_density(pi,pj)
  //   int const N = particles.size();
  //   for (int i = 0; i < N; ++i)
  //     particles[i].rho = 1.0f; // to be modified

  int const N = particles.size();
  for (int i = 0; i < N; ++i) {
    particles[i].rho = 0;
    for (int j = 0; j < N; j++) {
      if (norm(particles[i].p - particles[j].p) <= h)
        particles[i].rho += m * W_density(particles[i].p, particles[j].p, h);
    }
  }
}

// Convert the particle density to pressure
void update_pressure(numarray<particle_element> &particles, float rho0,
                     float stiffness) {
  const int N = particles.size();
  for (int i = 0; i < N; ++i)
    particles[i].pressure =
        density_to_pressure(particles[i].rho, rho0, stiffness);
}

// Compute the forces and update the acceleration of the particles
void update_force(numarray<particle_element> &particles, float h, float m,
                  float nu) {
  // gravity
  const int N = particles.size();
  for (int i = 0; i < N; ++i) {
    particles[i].f = m * vec3{0, -9.81f, 0};
  }

  //   update_pressure(particles, float rho0, float stiffness)

  // TO Do
  //  For all particles i
  //    Compute F_pressure
  //    Compute F_viscosity
  //    particles[i].f += (F_pressure + F_viscosity)
  //  ...

  for (int i = 0; i < N; i++) {

    vec3 sum_pr(0, 0, 0);
    for (int j = 0; j < N; j++) {

      if (i == j)
        continue;

      const particle_element pi = particles.at(i);
      const particle_element pj = particles.at(j);

      const float d = norm(pi.p - pj.p);

      if (d >= h)
        continue;

      sum_pr += m * (pi.pressure + pj.pressure) / (2 * pj.rho) *
                W_gradient_pressure(pi.p, pj.p, h);
    }

    vec3 F_pressure = -m / particles.at(i).rho * sum_pr;

    vec3 sum_vis(0, 0, 0);
    for (int j = 0; j < N; j++) {
      if (i == j)
        continue;

      const particle_element pi = particles.at(i);
      const particle_element pj = particles.at(j);

      const float d = norm(pi.p - pj.p);

      if (d >= h)
        continue;

      sum_vis +=
          m * (pj.v - pi.v) / (pj.rho) * W_laplacian_viscosity(pi.p, pj.p, h);
    }

    vec3 F_viscosity = m * nu * sum_vis;

    particles[i].f += (F_pressure + F_viscosity);
  }
}

void simulate(float dt, numarray<particle_element> &particles,
              sph_parameters_structure const &sph_parameters) {

  // Update values
  update_density(particles, sph_parameters.h,
                 sph_parameters.m); // First compute updated density
  update_pressure(particles, sph_parameters.rho0,
                  sph_parameters.stiffness); // Compute associated pressure
  update_force(particles, sph_parameters.h, sph_parameters.m,
               sph_parameters.nu); // Update forces

  // Numerical integration
  float const damping = 0.005f;
  int const N = particles.size();
  float const m = sph_parameters.m;
  for (int k = 0; k < N; ++k) {
    vec3 &p = particles[k].p;
    vec3 &v = particles[k].v;
    vec3 &f = particles[k].f;

    v = (1 - damping) * v + dt * f / m;
    p = p + dt * v;
  }

  // Collision
  float const epsilon = 1e-3f;
  for (int k = 0; k < N; ++k) {
    vec3 &p = particles[k].p;
    vec3 &v = particles[k].v;

    // small perturbation to avoid alignment
    if (p.y < -1) {
      p.y = -1 + epsilon * rand_uniform();
      v.y *= -0.5f;
    }
    if (p.x < -1) {
      p.x = -1 + epsilon * rand_uniform();
      v.x *= -0.5f;
    }
    if (p.x > 1) {
      p.x = 1 - epsilon * rand_uniform();
      v.x *= -0.5f;
    }
  }
}
#pragma once

#include "cgp/cgp.hpp"

struct squash_structure {
    float amount = 0.0f;
    float flatness = 0.85f;
    float squash_speed = 15.0f;
    float reinflate_speed = 1.0f;
};

void squash_compute_offset(squash_structure const& squash, cgp::numarray<cgp::vec3> const& bind_position, cgp::numarray<cgp::vec3>& offset);

#include "character_loader.hpp"
#include "../environment.hpp"

using namespace cgp;

character_structure load_character_jerry() {
  filename_loader_structure loader_param;
  loader_param.set_skeleton(project::path + "assets/jerry/skeleton/");
  loader_param.add_rigged_mesh("body", project::path + "assets/jerry/mesh-jerry/", project::path + "assets/jerry/textures/texture.png");
  loader_param.add_animation("Idle", project::path + "assets/jerry/animation/idle/");
  loader_param.add_animation("Shuffling", project::path + "assets/jerry/animation/shuffling/");
  loader_param.add_animation("Walk", project::path + "assets/jerry/animation/walk/");

  character_structure character;
  character.load_and_initialize(loader_param, affine_rts().set_scaling(0.06f));

  return character;
}

character_structure load_character_tom() {
  filename_loader_structure loader_param;
  loader_param.set_skeleton(project::path + "assets/tom/skeleton/");
  loader_param.add_rigged_mesh("body", project::path + "assets/tom/mesh-tom/", project::path + "assets/tom/textures/texture.png");
  loader_param.add_animation("Death", project::path + "assets/tom/animation/death/");
  loader_param.add_animation("Idle", project::path + "assets/tom/animation/idle/");
  loader_param.add_animation("Looking", project::path + "assets/tom/animation/looking/");
  loader_param.add_animation("StandUp", project::path + "assets/tom/animation/stand-up/");
  loader_param.add_animation("Walk", project::path + "assets/tom/animation/walk/");

  character_structure character;
  character.load_and_initialize(loader_param, affine_rts().set_scaling(0.1f));

  return character;
}

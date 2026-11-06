#include "character_loader.hpp"
#include "../environment.hpp"

using namespace cgp;

character_structure load_character_lola() {
  filename_loader_structure loader_param;
  loader_param.set_skeleton(project::path + "assets/lola/skeleton/");
  loader_param.add_rigged_mesh("body", project::path + "assets/lola/mesh-lola/",
                               project::path +
                                   "assets/lola/mesh-lola/texture.jpg");
  loader_param.add_animation("Dance1",
                             project::path + "assets/lola/animation/dance_1/");
  loader_param.add_animation("Dance2",
                             project::path + "assets/lola/animation/dance_2/");
  loader_param.add_animation("Dance3",
                             project::path + "assets/lola/animation/dance_3/");
  loader_param.add_animation("Dance4",
                             project::path + "assets/lola/animation/dance_4/");
  loader_param.add_animation("Idle",
                             project::path + "assets/lola/animation/idle/");
  loader_param.add_animation("Walk",
                             project::path + "assets/lola/animation/walk/");
  loader_param.add_animation("Walk2", project::path +
                                          "assets/lola/animation/walk_style/");

  character_structure character;
  character.load_and_initialize(loader_param, affine_rts().set_scaling(0.01f));

  return character;
}

character_structure load_character_maria_sword() {
  filename_loader_structure loader_param;
  loader_param.set_skeleton(project::path + "assets/maria-sword/");
  loader_param.add_rigged_mesh(
      "Maria", project::path + "assets/maria-sword/mesh-maria/",
      project::path + "assets/maria-sword/mesh-maria/texture.png");
  loader_param.add_rigged_mesh(
      "Sword", project::path + "assets/maria-sword/mesh-maria_sword/",
      project::path + "assets/maria-sword/mesh-maria_sword/texture.png");
  loader_param.add_animation("Idle", project::path +
                                         "assets/maria-sword/animation/idle/");
  loader_param.add_animation(
      "Idle2", project::path + "assets/maria-sword/animation/idle2/");
  loader_param.add_animation(
      "Walk", project::path + "assets/maria-sword/animation/walking/");
  loader_param.add_animation(
      "Slash", project::path + "assets/maria-sword/animation/slash/");

  character_structure character;
  character.load_and_initialize(loader_param, affine_rts().set_scaling(0.01f));

  return character;
}

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

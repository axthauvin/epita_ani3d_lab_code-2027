#include "scene.hpp"

#include <map>
#include <regex>

#include "character_loader/character_loader.hpp"

using namespace cgp;

float constexpr house_scale = 0.2f;
int constexpr mixamo_head_index = 10;

void initialize_house(std::vector<mesh_drawable> &house, std::vector<mesh_drawable> &house_roof,
                      std::vector<vec3> &holes,
                      std::vector<std::pair<vec3, vec3>> &hole_boxes,
                      std::vector<mesh> &collision_meshes);

void scene_structure::initialize() {
// Give access to the inputs and window global state to the camera controler
  camera_control.initialize(inputs, window);
  camera_control.set_rotation_axis_y();
  camera_control.look_at({4.0f, 3.0f, 3.0f}, {0, 0, 0}, {0, 0, 1});
  global_frame.initialize_data_on_gpu(mesh_primitive_frame());

  sphere_ik.initialize_data_on_gpu(mesh_primitive_sphere());
  sphere_ik.model.scaling = 0.05f;
  sphere_ik.material.color = {1, 0, 0};

  std::cout << "- Load house" << std::endl;
  initialize_house(house, house_roof, holes, hole_boxes, house_collision_meshes);
  nav_build_parameters nav_param;
  nav_grid_build(nav_grid, house_collision_meshes, hole_boxes, nav_param);
  chase_debug_initialize(chase_debug, nav_grid);

  std::cout << "- Load Jerry character" << std::endl;
  characters["Jerry"] = load_character_jerry();
  characters["Jerry"].animated_model.apply_transformation({-3.5f, 0, 0});

  std::cout << "- Load Tom character" << std::endl;
  characters["Tom"] = load_character_tom();
  characters["Tom"].animated_model.apply_transformation({1.5f, 0, 0});

  for (std::string name : {"Tom", "Jerry"}) {
    // Set walk animation otherwise the first animation in alphabetical order is used
    characters[name].set_current_animation("Walk");
    effect_transition[name].transition_time = 0.3f;
  }

  chase.param.fall_duration = characters["Tom"].animated_model.animation.at("Death").time_max;
  chase.param.standup_duration = characters["Tom"].animated_model.animation.at("StandUp").time_max;

  chase_initialize(chase, nav_grid);

  current_active_character = "Tom";

  for (auto &entry : characters)
    entry.second.timer.start();
}

void scene_structure::display_frame() {
  // Update the camera view matrix from the camera controller
  environment.camera_view = camera_control.camera_model.matrix_view();

  // Set the light to the current position of the camera
  environment.light = camera_control.camera_model.position();

  if (gui.display_frame)
    draw(global_frame, environment);

  if (gui.display_house) {
    for (auto &mesh : house)
      draw(mesh, environment);
    if (gui.display_roof)
      for (auto &mesh : house_roof)
        draw(mesh, environment);
  }

  // Update the local time for each character
  for (auto &entry : characters) {
    entry.second.timer.update();
  }

  // ************************************************* //
  // Update the current skeleton of each character
  // ************************************************* //
  for (auto &entry : characters) {
    std::string character_name = entry.first;
    character_structure &character = entry.second;
    effect_transition_structure &transition = effect_transition[character_name];

    // Default animation reading a standard animation cycle
    if (transition.active == false) {
      character.animated_model.set_skeleton_from_animation(
          character.current_animation_name, character.timer.t_periodic);
    }
    // Currently with an active transition between two animations
    else {
      effect_transition_compute(transition, character);
      effect_transition_stop_if_completed(transition, character);
    }
  }

  // Chasing
  if (gui.chase_active) {
    chase_update(chase, nav_grid, inputs.time_interval);

    // Start a smooth transition when the requested animation changes
    auto request_animation = [&](std::string const &name, std::string const &anim) {
      character_structure &ch = characters[name];

      // Wait for the end of the running transition
      if (effect_transition[name].active)
        return;

      if (ch.current_animation_name == anim || ch.animated_model.animation.count(anim) == 0)
        return;

      effect_transition_start(effect_transition[name], ch, anim);
    };

    request_animation("Tom", chase.tom.anim);
    request_animation("Jerry", chase.jerry.anim);

    chase_apply(chase, nav_grid, characters["Tom"], characters["Jerry"]);
  }

  // ********************************** //
  // Apply effects on the skeleton
  // ********************************** //

  // Apply the walk effect if activated
  if (effect_walk.active) {
    effect_walking(effect_walk, characters[current_active_character], inputs,
                   effect_transition[current_active_character]);
  }

  // Apply the Inverse Kinematics effect if activated
  if (effect_ik.active) {
    effect_ik_compute(
        effect_ik,
        characters[current_active_character].animated_model.skeleton);
    sphere_ik.model.translation =
        effect_ik.target_position + effect_ik.target_offset;
    draw(sphere_ik, environment); // end effector

    sphere_ik.model.translation =
        characters[current_active_character]
            .animated_model.skeleton
            .joint_matrix_global[effect_ik.joint_root_ik]
            .get_block_translation(); // root for the IK
    draw(sphere_ik, environment);     // start joint
  }

  // Apply the head rotation effect if activated
  if (gui.rotate_head_effect_active) {
    for (auto &entry_character : characters) {
      effect_rotate_head_toward_objective_position(
          entry_character.second.animated_model.skeleton, mixamo_head_index ,
          camera_control.camera_model.position());
    }
  }
  // Tom looks at Jerry
  // else if (gui.chase_active && chase.jerry.visible) // when Jerry is visible or not?
  // If not, Tom looks at the hole jerry just entered.
  else if (gui.chase_active) {
    skeleton_structure &tom_skeleton = characters["Tom"].animated_model.skeleton;
    vec3 const jerry_position(chase.jerry.position.x, nav_grid.floor_y + 0.1f, chase.jerry.position.y);
    effect_rotate_head_toward_objective_position(tom_skeleton, mixamo_head_index, jerry_position);
  }

  // ********************************** //
  // Compute Skinning deformation
  // ********************************** //
  for (auto &entry_character : characters) {
    animated_model_structure &animated_model =
        entry_character.second.animated_model;
    for (auto &rigged_mesh_entry : animated_model.rigged_mesh) {
      std::string mesh_name = rigged_mesh_entry.first;
      animated_model.skinning_lbs(mesh_name);
    }
  }

  // ************************************** //
  // Display the surface and the skeletons
  // ************************************** //
  for (auto &entry_character : characters) {
    character_structure &character = entry_character.second;
    if (gui.chase_active && entry_character.first == "Jerry" && !chase.jerry.visible)
      continue;
    animated_model_structure &animated_model =
        entry_character.second.animated_model;

    // Display meshes
    for (auto &rigged_mesh_entry : animated_model.rigged_mesh) {
      std::string mesh_name = rigged_mesh_entry.first;
      rigged_mesh_structure &rigged_mesh = rigged_mesh_entry.second;

      mesh_drawable &drawable = character.drawable[mesh_name];
      drawable.vbo_position.update(rigged_mesh.mesh_deformed.position);
      drawable.vbo_normal.update(rigged_mesh.mesh_deformed.normal);

      if (gui.display_surface) {
        drawable.material.texture_settings.active = gui.display_texture;
        draw(drawable, environment);
      }
      if (gui.display_wireframe) {
        draw_wireframe(drawable, environment);
      }
    }

    // Display skeleton
    if (gui.display_skeleton) {
      character.sk_drawable.update(animated_model.skeleton);
      character.sk_drawable.display_joint_frame =
          gui.display_skeleton_joint_frame;
      character.sk_drawable.display_joint_sphere =
          gui.display_skeleton_joint_sphere;
      character.sk_drawable.display_segments = gui.display_skeleton_bone;
      draw(character.sk_drawable, environment);
    }
  }

  chase_debug_draw(chase_debug, chase, nav_grid, environment, gui.display_nav_grid,
                   gui.display_nav_free, gui.display_paths && gui.chase_active);
}

void scene_structure::display_gui() {
  // General display functions
  ImGui::Checkbox("Global Frame", &gui.display_frame);
  ImGui::SameLine();
  ImGui::Checkbox("House", &gui.display_house);
  ImGui::SameLine();
  ImGui::Checkbox("Roof", &gui.display_roof);
  ImGui::Checkbox("Wireframe", &gui.display_wireframe);
  ImGui::Checkbox("Display surface", &gui.display_surface);
  ImGui::SameLine();
  ImGui::Checkbox("Texture", &gui.display_texture);
  ImGui::Checkbox("Skeleton", &gui.display_skeleton);

  ImGui::Indent();
  ImGui::Checkbox("Bone", &gui.display_skeleton_bone);
  ImGui::SameLine();
  ImGui::Checkbox("Joint", &gui.display_skeleton_joint_sphere);
  ImGui::SameLine();
  ImGui::Checkbox("Frame", &gui.display_skeleton_joint_frame);
  ImGui::Unindent();

  ImGui::Spacing();
  ImGui::Spacing();

  // Effects
  ImGui::Spacing();
  ImGui::Separator();
  ImGui::Text("Effects: ");
  ImGui::Indent();
  bool is_walk_clicked = ImGui::Checkbox("Walk", &effect_walk.active);
  ImGui::SameLine();
  ImGui::Checkbox("Head", &gui.rotate_head_effect_active);
  ImGui::SameLine();
  bool is_ik_clicked = ImGui::Checkbox("IK", &effect_ik.active);
  ImGui::Unindent();

  // Chase
  ImGui::Separator();
  ImGui::Text("Chase: ");
  ImGui::Indent();
  if (ImGui::Checkbox("Run chase", &gui.chase_active) && gui.chase_active)
    chase_initialize(chase, nav_grid);
  ImGui::Checkbox("Grid", &gui.display_nav_grid);
  ImGui::SameLine();
  ImGui::Checkbox("Free cells", &gui.display_nav_free);
  ImGui::SameLine();
  ImGui::Checkbox("Paths", &gui.display_paths);
  ImGui::SliderFloat("Tom speed", &chase.param.tom_speed, 0.2f, 5.0f);
  ImGui::SliderFloat("Jerry speed", &chase.param.jerry_speed, 0.2f, 5.0f);
  ImGui::SliderFloat("Catch gap", &chase.param.catch_gap, 0.1f, 2.0f);
  ImGui::Unindent();

  // Handle start of walk in setting the current animation to Idle
  if (is_walk_clicked && effect_walk.active) {
    effect_transition[current_active_character].transition_time = chase.param.blend_time;
    characters[current_active_character].set_current_animation("Idle");
    effect_walk.root_position = vec3(0, 0, 0);
  }

  // Handle start of IK
  if (is_ik_clicked && effect_ik.active) {
    effect_ik_start(
        effect_ik, characters[current_active_character].animated_model.skeleton,
        effect_ik.joint_target);
  }

  // Specific parameters for the IK
  if (ImGui::CollapsingHeader("IK Parameters")) {
    ImGui::Text("Target Offset");
    ImGui::Indent();
    ImGui::SliderFloat("X", &effect_ik.target_offset[0], -2, 2);
    ImGui::SliderFloat("Y", &effect_ik.target_offset[1], -2, 2);
    ImGui::SliderFloat("Z", &effect_ik.target_offset[2], -2, 2);
    ImGui::Unindent();

    ImGui::Text("IK Joints: ");
    ImGui::Indent();

    bool changed_ik_joint = false;
    changed_ik_joint |= ImGui::InputInt("target", &effect_ik.joint_target);
    std::string joint_end_name =
        characters[current_active_character]
            .animated_model.skeleton.joint_name[effect_ik.joint_target];
    ImGui::Text(joint_end_name.c_str(), "%s");

    changed_ik_joint |= ImGui::InputInt("root", &effect_ik.joint_root_ik);
    std::string joint_start_name =
        characters[current_active_character]
            .animated_model.skeleton.joint_name[effect_ik.joint_root_ik];
    ImGui::Text(joint_start_name.c_str(), "%s");

    ImGui::Unindent();

    if (changed_ik_joint) {
      effect_ik_start(
          effect_ik,
          characters[current_active_character].animated_model.skeleton,
          effect_ik.joint_target);
    }
  }

  ImGui::Spacing();
  ImGui::Spacing();

  ImGui::Text("Current character: ");
  ImGui::SameLine();
  ImGui::Text(current_active_character.c_str(), "%s");

  // Display info for all characters
  for (auto &entry : characters) {

    ImGui::Spacing();
    ImGui::Spacing();
    ImGui::Separator();

    std::string name = entry.first;
    auto &character = entry.second;

    // Name of the character - special color for the active one
    ImVec4 current_color = name == current_active_character
                               ? ImVec4(1.0f, 0.0f, 0.0f, 0.5f)
                               : ImVec4(0.5f, 0.5f, 1.0f, 0.3f);
    ImGui::PushStyleColor(ImGuiCol_Button, current_color);
    if (ImGui::Button(name.c_str())) {
      current_active_character = name;
      // Update the values
      if (effect_ik.active) {
        effect_ik_start(
            effect_ik,
            characters[current_active_character].animated_model.skeleton,
            effect_ik.joint_target);
      }
      if (effect_walk.active) {
        effect_transition[current_active_character].transition_time = 0.2f;
        characters[current_active_character].set_current_animation("Idle");
        effect_walk.root_position = vec3(0, 0, 0);
        effect_walk.root_angle = 0.0f;
      }
    }
    ImGui::PopStyleColor();

    // Timers associated to the character
    ImGui::Indent();
    std::string transition_time_txt = "Transition time##" + name;
    ImGui::SliderFloat(transition_time_txt.c_str(),
                       &effect_transition[name].transition_time, 0.1f, 5.0f);
    std::string local_time_txt = "Anim cycle time##" + name;
    ImGui::SliderFloat(local_time_txt.c_str(), &character.timer.t_periodic,
                       0.0f, character.timer.event_period);

    // List all possible animations
    for (auto &entry_anim : character.animated_model.animation) {
      std::string animation_name = entry_anim.first;
      std::string animation_name_button_txt = animation_name + "##" + name;

      bool is_active_animation =
          animation_name == character.current_animation_name;
      ImVec4 current_color = is_active_animation
                                 ? ImVec4(1.0f, 0.0f, 0.0f, 0.5f)
                                 : ImVec4(0.5f, 0.5f, 1.0f, 0.3f);
      ImGui::PushStyleColor(ImGuiCol_Button, current_color);
      bool click_anim = ImGui::Button(animation_name_button_txt.c_str());
      ImGui::SameLine();
      ImGui::PopStyleColor();

      // If we click on an animation: start a transition toward the new
      // animation
      if (click_anim) {
        if (effect_transition[name].active) {
          std::cout << "Character " << name
                    << " did not finish his previous transition "
                    << animation_name << std::endl;
        }
        effect_transition_start(effect_transition[name], character,
                                animation_name);
      }
    }
    ImGui::NewLine();
    ImGui::Unindent();
  }
}

void scene_structure::mouse_move_event() {
  if (!inputs.keyboard.shift)
    camera_control.action_mouse_move();
}
void scene_structure::mouse_click_event() {
  camera_control.action_mouse_click();
}
void scene_structure::keyboard_event() {
  camera_control.action_keyboard();

  if (effect_walk.active) {
    effect_walking_keyboard_event(effect_transition[current_active_character],
                                  characters[current_active_character], inputs,
                                  effect_walk);
  }
}
void scene_structure::idle_frame() { camera_control.idle_frame(); }

void initialize_house(std::vector<mesh_drawable> &house, std::vector<mesh_drawable> &house_roof,
                      std::vector<vec3> &holes,
                      std::vector<std::pair<vec3, vec3>> &hole_boxes,
                      std::vector<mesh> &collision_meshes) {
  auto elements = mesh_load_file_obj_advanced(project::path + "assets/house/", "house.obj");

  // Several elements can share the same name: accumulate one box per hole id
  std::regex const hole_regex("TrouSouris_(\\d+)_Tunnel");
  std::map<int, std::pair<vec3, vec3>> boxes;

  holes.clear();
  hole_boxes.clear();
  collision_meshes.clear();

  for (auto const &elm : elements) {
    std::smatch match;
    // Check if the mesh represents a mouse hole (from its name)
    if (std::regex_search(elm.name, match, hole_regex)) {
      int const id = std::stoi(match[1].str());
      for (int k = 0; k < int(elm.mesh_element.position.size()); k++) {
        vec3 const q = house_scale * elm.mesh_element.position[k];
        if (boxes.find(id) == boxes.end())
          boxes[id] = {q, q};
        auto &b = boxes[id];
        // Update bounding boxe size
        b.first = {std::min(b.first.x, q.x), std::min(b.first.y, q.y), std::min(b.first.z, q.z)};
        b.second = {std::max(b.second.x, q.x), std::max(b.second.y, q.y), std::max(b.second.z, q.z)};
      }
    } else {
      // Every other element is an obstacle candidate, scaled to world coordinates
      mesh m = elm.mesh_element;
      for (int k = 0; k < int(m.position.size()); k++)
        m.position[k] *= house_scale;
      collision_meshes.push_back(m);
    }
  }

  // Record every nouse holes
  for (auto const &entry : boxes) {
    hole_boxes.push_back(entry.second);
    // Get the middle of the bounding boxe
    holes.push_back(0.5f * (entry.second.first + entry.second.second));
  }

  std::vector<mesh_drawable> drawables = mesh_obj_advanced_loader::convert_to_mesh_drawable(elements);

  house.clear();
  house_roof.clear();
  for (int k = 0; k < int(drawables.size()); k++) {
    drawables[k].model.scaling = house_scale;
    bool const is_roof = elements[k].name.find("Ceiling") != std::string::npos;
    if (is_roof)
      house_roof.push_back(drawables[k]);
    else
      house.push_back(drawables[k]);
  }
}

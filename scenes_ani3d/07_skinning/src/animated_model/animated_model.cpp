#include "animated_model.hpp"
using namespace cgp;
void animated_model_structure::skinning_lbs() {
  // ************************************************************** //
  // TO DO: Compute the Linear Blend Skinning (LBS) deformation
  // ...
  // ************************************************************** //
  //
  // Help:
  //     - The function should update the values of
  //     rigged_mesh.mesh_deformed.position based on the skeleton and bind pose
  //     (rigged_mesh.mesh_bind_pose.position)
  //     - Once the computation is working on the position, you may also update
  //     the normals
  //     - The skinning weights are available via: rigged_mesh.skinning_weight
  //       They are stored per vertex and per joint: float weight_ij =
  //       rigged_mesh.skinning_weight[vertex_i][joint_j];
  //
  //     - The matrices representing the joint frames can be accessed via:
  //         - skeleton.joint_matrix_global[joint_index] : the global
  //         transformation of the joint
  //         - skeleton.joint_matrix_global_bind_pose[joint_index] : the global
  //         transformation of the joint in the bind pose
  //
  //
  //     - Given a mat4 M representing a rigid transformation (with rotation and
  //     translation only), you can compute efficiently its inverse using the
  //     syntax
  //        mat4 M_inversed = M.inverse_assuming_rigid_transform();
  //
  //     - Consider a mat4 M representing a projective (or affine, or rigid)
  //     transformation, and a vec3 p. We call q the transformation of p by M.
  //         - If p represents a 3D point, then q can be expressed as
  //             vec3 q = vec3( M * vec4(p,1.0f) ); or similarily vec3 q =
  //             M.transform_position(p);
  //         - If p represents a 3D vector, then q can be expressed as
  //             vec3 q = vec3( M * vec4(p,0.0f) ); or similarily vec3 q =
  //             M.transform_vector(p);
  //

  // Example of looping over the positions of the mesh
  int N_vertex = rigged_mesh.mesh_bind_pose.position.size();
  for (int i = 0; i < N_vertex; ++i) {
    vec3 const &position_in_bind_pose =
        rigged_mesh.mesh_bind_pose
            .position[i]; // The "initial/bind pose" position p0
    vec3 const &normal_in_bind_pose =
        rigged_mesh.mesh_bind_pose
            .normal[i]; // The "initial/bind pose" normal n0
    vec3 &position_to_be_deformed =
        rigged_mesh.mesh_deformed
            .position[i]; // The position to be deformed by LBS
    vec3 &normal_to_be_deformed =
        rigged_mesh.mesh_deformed.normal[i]; // The normal to be deformed by LBS

    vec3 pi(0, 0, 0);
    vec3 ni(0, 0, 0);
    for (int j = 0; j < rigged_mesh.skinning_weight.at(i).size(); j++) {
      float weight_ij = rigged_mesh.skinning_weight[i][j];

      mat4 Tj = skeleton.joint_matrix_global[j] *
                skeleton.joint_matrix_global_bind_pose[j]
                    .inverse_assuming_rigid_transform();

      pi += weight_ij * vec3(Tj.transform_position(position_in_bind_pose));
      ni += weight_ij * vec3(Tj.transform_vector(normal_in_bind_pose));
    }

    position_to_be_deformed = pi;
    normal_to_be_deformed = ni;
  }
}
void animated_model_structure::skinning_dqs() {
  // ************************************************************** //
  // TO DO: Compute Dual Quaternion Skinning (DQS) deformation
  // ...
  // ************************************************************** //
  //
  // Help:
  //     - Given a mat4 representing a rigid transformation, the following
  //     syntax allows to access the rotation and translation part:
  //         affine_rt a = affine_rt::from_matrix({mat4});
  //         rotation_transform rot = a.rotation
  //         vec3 translation = a.translation
  //     - The quaternion of a rotation_transform can be accessed via
  //     {rotation_transform}.get_quaternion();
  //     - The structure quaternion is a specialized type derived from a vec4.
  //     You can access to its .x .y .z .w component similarily to a vec4.
  //
}

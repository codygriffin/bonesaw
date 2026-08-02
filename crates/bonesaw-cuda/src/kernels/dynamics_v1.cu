extern "C" __device__ __forceinline__ void compose_fixed_pose(
    const float left[12], const float right[12], float output[12]) {
  #pragma unroll
  for (unsigned row = 0; row < 3; ++row) {
    #pragma unroll
    for (unsigned column = 0; column < 3; ++column) {
      float value = left[row * 3] * right[column];
      value = fmaf(left[row * 3 + 1], right[3 + column], value);
      value = fmaf(left[row * 3 + 2], right[6 + column], value);
      output[row * 3 + column] = value;
    }
    float translation = left[row * 3] * right[9];
    translation = fmaf(left[row * 3 + 1], right[10], translation);
    translation = fmaf(left[row * 3 + 2], right[11], translation);
    output[9 + row] = translation + left[9 + row];
  }
}

extern "C" __device__ __forceinline__ void load_pose(
    const float* body_pose, unsigned body, unsigned agent,
    unsigned agent_stride, float output[12]) {
  #pragma unroll
  for (unsigned component = 0; component < 12; ++component)
    output[component] = body_pose[(body * 12 + component) * agent_stride + agent];
}

extern "C" __device__ __forceinline__ void transform_vector(
    const float pose[12], const float vector[3], float output[3]) {
  #pragma unroll
  for (unsigned row = 0; row < 3; ++row) {
    float value = pose[row * 3] * vector[0];
    value = fmaf(pose[row * 3 + 1], vector[1], value);
    value = fmaf(pose[row * 3 + 2], vector[2], value);
    output[row] = value;
  }
}

extern "C" __device__ __forceinline__ void cross_product(
    const float left[3], const float right[3], float output[3]) {
  output[0] = fmaf(left[1], right[2], -(left[2] * right[1]));
  output[1] = fmaf(left[2], right[0], -(left[0] * right[2]));
  output[2] = fmaf(left[0], right[1], -(left[1] * right[0]));
}

extern "C" __device__ __forceinline__ float dot_product(
    const float left[3], const float right[3]) {
  return fmaf(left[2], right[2],
              fmaf(left[1], right[1], left[0] * right[0]));
}

extern "C" __device__ __forceinline__ void matrix_vector(
    const float matrix[9], const float vector[3], float output[3]) {
  #pragma unroll
  for (unsigned row = 0; row < 3; ++row)
    output[row] = fmaf(matrix[row * 3 + 2], vector[2],
                       fmaf(matrix[row * 3 + 1], vector[1],
                            matrix[row * 3] * vector[0]));
}

extern "C" __device__ __forceinline__ void inertia_in_world(
    const float pose[12], const float body_inertia[9], float output[9]) {
  float intermediate[9];
  #pragma unroll
  for (unsigned row = 0; row < 3; ++row) {
    #pragma unroll
    for (unsigned column = 0; column < 3; ++column)
      intermediate[row * 3 + column] =
          fmaf(pose[row * 3 + 2], body_inertia[6 + column],
               fmaf(pose[row * 3 + 1], body_inertia[3 + column],
                    pose[row * 3] * body_inertia[column]));
  }
  #pragma unroll
  for (unsigned row = 0; row < 3; ++row) {
    #pragma unroll
    for (unsigned column = 0; column < 3; ++column)
      output[row * 3 + column] =
          fmaf(intermediate[row * 3 + 2], pose[column * 3 + 2],
               fmaf(intermediate[row * 3 + 1], pose[column * 3 + 1],
                    intermediate[row * 3] * pose[column * 3]));
  }
}

extern "C" __device__ __forceinline__ unsigned body_vector_index(
    unsigned body, unsigned component, unsigned agent_stride, unsigned agent) {
  return (body * 3 + component) * agent_stride + agent;
}

extern "C" __device__ __forceinline__ unsigned frame_index(
    unsigned body, unsigned component, unsigned coordinate,
    unsigned generalized_coordinate_count, unsigned agent_stride,
    unsigned agent) {
  return ((body * 6 + component) * generalized_coordinate_count + coordinate)
      * agent_stride + agent;
}

extern "C" __device__ __forceinline__ unsigned generalized_index(
    unsigned coordinate, unsigned agent_stride, unsigned agent) {
  return coordinate * agent_stride + agent;
}

extern "C" __device__ __forceinline__ unsigned mass_index(
    unsigned row, unsigned column, unsigned generalized_coordinate_count,
    unsigned agent_stride, unsigned agent) {
  return (row * generalized_coordinate_count + column) * agent_stride + agent;
}

extern "C" __device__ __forceinline__ unsigned centroidal_index(
    unsigned component, unsigned coordinate,
    unsigned generalized_coordinate_count, unsigned agent_stride,
    unsigned agent) {
  return (component * generalized_coordinate_count + coordinate)
      * agent_stride + agent;
}

extern "C" __device__ __forceinline__ void load_body_vector(
    const float* storage, unsigned body, unsigned agent,
    unsigned agent_stride, float output[3]) {
  #pragma unroll
  for (unsigned component = 0; component < 3; ++component)
    output[component] = storage[body_vector_index(
        body, component, agent_stride, agent)];
}

extern "C" __device__ __forceinline__ void store_body_vector(
    float* storage, unsigned body, unsigned agent, unsigned agent_stride,
    const float value[3]) {
  #pragma unroll
  for (unsigned component = 0; component < 3; ++component)
    storage[body_vector_index(body, component, agent_stride, agent)] =
        value[component];
}

extern "C" __device__ __forceinline__ void load_frame_column(
    const float* frame_jacobian, unsigned body, unsigned coordinate,
    unsigned generalized_coordinate_count, unsigned agent_stride,
    unsigned agent, float angular[3], float linear_origin[3]) {
  #pragma unroll
  for (unsigned component = 0; component < 3; ++component) {
    angular[component] = frame_jacobian[frame_index(
        body, component, coordinate, generalized_coordinate_count,
        agent_stride, agent)];
    linear_origin[component] = frame_jacobian[frame_index(
        body, 3 + component, coordinate, generalized_coordinate_count,
        agent_stride, agent)];
  }
}

extern "C" __global__ void bonesaw_dynamics_v1(
    const float* generalized_velocity,
    const float* gravity_world,
    const float* body_pose,
    const float* system_center_of_mass,
    const unsigned char* input_status,
    unsigned active_agents,
    unsigned agent_stride,
    unsigned root_body,
    unsigned joint_count,
    unsigned body_count,
    unsigned generalized_coordinate_count,
    const unsigned* joint_parent,
    const unsigned* joint_child,
    const unsigned* joint_coordinate,
    const unsigned* joint_kind,
    const float* joint_parent_from_joint,
    const float* joint_axis,
    const float* body_mass,
    const float* body_com,
    const float* body_inertia,
    const float* frame_jacobian,
    float* angular_velocity,
    float* angular_acceleration,
    float* linear_velocity_origin,
    float* linear_acceleration_origin,
    float* mass_matrix,
    float* bias_force,
    float* centroidal_map,
    unsigned char* output_status) {
  const unsigned agent = blockIdx.x * blockDim.x + threadIdx.x;
  if (agent >= agent_stride) return;

  for (unsigned row = 0; row < generalized_coordinate_count; ++row) {
    bias_force[generalized_index(row, agent_stride, agent)] = 0.0f;
    for (unsigned column = 0; column < generalized_coordinate_count; ++column)
      mass_matrix[mass_index(row, column, generalized_coordinate_count,
                             agent_stride, agent)] = 0.0f;
  }
  for (unsigned component = 0; component < 6; ++component)
    for (unsigned coordinate = 0; coordinate < generalized_coordinate_count;
         ++coordinate)
      centroidal_map[centroidal_index(component, coordinate,
          generalized_coordinate_count, agent_stride, agent)] = 0.0f;
  for (unsigned body = 0; body < body_count; ++body) {
    #pragma unroll
    for (unsigned component = 0; component < 3; ++component) {
      const unsigned index = body_vector_index(body, component,
                                               agent_stride, agent);
      angular_velocity[index] = 0.0f;
      angular_acceleration[index] = 0.0f;
      linear_velocity_origin[index] = 0.0f;
      linear_acceleration_origin[index] = 0.0f;
    }
  }

  output_status[agent] = input_status[agent];
  if (input_status[agent] != 1) return;
  if (agent >= active_agents) {
    output_status[agent] = 0;
    return;
  }
  for (unsigned coordinate = 0; coordinate < generalized_coordinate_count;
       ++coordinate) {
    const float value = generalized_velocity[
        generalized_index(coordinate, agent_stride, agent)];
    if (!isfinite(value)) {
      output_status[agent] = 2;
      return;
    }
  }
  #pragma unroll
  for (unsigned component = 0; component < 3; ++component) {
    const float value = gravity_world[component * agent_stride + agent];
    if (!isfinite(value)) {
      output_status[agent] = 2;
      return;
    }
  }

  float root_angular_velocity[3], root_linear_velocity[3];
  #pragma unroll
  for (unsigned component = 0; component < 3; ++component) {
    root_angular_velocity[component] = generalized_velocity[
        generalized_index(component, agent_stride, agent)];
    root_linear_velocity[component] = generalized_velocity[
        generalized_index(3 + component, agent_stride, agent)];
  }
  store_body_vector(angular_velocity, root_body, agent, agent_stride,
                    root_angular_velocity);
  store_body_vector(linear_velocity_origin, root_body, agent, agent_stride,
                    root_linear_velocity);

  for (unsigned joint = 0; joint < joint_count; ++joint) {
    const unsigned parent = joint_parent[joint];
    const unsigned child = joint_child[joint];
    float angular_parent[3], alpha_parent[3], velocity_parent[3],
          acceleration_parent[3];
    load_body_vector(angular_velocity, parent, agent, agent_stride,
                     angular_parent);
    load_body_vector(angular_acceleration, parent, agent, agent_stride,
                     alpha_parent);
    load_body_vector(linear_velocity_origin, parent, agent, agent_stride,
                     velocity_parent);
    load_body_vector(linear_acceleration_origin, parent, agent, agent_stride,
                     acceleration_parent);

    float parent_pose[12], fixed_pose[12], joint_pose[12];
    load_pose(body_pose, parent, agent, agent_stride, parent_pose);
    #pragma unroll
    for (unsigned component = 0; component < 12; ++component)
      fixed_pose[component] = joint_parent_from_joint[joint * 12 + component];
    compose_fixed_pose(parent_pose, fixed_pose, joint_pose);
    float parent_to_joint[3] = {
        joint_pose[9] - parent_pose[9],
        joint_pose[10] - parent_pose[10],
        joint_pose[11] - parent_pose[11]};
    float angular_cross_offset[3], alpha_cross_offset[3],
          centripetal[3];
    cross_product(angular_parent, parent_to_joint, angular_cross_offset);
    cross_product(alpha_parent, parent_to_joint, alpha_cross_offset);
    cross_product(angular_parent, angular_cross_offset, centripetal);
    float velocity_joint[3], acceleration_joint[3];
    #pragma unroll
    for (unsigned component = 0; component < 3; ++component) {
      velocity_joint[component] = velocity_parent[component]
          + angular_cross_offset[component];
      acceleration_joint[component] = acceleration_parent[component]
          + alpha_cross_offset[component] + centripetal[component];
    }

    float local_axis[3] = {joint_axis[joint * 3],
                           joint_axis[joint * 3 + 1],
                           joint_axis[joint * 3 + 2]};
    float axis_world[3];
    transform_vector(joint_pose, local_axis, axis_world);
    const unsigned coordinate = joint_coordinate[joint];
    const float joint_velocity = coordinate == 0xffffffffu ? 0.0f
        : generalized_velocity[generalized_index(
              6 + coordinate, agent_stride, agent)];
    float axis_velocity[3];
    #pragma unroll
    for (unsigned component = 0; component < 3; ++component)
      axis_velocity[component] = axis_world[component] * joint_velocity;

    if (joint_kind[joint] == 1) {
      float coriolis_angular[3], child_angular[3], child_alpha[3];
      cross_product(angular_parent, axis_velocity, coriolis_angular);
      #pragma unroll
      for (unsigned component = 0; component < 3; ++component) {
        child_angular[component] = angular_parent[component]
            + axis_velocity[component];
        child_alpha[component] = alpha_parent[component]
            + coriolis_angular[component];
      }
      store_body_vector(angular_velocity, child, agent, agent_stride,
                        child_angular);
      store_body_vector(angular_acceleration, child, agent, agent_stride,
                        child_alpha);
      store_body_vector(linear_velocity_origin, child, agent, agent_stride,
                        velocity_joint);
      store_body_vector(linear_acceleration_origin, child, agent, agent_stride,
                        acceleration_joint);
    } else if (joint_kind[joint] == 2) {
      float child_pose[12];
      load_pose(body_pose, child, agent, agent_stride, child_pose);
      float joint_to_child[3] = {child_pose[9] - joint_pose[9],
                                 child_pose[10] - joint_pose[10],
                                 child_pose[11] - joint_pose[11]};
      float angular_offset[3], alpha_offset[3], angular_angular_offset[3],
          prismatic_coriolis[3];
      cross_product(angular_parent, joint_to_child, angular_offset);
      cross_product(alpha_parent, joint_to_child, alpha_offset);
      cross_product(angular_parent, angular_offset, angular_angular_offset);
      cross_product(angular_parent, axis_velocity, prismatic_coriolis);
      float child_velocity[3], child_acceleration[3];
      #pragma unroll
      for (unsigned component = 0; component < 3; ++component) {
        child_velocity[component] = velocity_joint[component]
            + angular_offset[component] + axis_velocity[component];
        child_acceleration[component] = acceleration_joint[component]
            + alpha_offset[component] + angular_angular_offset[component]
            + 2.0f * prismatic_coriolis[component];
      }
      store_body_vector(angular_velocity, child, agent, agent_stride,
                        angular_parent);
      store_body_vector(angular_acceleration, child, agent, agent_stride,
                        alpha_parent);
      store_body_vector(linear_velocity_origin, child, agent, agent_stride,
                        child_velocity);
      store_body_vector(linear_acceleration_origin, child, agent, agent_stride,
                        child_acceleration);
    } else {
      store_body_vector(angular_velocity, child, agent, agent_stride,
                        angular_parent);
      store_body_vector(angular_acceleration, child, agent, agent_stride,
                        alpha_parent);
      store_body_vector(linear_velocity_origin, child, agent, agent_stride,
                        velocity_joint);
      store_body_vector(linear_acceleration_origin, child, agent, agent_stride,
                        acceleration_joint);
    }
  }

  float center_of_mass[3], gravity[3];
  #pragma unroll
  for (unsigned component = 0; component < 3; ++component) {
    center_of_mass[component] =
        system_center_of_mass[component * agent_stride + agent];
    gravity[component] = gravity_world[component * agent_stride + agent];
  }

  for (unsigned body = 0; body < body_count; ++body) {
    const float mass = body_mass[body];
    if (mass <= 0.0f) continue;
    float pose[12];
    load_pose(body_pose, body, agent, agent_stride, pose);
    float local_com[3] = {body_com[body * 3], body_com[body * 3 + 1],
                          body_com[body * 3 + 2]};
    float com_offset[3];
    transform_vector(pose, local_com, com_offset);
    float com_world[3] = {pose[9] + com_offset[0],
                          pose[10] + com_offset[1],
                          pose[11] + com_offset[2]};
    float local_inertia[9], inertia_world[9];
    #pragma unroll
    for (unsigned index = 0; index < 9; ++index)
      local_inertia[index] = body_inertia[body * 9 + index];
    inertia_in_world(pose, local_inertia, inertia_world);

    for (unsigned coordinate = 0;
         coordinate < generalized_coordinate_count; ++coordinate) {
      float angular[3], linear_origin[3], angular_cross_com[3], linear[3];
      load_frame_column(frame_jacobian, body, coordinate,
          generalized_coordinate_count, agent_stride, agent,
          angular, linear_origin);
      cross_product(angular, com_offset, angular_cross_com);
      float linear_momentum[3];
      #pragma unroll
      for (unsigned component = 0; component < 3; ++component) {
        linear[component] = linear_origin[component]
            + angular_cross_com[component];
        linear_momentum[component] = linear[component] * mass;
      }
      float inertia_angular[3], com_lever[3], orbital[3],
          angular_momentum[3];
      matrix_vector(inertia_world, angular, inertia_angular);
      #pragma unroll
      for (unsigned component = 0; component < 3; ++component)
        com_lever[component] = com_world[component] - center_of_mass[component];
      cross_product(com_lever, linear_momentum, orbital);
      #pragma unroll
      for (unsigned component = 0; component < 3; ++component) {
        angular_momentum[component] = inertia_angular[component]
            + orbital[component];
        const unsigned angular_index = centroidal_index(component, coordinate,
            generalized_coordinate_count, agent_stride, agent);
        const unsigned linear_index = centroidal_index(3 + component, coordinate,
            generalized_coordinate_count, agent_stride, agent);
        centroidal_map[angular_index] += angular_momentum[component];
        centroidal_map[linear_index] += linear_momentum[component];
      }

      for (unsigned column = 0; column <= coordinate; ++column) {
        float angular_column[3], linear_origin_column[3],
            angular_column_cross_com[3], linear_column[3],
            inertia_angular_column[3];
        load_frame_column(frame_jacobian, body, column,
            generalized_coordinate_count, agent_stride, agent,
            angular_column, linear_origin_column);
        cross_product(angular_column, com_offset, angular_column_cross_com);
        #pragma unroll
        for (unsigned component = 0; component < 3; ++component)
          linear_column[component] = linear_origin_column[component]
              + angular_column_cross_com[component];
        matrix_vector(inertia_world, angular_column, inertia_angular_column);
        const float value = fmaf(mass, dot_product(linear, linear_column),
                                 dot_product(angular,
                                             inertia_angular_column));
        mass_matrix[mass_index(coordinate, column,
            generalized_coordinate_count, agent_stride, agent)] += value;
        if (coordinate != column)
          mass_matrix[mass_index(column, coordinate,
              generalized_coordinate_count, agent_stride, agent)] += value;
      }
    }

    float omega[3], alpha[3], origin_acceleration[3];
    load_body_vector(angular_velocity, body, agent, agent_stride, omega);
    load_body_vector(angular_acceleration, body, agent, agent_stride, alpha);
    load_body_vector(linear_acceleration_origin, body, agent, agent_stride,
                     origin_acceleration);
    float alpha_cross_com[3], omega_cross_com[3], centripetal_com[3];
    cross_product(alpha, com_offset, alpha_cross_com);
    cross_product(omega, com_offset, omega_cross_com);
    cross_product(omega, omega_cross_com, centripetal_com);
    float force[3];
    #pragma unroll
    for (unsigned component = 0; component < 3; ++component) {
      const float com_acceleration = origin_acceleration[component]
          + alpha_cross_com[component] + centripetal_com[component];
      force[component] = (com_acceleration - gravity[component]) * mass;
    }
    float inertia_omega[3], inertia_alpha[3], gyroscopic[3], moment[3];
    matrix_vector(inertia_world, omega, inertia_omega);
    matrix_vector(inertia_world, alpha, inertia_alpha);
    cross_product(omega, inertia_omega, gyroscopic);
    #pragma unroll
    for (unsigned component = 0; component < 3; ++component)
      moment[component] = inertia_alpha[component] + gyroscopic[component];

    for (unsigned coordinate = 0;
         coordinate < generalized_coordinate_count; ++coordinate) {
      float angular[3], linear_origin[3], angular_cross_com[3], linear[3];
      load_frame_column(frame_jacobian, body, coordinate,
          generalized_coordinate_count, agent_stride, agent,
          angular, linear_origin);
      cross_product(angular, com_offset, angular_cross_com);
      #pragma unroll
      for (unsigned component = 0; component < 3; ++component)
        linear[component] = linear_origin[component]
            + angular_cross_com[component];
      const unsigned index = generalized_index(coordinate,
                                                agent_stride, agent);
      #pragma unroll
      for (unsigned component = 0; component < 3; ++component)
        bias_force[index] = fmaf(linear[component], force[component],
                                 bias_force[index]);
      #pragma unroll
      for (unsigned component = 0; component < 3; ++component)
        bias_force[index] = fmaf(angular[component], moment[component],
                                 bias_force[index]);
    }
  }
}

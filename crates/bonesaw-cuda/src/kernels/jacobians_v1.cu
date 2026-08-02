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

extern "C" __device__ __forceinline__ void transform_point(
    const float pose[12], const float point[3], float output[3]) {
  transform_vector(pose, point, output);
  #pragma unroll
  for (unsigned component = 0; component < 3; ++component)
    output[component] += pose[9 + component];
}

extern "C" __device__ __forceinline__ void cross_product(
    const float left[3], const float right[3], float output[3]) {
  output[0] = fmaf(left[1], right[2], -(left[2] * right[1]));
  output[1] = fmaf(left[2], right[0], -(left[0] * right[2]));
  output[2] = fmaf(left[0], right[1], -(left[1] * right[0]));
}

extern "C" __device__ __forceinline__ void load_pose(
    const float* body_pose, unsigned body, unsigned agent,
    unsigned agent_stride, float output[12]) {
  #pragma unroll
  for (unsigned component = 0; component < 12; ++component)
    output[component] = body_pose[(body * 12 + component) * agent_stride + agent];
}

extern "C" __device__ __forceinline__ unsigned frame_index(
    unsigned body, unsigned component, unsigned generalized_coordinate,
    unsigned generalized_coordinate_count, unsigned agent_stride,
    unsigned agent) {
  return ((body * 6 + component) * generalized_coordinate_count
          + generalized_coordinate) * agent_stride + agent;
}

extern "C" __device__ __forceinline__ unsigned com_index(
    unsigned component, unsigned generalized_coordinate,
    unsigned generalized_coordinate_count, unsigned agent_stride,
    unsigned agent) {
  return (component * generalized_coordinate_count + generalized_coordinate)
      * agent_stride + agent;
}

extern "C" __global__ void bonesaw_jacobians_v1(
    const float* body_pose,
    const float* total_mass,
    const unsigned char* input_status,
    unsigned agent_stride,
    unsigned root_body,
    unsigned body_count,
    unsigned generalized_coordinate_count,
    const unsigned* joint_parent,
    const unsigned* joint_coordinate,
    const unsigned* joint_kind,
    const float* joint_parent_from_joint,
    const float* joint_axis,
    const unsigned* body_parent_joint,
    const float* body_mass,
    const float* body_com,
    float* frame_jacobian,
    float* center_of_mass_jacobian,
    unsigned char* output_status) {
  const unsigned agent = blockIdx.x * blockDim.x + threadIdx.x;
  if (agent >= agent_stride) return;

  for (unsigned body = 0; body < body_count; ++body)
    for (unsigned component = 0; component < 6; ++component)
      for (unsigned coordinate = 0; coordinate < generalized_coordinate_count; ++coordinate)
        frame_jacobian[frame_index(body, component, coordinate,
            generalized_coordinate_count, agent_stride, agent)] = 0.0f;
  for (unsigned component = 0; component < 3; ++component)
    for (unsigned coordinate = 0; coordinate < generalized_coordinate_count; ++coordinate)
      center_of_mass_jacobian[com_index(component, coordinate,
          generalized_coordinate_count, agent_stride, agent)] = 0.0f;
  output_status[agent] = input_status[agent];
  if (input_status[agent] != 1) return;

  float root_pose[12];
  load_pose(body_pose, root_body, agent, agent_stride, root_pose);
  float root_origin[3] = {root_pose[9], root_pose[10], root_pose[11]};

  for (unsigned body = 0; body < body_count; ++body) {
    float current_pose[12];
    load_pose(body_pose, body, agent, agent_stride, current_pose);
    float body_origin[3] = {current_pose[9], current_pose[10], current_pose[11]};
    float root_to_body[3] = {body_origin[0] - root_origin[0],
                             body_origin[1] - root_origin[1],
                             body_origin[2] - root_origin[2]};
    #pragma unroll
    for (unsigned axis = 0; axis < 3; ++axis) {
      frame_jacobian[frame_index(body, axis, axis,
          generalized_coordinate_count, agent_stride, agent)] = 1.0f;
      frame_jacobian[frame_index(body, 3 + axis, 3 + axis,
          generalized_coordinate_count, agent_stride, agent)] = 1.0f;
      float basis[3] = {0.0f, 0.0f, 0.0f};
      basis[axis] = 1.0f;
      float linear[3];
      cross_product(basis, root_to_body, linear);
      #pragma unroll
      for (unsigned component = 0; component < 3; ++component)
        frame_jacobian[frame_index(body, 3 + component, axis,
            generalized_coordinate_count, agent_stride, agent)] = linear[component];
    }

    unsigned ancestor = body;
    while (body_parent_joint[ancestor] != 0xffffffffu) {
      const unsigned joint = body_parent_joint[ancestor];
      const unsigned parent_body = joint_parent[joint];
      float parent_pose[12], fixed_pose[12], joint_pose[12];
      load_pose(body_pose, parent_body, agent, agent_stride, parent_pose);
      #pragma unroll
      for (unsigned component = 0; component < 12; ++component)
        fixed_pose[component] = joint_parent_from_joint[joint * 12 + component];
      compose_fixed_pose(parent_pose, fixed_pose, joint_pose);
      const unsigned coordinate = joint_coordinate[joint];
      if (coordinate != 0xffffffffu) {
        float local_axis[3] = {joint_axis[joint * 3], joint_axis[joint * 3 + 1],
                               joint_axis[joint * 3 + 2]};
        float axis_world[3];
        transform_vector(joint_pose, local_axis, axis_world);
        const unsigned generalized_coordinate = 6 + coordinate;
        if (joint_kind[joint] == 1) {
          float joint_to_body[3] = {body_origin[0] - joint_pose[9],
                                    body_origin[1] - joint_pose[10],
                                    body_origin[2] - joint_pose[11]};
          float linear[3];
          cross_product(axis_world, joint_to_body, linear);
          #pragma unroll
          for (unsigned component = 0; component < 3; ++component) {
            frame_jacobian[frame_index(body, component, generalized_coordinate,
                generalized_coordinate_count, agent_stride, agent)] = axis_world[component];
            frame_jacobian[frame_index(body, 3 + component, generalized_coordinate,
                generalized_coordinate_count, agent_stride, agent)] = linear[component];
          }
        } else if (joint_kind[joint] == 2) {
          #pragma unroll
          for (unsigned component = 0; component < 3; ++component)
            frame_jacobian[frame_index(body, 3 + component, generalized_coordinate,
                generalized_coordinate_count, agent_stride, agent)] = axis_world[component];
        }
      }
      ancestor = parent_body;
    }
  }

  const float mass = total_mass[agent];
  if (mass <= 0.0f) return;
  for (unsigned body = 0; body < body_count; ++body) {
    if (body_mass[body] <= 0.0f) continue;
    float pose[12];
    load_pose(body_pose, body, agent, agent_stride, pose);
    float local_com[3] = {body_com[body * 3], body_com[body * 3 + 1],
                          body_com[body * 3 + 2]};
    float point_world[3];
    transform_point(pose, local_com, point_world);
    const float scale = body_mass[body] / mass;
    float root_to_point[3] = {point_world[0] - root_origin[0],
                              point_world[1] - root_origin[1],
                              point_world[2] - root_origin[2]};
    #pragma unroll
    for (unsigned axis = 0; axis < 3; ++axis) {
      float basis[3] = {0.0f, 0.0f, 0.0f};
      basis[axis] = 1.0f;
      float linear[3];
      cross_product(basis, root_to_point, linear);
      #pragma unroll
      for (unsigned component = 0; component < 3; ++component) {
        const unsigned index = com_index(component, axis,
            generalized_coordinate_count, agent_stride, agent);
        center_of_mass_jacobian[index] =
            fmaf(scale, linear[component], center_of_mass_jacobian[index]);
      }
      const unsigned index = com_index(axis, 3 + axis,
          generalized_coordinate_count, agent_stride, agent);
      center_of_mass_jacobian[index] =
          fmaf(scale, 1.0f, center_of_mass_jacobian[index]);
    }

    unsigned ancestor = body;
    while (body_parent_joint[ancestor] != 0xffffffffu) {
      const unsigned joint = body_parent_joint[ancestor];
      const unsigned parent_body = joint_parent[joint];
      float parent_pose[12], fixed_pose[12], joint_pose[12];
      load_pose(body_pose, parent_body, agent, agent_stride, parent_pose);
      #pragma unroll
      for (unsigned component = 0; component < 12; ++component)
        fixed_pose[component] = joint_parent_from_joint[joint * 12 + component];
      compose_fixed_pose(parent_pose, fixed_pose, joint_pose);
      const unsigned coordinate = joint_coordinate[joint];
      if (coordinate != 0xffffffffu) {
        float local_axis[3] = {joint_axis[joint * 3], joint_axis[joint * 3 + 1],
                               joint_axis[joint * 3 + 2]};
        float axis_world[3], value[3];
        transform_vector(joint_pose, local_axis, axis_world);
        if (joint_kind[joint] == 1) {
          float joint_to_point[3] = {point_world[0] - joint_pose[9],
                                     point_world[1] - joint_pose[10],
                                     point_world[2] - joint_pose[11]};
          cross_product(axis_world, joint_to_point, value);
        } else if (joint_kind[joint] == 2) {
          value[0] = axis_world[0]; value[1] = axis_world[1]; value[2] = axis_world[2];
        } else {
          value[0] = value[1] = value[2] = 0.0f;
        }
        const unsigned generalized_coordinate = 6 + coordinate;
        #pragma unroll
        for (unsigned component = 0; component < 3; ++component) {
          const unsigned index = com_index(component, generalized_coordinate,
              generalized_coordinate_count, agent_stride, agent);
          center_of_mass_jacobian[index] =
              fmaf(scale, value[component], center_of_mass_jacobian[index]);
        }
      }
      ancestor = parent_body;
    }
  }
}

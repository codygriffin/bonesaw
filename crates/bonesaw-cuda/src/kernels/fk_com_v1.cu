extern "C" __device__ __forceinline__ void compose_pose(
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

extern "C" __device__ __forceinline__ bool valid_rotation(const float pose[12]) {
  constexpr float tolerance = 2.0e-3f;
  #pragma unroll
  for (unsigned row = 0; row < 3; ++row) {
    float norm = fmaf(pose[row * 3], pose[row * 3],
        fmaf(pose[row * 3 + 1], pose[row * 3 + 1],
             pose[row * 3 + 2] * pose[row * 3 + 2]));
    if (fabsf(norm - 1.0f) > tolerance) return false;
    for (unsigned other = row + 1; other < 3; ++other) {
      float dot = fmaf(pose[row * 3], pose[other * 3],
          fmaf(pose[row * 3 + 1], pose[other * 3 + 1],
               pose[row * 3 + 2] * pose[other * 3 + 2]));
      if (fabsf(dot) > tolerance) return false;
    }
  }
  float determinant = fmaf(
      pose[0], fmaf(pose[4], pose[8], -(pose[5] * pose[7])),
      -fmaf(pose[1], fmaf(pose[3], pose[8], -(pose[5] * pose[6])),
            -(pose[2] * fmaf(pose[3], pose[7], -(pose[4] * pose[6])))));
  return fabsf(determinant - 1.0f) <= 4.0f * tolerance;
}

extern "C" __device__ __forceinline__ void joint_motion(
    unsigned kind, const float axis[3], float value, float output[12]) {
  #pragma unroll
  for (unsigned component = 0; component < 12; ++component) output[component] = 0.0f;
  output[0] = output[4] = output[8] = 1.0f;
  if (kind == 1) {
    float sine, cosine;
    sincosf(value, &sine, &cosine);
    const float one_minus_cosine = 1.0f - cosine;
    const float x = axis[0], y = axis[1], z = axis[2];
    const float xx = x * x * one_minus_cosine;
    const float yy = y * y * one_minus_cosine;
    const float zz = z * z * one_minus_cosine;
    const float xy = x * y * one_minus_cosine;
    const float xz = x * z * one_minus_cosine;
    const float yz = y * z * one_minus_cosine;
    const float xs = x * sine, ys = y * sine, zs = z * sine;
    output[0] = xx + cosine; output[1] = xy - zs; output[2] = xz + ys;
    output[3] = xy + zs; output[4] = yy + cosine; output[5] = yz - xs;
    output[6] = xz - ys; output[7] = yz + xs; output[8] = zz + cosine;
  } else if (kind == 2) {
    output[9] = axis[0] * value;
    output[10] = axis[1] * value;
    output[11] = axis[2] * value;
  }
}

extern "C" __global__ void bonesaw_fk_com_v1(
    const float* q,
    const float* root_pose,
    const unsigned char* state_status,
    unsigned active_agents,
    unsigned coordinate_count,
    unsigned agent_stride,
    unsigned root_body,
    unsigned joint_count,
    unsigned body_count,
    const unsigned* joint_parent,
    const unsigned* joint_child,
    const unsigned* joint_coordinate,
    const unsigned* joint_kind,
    const float* joint_parent_from_joint,
    const float* joint_axis,
    const float* body_mass,
    const float* body_com,
    float* body_pose,
    float* center_of_mass,
    float* total_mass_output,
    unsigned char* output_status) {
  const unsigned agent = blockIdx.x * blockDim.x + threadIdx.x;
  if (agent >= agent_stride) return;

  for (unsigned body = 0; body < body_count; ++body)
    for (unsigned component = 0; component < 12; ++component)
      body_pose[(body * 12 + component) * agent_stride + agent] = 0.0f;
  center_of_mass[agent] = 0.0f;
  center_of_mass[agent_stride + agent] = 0.0f;
  center_of_mass[2 * agent_stride + agent] = 0.0f;
  total_mass_output[agent] = 0.0f;
  output_status[agent] = 0;
  if (agent >= active_agents || state_status[agent] != 1) {
    if (agent < active_agents) output_status[agent] = state_status[agent];
    return;
  }

  float root[12];
  #pragma unroll
  for (unsigned component = 0; component < 12; ++component)
    root[component] = root_pose[component * agent_stride + agent];
  if (!valid_rotation(root)) {
    output_status[agent] = 2;
    return;
  }
  #pragma unroll
  for (unsigned component = 0; component < 12; ++component)
    body_pose[(root_body * 12 + component) * agent_stride + agent] = root[component];

  for (unsigned joint = 0; joint < joint_count; ++joint) {
    float parent[12], fixed_pose[12], joint_world[12], motion[12], child[12];
    const unsigned parent_body = joint_parent[joint];
    #pragma unroll
    for (unsigned component = 0; component < 12; ++component) {
      parent[component] = body_pose[(parent_body * 12 + component) * agent_stride + agent];
      fixed_pose[component] = joint_parent_from_joint[joint * 12 + component];
    }
    compose_pose(parent, fixed_pose, joint_world);
    float axis[3] = {joint_axis[joint * 3], joint_axis[joint * 3 + 1],
                     joint_axis[joint * 3 + 2]};
    const unsigned coordinate = joint_coordinate[joint];
    const float value = coordinate == 0xffffffffu ? 0.0f
        : q[coordinate * agent_stride + agent];
    joint_motion(joint_kind[joint], axis, value, motion);
    compose_pose(joint_world, motion, child);
    const unsigned child_body = joint_child[joint];
    #pragma unroll
    for (unsigned component = 0; component < 12; ++component)
      body_pose[(child_body * 12 + component) * agent_stride + agent] = child[component];
  }

  float weighted[3] = {0.0f, 0.0f, 0.0f};
  float total_mass = 0.0f;
  for (unsigned body = 0; body < body_count; ++body) {
    float pose[12];
    #pragma unroll
    for (unsigned component = 0; component < 12; ++component)
      pose[component] = body_pose[(body * 12 + component) * agent_stride + agent];
    const float local_x = body_com[body * 3];
    const float local_y = body_com[body * 3 + 1];
    const float local_z = body_com[body * 3 + 2];
    float world[3];
    #pragma unroll
    for (unsigned row = 0; row < 3; ++row) {
      world[row] = pose[row * 3] * local_x;
      world[row] = fmaf(pose[row * 3 + 1], local_y, world[row]);
      world[row] = fmaf(pose[row * 3 + 2], local_z, world[row]);
      world[row] += pose[9 + row];
      weighted[row] = fmaf(body_mass[body], world[row], weighted[row]);
    }
    total_mass += body_mass[body];
  }
  if (total_mass > 0.0f) {
    center_of_mass[agent] = weighted[0] / total_mass;
    center_of_mass[agent_stride + agent] = weighted[1] / total_mass;
    center_of_mass[2 * agent_stride + agent] = weighted[2] / total_mass;
  }
  total_mass_output[agent] = total_mass;
  output_status[agent] = 1;
}

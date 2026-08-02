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

extern "C" __device__ __forceinline__ unsigned point_index(
    unsigned slot, unsigned component, unsigned agent_stride, unsigned agent) {
  return (slot * 3 + component) * agent_stride + agent;
}

extern "C" __device__ __forceinline__ unsigned point_jacobian_index(
    unsigned slot, unsigned component, unsigned coordinate,
    unsigned generalized_coordinate_count, unsigned agent_stride,
    unsigned agent) {
  return ((slot * 3 + component) * generalized_coordinate_count + coordinate)
      * agent_stride + agent;
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

extern "C" __device__ __forceinline__ void load_body_vector(
    const float* storage, unsigned body, unsigned agent,
    unsigned agent_stride, float output[3]) {
  #pragma unroll
  for (unsigned component = 0; component < 3; ++component)
    output[component] = storage[body_vector_index(
        body, component, agent_stride, agent)];
}

extern "C" __global__ void bonesaw_point_queries_v1(
    const float* body_pose,
    const float* frame_jacobian,
    const unsigned char* input_status,
    unsigned agent_stride,
    unsigned point_query_count,
    unsigned generalized_coordinate_count,
    const unsigned* query_frame,
    const float* query_point_in_frame,
    const float* angular_velocity,
    const float* angular_acceleration,
    const float* linear_acceleration_origin,
    float* point_position,
    float* point_jacobian,
    float* point_bias_acceleration,
    unsigned char* output_status) {
  const unsigned agent = blockIdx.x * blockDim.x + threadIdx.x;
  if (agent >= agent_stride) return;

  for (unsigned slot = 0; slot < point_query_count; ++slot) {
    #pragma unroll
    for (unsigned component = 0; component < 3; ++component) {
      point_position[point_index(slot, component, agent_stride, agent)] = 0.0f;
      point_bias_acceleration[point_index(
          slot, component, agent_stride, agent)] = 0.0f;
      for (unsigned coordinate = 0;
           coordinate < generalized_coordinate_count; ++coordinate)
        point_jacobian[point_jacobian_index(slot, component, coordinate,
            generalized_coordinate_count, agent_stride, agent)] = 0.0f;
    }
  }
  output_status[agent] = input_status[agent];
  if (input_status[agent] != 1) return;

  for (unsigned slot = 0; slot < point_query_count; ++slot) {
    const unsigned body = query_frame[slot];
    float pose[12];
    load_pose(body_pose, body, agent, agent_stride, pose);
    float local_offset[3] = {query_point_in_frame[slot * 3],
                             query_point_in_frame[slot * 3 + 1],
                             query_point_in_frame[slot * 3 + 2]};
    float offset_world[3];
    transform_vector(pose, local_offset, offset_world);
    #pragma unroll
    for (unsigned component = 0; component < 3; ++component)
      point_position[point_index(slot, component, agent_stride, agent)] =
          pose[9 + component] + offset_world[component];

    for (unsigned coordinate = 0;
         coordinate < generalized_coordinate_count; ++coordinate) {
      float angular[3], linear_origin[3];
      #pragma unroll
      for (unsigned component = 0; component < 3; ++component) {
        angular[component] = frame_jacobian[frame_index(body, component,
            coordinate, generalized_coordinate_count, agent_stride, agent)];
        linear_origin[component] = frame_jacobian[frame_index(body,
            3 + component, coordinate, generalized_coordinate_count,
            agent_stride, agent)];
      }
      float angular_cross_offset[3];
      cross_product(angular, offset_world, angular_cross_offset);
      #pragma unroll
      for (unsigned component = 0; component < 3; ++component)
        point_jacobian[point_jacobian_index(slot, component, coordinate,
            generalized_coordinate_count, agent_stride, agent)] =
            linear_origin[component] + angular_cross_offset[component];
    }

    float omega[3], alpha[3], origin_acceleration[3];
    load_body_vector(angular_velocity, body, agent, agent_stride, omega);
    load_body_vector(angular_acceleration, body, agent, agent_stride, alpha);
    load_body_vector(linear_acceleration_origin, body, agent, agent_stride,
                     origin_acceleration);
    float alpha_cross_offset[3], omega_cross_offset[3], centripetal[3];
    cross_product(alpha, offset_world, alpha_cross_offset);
    cross_product(omega, offset_world, omega_cross_offset);
    cross_product(omega, omega_cross_offset, centripetal);
    #pragma unroll
    for (unsigned component = 0; component < 3; ++component)
      point_bias_acceleration[point_index(
          slot, component, agent_stride, agent)] =
          origin_acceleration[component] + alpha_cross_offset[component]
          + centripetal[component];
  }
}

extern "C" __device__ __forceinline__ unsigned vector_index(
    unsigned slot, unsigned component, unsigned agent_stride, unsigned agent) {
  return (slot * 3 + component) * agent_stride + agent;
}

extern "C" __device__ __forceinline__ unsigned matrix_index(
    unsigned slot, unsigned component, unsigned coordinate,
    unsigned generalized_coordinate_count, unsigned agent_stride,
    unsigned agent) {
  return ((slot * 3 + component) * generalized_coordinate_count + coordinate)
      * agent_stride + agent;
}

extern "C" __device__ __forceinline__ bool axis_enabled(
    unsigned mode, unsigned component) {
  if (mode == 1) return component < 3;
  if (mode == 2) return component == 2;
  if (mode == 3) return component == 1 || component == 2;
  return false;
}

extern "C" __global__ void bonesaw_emission_v1(
    const float* generalized_velocity,
    const float* point_position,
    const float* point_jacobian,
    const float* point_bias_acceleration,
    const unsigned char* input_status,
    unsigned emission_active_agents,
    unsigned dynamics_active_agents,
    unsigned agent_stride,
    unsigned point_task_count,
    unsigned contact_lock_count,
    unsigned generalized_coordinate_count,
    const unsigned* task_query_slot,
    const float* task_bandwidth_hz,
    const unsigned* contact_query_slot,
    const unsigned* contact_mode,
    const unsigned char* task_active_input,
    const float* target_position,
    const float* target_velocity,
    const float* target_acceleration,
    const unsigned char* contact_active_input,
    const float* contact_desired_acceleration,
    unsigned char* task_active_output,
    float* task_position_error,
    float* task_velocity_error,
    float* task_desired_acceleration,
    float* task_jacobian,
    float* task_rhs,
    unsigned char* contact_active_output,
    float* contact_jacobian,
    float* contact_rhs,
    unsigned char* output_status) {
  const unsigned agent = blockIdx.x * blockDim.x + threadIdx.x;
  if (agent >= agent_stride) return;

  for (unsigned slot = 0; slot < point_task_count; ++slot) {
    task_active_output[slot * agent_stride + agent] = 0;
    for (unsigned component = 0; component < 3; ++component) {
      const unsigned index = vector_index(slot, component, agent_stride, agent);
      task_position_error[index] = 0.0f;
      task_velocity_error[index] = 0.0f;
      task_desired_acceleration[index] = 0.0f;
      task_rhs[index] = 0.0f;
      for (unsigned coordinate = 0;
           coordinate < generalized_coordinate_count; ++coordinate)
        task_jacobian[matrix_index(slot, component, coordinate,
            generalized_coordinate_count, agent_stride, agent)] = 0.0f;
    }
  }
  for (unsigned slot = 0; slot < contact_lock_count; ++slot) {
    contact_active_output[slot * agent_stride + agent] = 0;
    for (unsigned component = 0; component < 3; ++component) {
      contact_rhs[vector_index(slot, component, agent_stride, agent)] = 0.0f;
      for (unsigned coordinate = 0;
           coordinate < generalized_coordinate_count; ++coordinate)
        contact_jacobian[matrix_index(slot, component, coordinate,
            generalized_coordinate_count, agent_stride, agent)] = 0.0f;
    }
  }

  unsigned char status = input_status[agent];
  if (status == 1 &&
      (agent >= emission_active_agents || agent >= dynamics_active_agents))
    status = 0;
  if (status == 1) {
    for (unsigned slot = 0; slot < point_task_count; ++slot) {
      const unsigned char active = task_active_input[slot * agent_stride + agent];
      if (active > 1) status = 2;
      if (active == 1) {
        for (unsigned component = 0; component < 3; ++component) {
          const unsigned index = vector_index(slot, component, agent_stride, agent);
          if (!isfinite(target_position[index]) ||
              !isfinite(target_velocity[index]) ||
              !isfinite(target_acceleration[index])) status = 2;
        }
      }
    }
    for (unsigned slot = 0; slot < contact_lock_count; ++slot) {
      const unsigned char active = contact_active_input[slot * agent_stride + agent];
      if (active > 1) status = 2;
      if (active == 1)
        for (unsigned component = 0; component < 3; ++component)
          if (!isfinite(contact_desired_acceleration[
                  vector_index(slot, component, agent_stride, agent)])) status = 2;
    }
  }
  output_status[agent] = status;
  if (status != 1) return;

  for (unsigned slot = 0; slot < point_task_count; ++slot) {
    if (task_active_input[slot * agent_stride + agent] == 0) continue;
    task_active_output[slot * agent_stride + agent] = 1;
    const unsigned query = task_query_slot[slot];
    const float omega = 6.28318530717958647692f * task_bandwidth_hz[slot];
    const float velocity_gain = 2.0f * omega;
    const float position_gain = omega * omega;
    for (unsigned component = 0; component < 3; ++component) {
      float current_velocity = 0.0f;
      for (unsigned coordinate = 0;
           coordinate < generalized_coordinate_count; ++coordinate) {
        const float jacobian = point_jacobian[matrix_index(query, component,
            coordinate, generalized_coordinate_count, agent_stride, agent)];
        current_velocity = fmaf(jacobian,
            generalized_velocity[coordinate * agent_stride + agent],
            current_velocity);
        task_jacobian[matrix_index(slot, component, coordinate,
            generalized_coordinate_count, agent_stride, agent)] = jacobian;
      }
      const unsigned task_index = vector_index(slot, component, agent_stride, agent);
      const unsigned point_index = vector_index(query, component, agent_stride, agent);
      const float position_error = target_position[task_index] - point_position[point_index];
      const float velocity_error = target_velocity[task_index] - current_velocity;
      const float desired = fmaf(position_gain, position_error,
          fmaf(velocity_gain, velocity_error, target_acceleration[task_index]));
      task_position_error[task_index] = position_error;
      task_velocity_error[task_index] = velocity_error;
      task_desired_acceleration[task_index] = desired;
      task_rhs[task_index] = desired - point_bias_acceleration[point_index];
    }
  }

  for (unsigned slot = 0; slot < contact_lock_count; ++slot) {
    if (contact_active_input[slot * agent_stride + agent] == 0) continue;
    contact_active_output[slot * agent_stride + agent] = 1;
    const unsigned query = contact_query_slot[slot];
    const unsigned mode = contact_mode[slot];
    for (unsigned component = 0; component < 3; ++component) {
      if (!axis_enabled(mode, component)) continue;
      for (unsigned coordinate = 0;
           coordinate < generalized_coordinate_count; ++coordinate)
        contact_jacobian[matrix_index(slot, component, coordinate,
            generalized_coordinate_count, agent_stride, agent)] =
            point_jacobian[matrix_index(query, component, coordinate,
                generalized_coordinate_count, agent_stride, agent)];
      const unsigned contact_index = vector_index(slot, component, agent_stride, agent);
      const unsigned point_index = vector_index(query, component, agent_stride, agent);
      contact_rhs[contact_index] = contact_desired_acceleration[contact_index]
          - point_bias_acceleration[point_index];
    }
  }
}

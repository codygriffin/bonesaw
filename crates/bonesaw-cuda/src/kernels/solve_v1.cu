// FixedLevelApproximate v1. One CUDA thread owns one complete agent and all
// reduction order. There is no early exit, device allocation, cross-thread
// communication, or executable-command fallback.

static constexpr unsigned HARD_SWEEPS = 64;
static constexpr unsigned TASK_SWEEPS = 32;
static constexpr unsigned RESTORE_SWEEPS = 2;
static constexpr unsigned PRIORITY_LEVELS = 5;
static constexpr float HARD_TOLERANCE = 5.0e-4f;
static constexpr float SOFT_TOLERANCE = 5.0e-4f;
static constexpr float ROW_NORM_EPSILON = 1.0e-12f;
static constexpr float REGULARIZATION = 1.0e-6f;
static constexpr float RELAXATION = 0.85f;

extern "C" __device__ __forceinline__ unsigned sat_add(unsigned a,
                                                         unsigned b) {
  return a > 0xffffffffu - b ? 0xffffffffu : a + b;
}

extern "C" __device__ __forceinline__ unsigned vector_index(
    unsigned slot, unsigned component, unsigned stride, unsigned agent) {
  return (slot * 3 + component) * stride + agent;
}

extern "C" __device__ __forceinline__ unsigned matrix_index(
    unsigned slot, unsigned component, unsigned coordinate, unsigned dof,
    unsigned stride, unsigned agent) {
  return ((slot * 3 + component) * dof + coordinate) * stride + agent;
}

extern "C" __device__ __forceinline__ unsigned generalized_index(
    unsigned coordinate, unsigned stride, unsigned agent) {
  return coordinate * stride + agent;
}

extern "C" __device__ __forceinline__ unsigned locked_matrix_index(
    unsigned row, unsigned coordinate, unsigned dof, unsigned stride,
    unsigned agent) {
  return (row * dof + coordinate) * stride + agent;
}

extern "C" __device__ __forceinline__ bool axis_enabled(
    unsigned mode, unsigned component) {
  if (mode == 1) return component < 3;
  if (mode == 2) return component == 2;
  if (mode == 3) return component == 1 || component == 2;
  return false;
}

extern "C" __device__ __forceinline__ float clamp_scalar(
    float value, float lower, float upper) {
  return fminf(fmaxf(value, lower), upper);
}

extern "C" __device__ unsigned clamp_bounds(
    unsigned agent, unsigned dof, unsigned stride, const float* lower,
    const float* upper, float* x) {
  unsigned clipped = 0;
  for (unsigned coordinate = 0; coordinate < dof; ++coordinate) {
    const unsigned index = generalized_index(coordinate, stride, agent);
    const float value = x[index];
    const float clamped = clamp_scalar(value, lower[index], upper[index]);
    clipped = sat_add(
        clipped, __float_as_uint(value) != __float_as_uint(clamped));
    x[index] = clamped;
  }
  return clipped;
}

extern "C" __device__ float contact_dot(
    unsigned agent, unsigned slot, unsigned component, unsigned dof,
    unsigned stride, const float* jacobian, const float* x) {
  float value = 0.0f;
  for (unsigned coordinate = 0; coordinate < dof; ++coordinate)
    value = fmaf(jacobian[matrix_index(slot, component, coordinate, dof,
                                      stride, agent)],
                 x[generalized_index(coordinate, stride, agent)], value);
  return value;
}

extern "C" __device__ float contact_norm_squared(
    unsigned agent, unsigned slot, unsigned component, unsigned dof,
    unsigned stride, const float* jacobian) {
  float value = 0.0f;
  for (unsigned coordinate = 0; coordinate < dof; ++coordinate) {
    const float coefficient = jacobian[matrix_index(
        slot, component, coordinate, dof, stride, agent)];
    value = fmaf(coefficient, coefficient, value);
  }
  return value;
}

extern "C" __device__ float locked_dot(
    unsigned agent, unsigned row, unsigned dof, unsigned stride,
    const float* locked_rows, const float* x) {
  float value = 0.0f;
  for (unsigned coordinate = 0; coordinate < dof; ++coordinate)
    value = fmaf(locked_rows[locked_matrix_index(row, coordinate, dof, stride,
                                                agent)],
                 x[generalized_index(coordinate, stride, agent)], value);
  return value;
}

extern "C" __device__ float locked_norm_squared(
    unsigned agent, unsigned row, unsigned dof, unsigned stride,
    const float* locked_rows) {
  float value = 0.0f;
  for (unsigned coordinate = 0; coordinate < dof; ++coordinate) {
    const float coefficient = locked_rows[locked_matrix_index(
        row, coordinate, dof, stride, agent)];
    value = fmaf(coefficient, coefficient, value);
  }
  return value;
}

extern "C" __device__ unsigned project_sweep(
    unsigned agent, unsigned contact_count, unsigned dof, unsigned stride,
    unsigned locked_count, const unsigned char* contact_active,
    const float* contact_jacobian, const float* contact_rhs,
    const float* lower, const float* upper, const float* locked_rows,
    const float* locked_rhs, float* x) {
  unsigned clipped = 0;
  for (unsigned slot = 0; slot < contact_count; ++slot) {
    if (contact_active[slot * stride + agent] == 0) continue;
    for (unsigned component = 0; component < 3; ++component) {
      const float norm = contact_norm_squared(agent, slot, component, dof,
                                              stride, contact_jacobian);
      if (norm <= ROW_NORM_EPSILON) continue;
      const float residual = contact_rhs[vector_index(
                                 slot, component, stride, agent)] -
                             contact_dot(agent, slot, component, dof, stride,
                                         contact_jacobian, x);
      const float alpha = residual / norm;
      for (unsigned coordinate = 0; coordinate < dof; ++coordinate) {
        const unsigned index = generalized_index(coordinate, stride, agent);
        x[index] = fmaf(alpha,
                        contact_jacobian[matrix_index(
                            slot, component, coordinate, dof, stride, agent)],
                        x[index]);
      }
      clipped = sat_add(
          clipped, clamp_bounds(agent, dof, stride, lower, upper, x));
    }
  }
  for (unsigned row = 0; row < locked_count; ++row) {
    const float norm =
        locked_norm_squared(agent, row, dof, stride, locked_rows);
    if (norm <= ROW_NORM_EPSILON) continue;
    const float residual = locked_rhs[row * stride + agent] -
                           locked_dot(agent, row, dof, stride, locked_rows, x);
    const float alpha = residual / norm;
    for (unsigned coordinate = 0; coordinate < dof; ++coordinate) {
      const unsigned index = generalized_index(coordinate, stride, agent);
      x[index] = fmaf(alpha,
                      locked_rows[locked_matrix_index(
                          row, coordinate, dof, stride, agent)],
                      x[index]);
    }
    clipped =
        sat_add(clipped, clamp_bounds(agent, dof, stride, lower, upper, x));
  }
  return clipped;
}

extern "C" __device__ float maximum_hard_violation(
    unsigned agent, unsigned contact_count, unsigned dof, unsigned stride,
    unsigned locked_count, const unsigned char* contact_active,
    const float* contact_jacobian, const float* contact_rhs,
    const float* lower, const float* upper, const float* locked_rows,
    const float* locked_rhs, const float* x) {
  float maximum = 0.0f;
  for (unsigned slot = 0; slot < contact_count; ++slot) {
    if (contact_active[slot * stride + agent] == 0) continue;
    for (unsigned component = 0; component < 3; ++component) {
      const float residual = fabsf(contact_dot(agent, slot, component, dof,
                                               stride, contact_jacobian, x) -
                                   contact_rhs[vector_index(
                                       slot, component, stride, agent)]);
      maximum = fmaxf(maximum, residual);
    }
  }
  for (unsigned row = 0; row < locked_count; ++row)
    maximum = fmaxf(
        maximum,
        fabsf(locked_dot(agent, row, dof, stride, locked_rows, x) -
              locked_rhs[row * stride + agent]));
  for (unsigned coordinate = 0; coordinate < dof; ++coordinate) {
    const unsigned index = generalized_index(coordinate, stride, agent);
    maximum = fmaxf(maximum, fmaxf(lower[index] - x[index], 0.0f));
    maximum = fmaxf(maximum, fmaxf(x[index] - upper[index], 0.0f));
  }
  return maximum;
}

extern "C" __device__ float task_dot(
    unsigned agent, unsigned slot, unsigned component, unsigned dof,
    unsigned stride, const float* jacobian, const float* x) {
  float value = 0.0f;
  for (unsigned coordinate = 0; coordinate < dof; ++coordinate)
    value = fmaf(jacobian[matrix_index(slot, component, coordinate, dof,
                                      stride, agent)],
                 x[generalized_index(coordinate, stride, agent)], value);
  return value;
}

extern "C" __device__ float task_norm_squared(
    unsigned agent, unsigned slot, unsigned component, unsigned dof,
    unsigned stride, const float* jacobian) {
  float value = 0.0f;
  for (unsigned coordinate = 0; coordinate < dof; ++coordinate) {
    const float coefficient = jacobian[matrix_index(
        slot, component, coordinate, dof, stride, agent)];
    value = fmaf(coefficient, coefficient, value);
  }
  return value;
}

extern "C" __device__ float level_rms(
    unsigned agent, unsigned priority, unsigned task_count, unsigned dof,
    unsigned stride, const unsigned char* task_priority,
    const unsigned char* task_active, const float* task_jacobian,
    const float* task_rhs, const float* x) {
  float squared = 0.0f;
  unsigned rows = 0;
  for (unsigned slot = 0; slot < task_count; ++slot) {
    if (task_priority[slot] != priority ||
        task_active[slot * stride + agent] == 0)
      continue;
    for (unsigned component = 0; component < 3; ++component) {
      const float residual =
          task_dot(agent, slot, component, dof, stride, task_jacobian, x) -
          task_rhs[vector_index(slot, component, stride, agent)];
      squared = fmaf(residual, residual, squared);
      ++rows;
    }
  }
  return rows == 0 ? 0.0f : sqrtf(squared / static_cast<float>(rows));
}

extern "C" __device__ float maximum_locked_drift(
    unsigned agent, unsigned through_priority, unsigned locked_count,
    unsigned dof, unsigned stride, const float* locked_rows,
    const float* locked_reference_rhs, const unsigned char* locked_priority,
    const float* x) {
  float maximum = 0.0f;
  for (unsigned row = 0; row < locked_count; ++row) {
    if (locked_priority[row * stride + agent] > through_priority) continue;
    maximum = fmaxf(
        maximum,
        fabsf(locked_dot(agent, row, dof, stride, locked_rows, x) -
              locked_reference_rhs[row * stride + agent]));
  }
  return maximum;
}

extern "C" __device__ float minimum_bound_margin(
    unsigned agent, unsigned dof, unsigned stride, const float* lower,
    const float* upper, const float* x) {
  float minimum = __int_as_float(0x7f800000);
  for (unsigned coordinate = 0; coordinate < dof; ++coordinate) {
    const unsigned index = generalized_index(coordinate, stride, agent);
    minimum = fminf(minimum,
                    fminf(x[index] - lower[index], upper[index] - x[index]));
  }
  return minimum;
}

extern "C" __global__ void bonesaw_solve_v1(
    const unsigned char* emission_status, unsigned active_agents,
    unsigned agent_capacity, unsigned agent_stride, unsigned task_count,
    unsigned contact_count, unsigned dof,
    const unsigned char* task_priority, const float* task_weight,
    const unsigned* contact_mode, const unsigned char* task_active,
    const float* task_jacobian, const float* task_rhs,
    const unsigned char* contact_active, const float* contact_jacobian,
    const float* contact_rhs, const float* lower, const float* upper,
    float* command, float* candidate, unsigned char* status,
    unsigned* level_rows, float* level_rms_output,
    float* level_preservation_drift, float* initial_hard_violation,
    float* best_hard_violation, float* final_hard_violation,
    float* minimum_bound_margin_output, unsigned* hard_projection_sweeps,
    unsigned* task_sweeps, unsigned* clipped_updates,
    unsigned* active_hard_rows, unsigned* active_soft_rows, float* x,
    float* best, float* locked_rows, float* locked_rhs,
    float* locked_reference_rhs, unsigned char* locked_priority) {
  const unsigned agent = blockIdx.x * blockDim.x + threadIdx.x;
  if (agent >= agent_stride) return;

  status[agent] = 0;
  initial_hard_violation[agent] = 0.0f;
  best_hard_violation[agent] = 0.0f;
  final_hard_violation[agent] = 0.0f;
  minimum_bound_margin_output[agent] = __int_as_float(0x7f800000);
  hard_projection_sweeps[agent] = 0;
  task_sweeps[agent] = 0;
  clipped_updates[agent] = 0;
  active_hard_rows[agent] = 0;
  active_soft_rows[agent] = 0;
  for (unsigned priority = 0; priority < PRIORITY_LEVELS; ++priority) {
    const unsigned index = priority * agent_stride + agent;
    level_rows[index] = 0;
    level_rms_output[index] = 0.0f;
    level_preservation_drift[index] = 0.0f;
  }
  for (unsigned coordinate = 0; coordinate < dof; ++coordinate) {
    const unsigned index = generalized_index(coordinate, agent_stride, agent);
    command[index] = 0.0f;
    candidate[index] = 0.0f;
    x[index] = 0.0f;
    best[index] = 0.0f;
  }
  const unsigned maximum_soft_rows = task_count * 3;
  for (unsigned row = 0; row < maximum_soft_rows; ++row) {
    const unsigned row_index = row * agent_stride + agent;
    locked_rhs[row_index] = 0.0f;
    locked_reference_rhs[row_index] = 0.0f;
    locked_priority[row_index] = 0;
    for (unsigned coordinate = 0; coordinate < dof; ++coordinate)
      locked_rows[locked_matrix_index(row, coordinate, dof, agent_stride,
                                      agent)] = 0.0f;
  }

  if (emission_status[agent] == 2) {
    status[agent] = 5;
    return;
  }
  if (emission_status[agent] != 1 || agent >= active_agents ||
      agent >= agent_capacity)
    return;

  bool invalid = false;
  for (unsigned coordinate = 0; coordinate < dof; ++coordinate) {
    const unsigned index = generalized_index(coordinate, agent_stride, agent);
    invalid = invalid || isnan(lower[index]) || isnan(upper[index]) ||
              lower[index] > upper[index];
    x[index] = clamp_scalar(0.0f, lower[index], upper[index]);
  }
  if (invalid) {
    status[agent] = 4;
    return;
  }

  for (unsigned slot = 0; slot < contact_count; ++slot) {
    if (contact_active[slot * agent_stride + agent] == 0) continue;
    for (unsigned component = 0; component < 3; ++component)
      active_hard_rows[agent] += axis_enabled(contact_mode[slot], component);
  }
  for (unsigned slot = 0; slot < task_count; ++slot)
    if (task_active[slot * agent_stride + agent] != 0)
      active_soft_rows[agent] += 3;

  float initial = maximum_hard_violation(
      agent, contact_count, dof, agent_stride, 0, contact_active,
      contact_jacobian, contact_rhs, lower, upper, locked_rows, locked_rhs, x);
  initial_hard_violation[agent] = initial;
  float best_violation = initial;
  for (unsigned coordinate = 0; coordinate < dof; ++coordinate) {
    const unsigned index = generalized_index(coordinate, agent_stride, agent);
    best[index] = x[index];
  }
  unsigned clipped = 0;
  for (unsigned sweep = 0; sweep < HARD_SWEEPS; ++sweep) {
    clipped = sat_add(
        clipped,
        project_sweep(agent, contact_count, dof, agent_stride, 0,
                      contact_active, contact_jacobian, contact_rhs, lower,
                      upper, locked_rows, locked_rhs, x));
    const float violation = maximum_hard_violation(
        agent, contact_count, dof, agent_stride, 0, contact_active,
        contact_jacobian, contact_rhs, lower, upper, locked_rows, locked_rhs, x);
    if (violation < best_violation) {
      best_violation = violation;
      for (unsigned coordinate = 0; coordinate < dof; ++coordinate) {
        const unsigned index = generalized_index(coordinate, agent_stride, agent);
        best[index] = x[index];
      }
    }
  }
  hard_projection_sweeps[agent] = HARD_SWEEPS;
  if (best_violation > HARD_TOLERANCE || !isfinite(best_violation)) {
    for (unsigned coordinate = 0; coordinate < dof; ++coordinate) {
      const unsigned index = generalized_index(coordinate, agent_stride, agent);
      candidate[index] = best[index];
    }
    best_hard_violation[agent] = best_violation;
    final_hard_violation[agent] = maximum_hard_violation(
        agent, contact_count, dof, agent_stride, 0, contact_active,
        contact_jacobian, contact_rhs, lower, upper, locked_rows, locked_rhs, x);
    minimum_bound_margin_output[agent] = minimum_bound_margin(
        agent, dof, agent_stride, lower, upper, best);
    clipped_updates[agent] = clipped;
    status[agent] = 3;
    return;
  }
  for (unsigned coordinate = 0; coordinate < dof; ++coordinate) {
    const unsigned index = generalized_index(coordinate, agent_stride, agent);
    x[index] = best[index];
  }

  unsigned locked_count = 0;
  for (unsigned priority = 0; priority < PRIORITY_LEVELS; ++priority) {
    unsigned rows = 0;
    for (unsigned slot = 0; slot < task_count; ++slot)
      if (task_priority[slot] == priority &&
          task_active[slot * agent_stride + agent] != 0)
        rows += 3;
    const unsigned level_index = priority * agent_stride + agent;
    level_rows[level_index] = rows;
    if (rows == 0) continue;

    for (unsigned sweep = 0; sweep < TASK_SWEEPS; ++sweep) {
      for (unsigned slot = 0; slot < task_count; ++slot) {
        if (task_priority[slot] != priority ||
            task_active[slot * agent_stride + agent] == 0)
          continue;
        for (unsigned component = 0; component < 3; ++component) {
          const float norm = task_norm_squared(agent, slot, component, dof,
                                               agent_stride, task_jacobian);
          if (norm <= ROW_NORM_EPSILON) continue;
          const float residual = task_rhs[vector_index(
                                     slot, component, agent_stride, agent)] -
                                 task_dot(agent, slot, component, dof,
                                          agent_stride, task_jacobian, x);
          const float weight = fmaxf(task_weight[slot], 0.0f);
          const float alpha = RELAXATION * weight /
                              fmaf(weight, norm, REGULARIZATION);
          for (unsigned coordinate = 0; coordinate < dof; ++coordinate) {
            const unsigned index =
                generalized_index(coordinate, agent_stride, agent);
            x[index] = fmaf(
                alpha * residual,
                task_jacobian[matrix_index(slot, component, coordinate, dof,
                                           agent_stride, agent)],
                x[index]);
          }
          clipped = sat_add(
              clipped,
              clamp_bounds(agent, dof, agent_stride, lower, upper, x));
          for (unsigned restore = 0; restore < RESTORE_SWEEPS; ++restore)
            clipped = sat_add(
                clipped,
                project_sweep(agent, contact_count, dof, agent_stride,
                              locked_count, contact_active, contact_jacobian,
                              contact_rhs, lower, upper, locked_rows,
                              locked_rhs, x));
        }
      }
    }
    task_sweeps[agent] += TASK_SWEEPS;
    level_rms_output[level_index] = level_rms(
        agent, priority, task_count, dof, agent_stride, task_priority,
        task_active, task_jacobian, task_rhs, x);
    for (unsigned slot = 0; slot < task_count; ++slot) {
      if (task_priority[slot] != priority ||
          task_active[slot * agent_stride + agent] == 0)
        continue;
      for (unsigned component = 0; component < 3; ++component) {
        const unsigned row = locked_count;
        for (unsigned coordinate = 0; coordinate < dof; ++coordinate)
          locked_rows[locked_matrix_index(row, coordinate, dof, agent_stride,
                                          agent)] =
              task_jacobian[matrix_index(slot, component, coordinate, dof,
                                         agent_stride, agent)];
        const float achieved = task_dot(agent, slot, component, dof,
                                        agent_stride, task_jacobian, x);
        locked_rhs[row * agent_stride + agent] = achieved;
        locked_reference_rhs[row * agent_stride + agent] = achieved;
        locked_priority[row * agent_stride + agent] =
            static_cast<unsigned char>(priority);
        ++locked_count;
      }
    }
  }

  for (unsigned priority = 0; priority < PRIORITY_LEVELS; ++priority)
    level_preservation_drift[priority * agent_stride + agent] =
        maximum_locked_drift(agent, priority, locked_count, dof, agent_stride,
                             locked_rows, locked_reference_rhs,
                             locked_priority, x);

  const float final_hard = maximum_hard_violation(
      agent, contact_count, dof, agent_stride, locked_count, contact_active,
      contact_jacobian, contact_rhs, lower, upper, locked_rows, locked_rhs, x);
  best_violation = fminf(
      best_violation,
      maximum_hard_violation(agent, contact_count, dof, agent_stride, 0,
                             contact_active, contact_jacobian, contact_rhs,
                             lower, upper, locked_rows, locked_rhs, x));
  for (unsigned coordinate = 0; coordinate < dof; ++coordinate) {
    const unsigned index = generalized_index(coordinate, agent_stride, agent);
    candidate[index] = x[index];
  }
  best_hard_violation[agent] = best_violation;
  final_hard_violation[agent] = final_hard;
  minimum_bound_margin_output[agent] = minimum_bound_margin(
      agent, dof, agent_stride, lower, upper, x);
  clipped_updates[agent] = clipped;
  if (final_hard > HARD_TOLERANCE || !isfinite(final_hard)) {
    status[agent] = 3;
    return;
  }
  for (unsigned coordinate = 0; coordinate < dof; ++coordinate) {
    const unsigned index = generalized_index(coordinate, agent_stride, agent);
    command[index] = x[index];
  }
  bool has_residual = false;
  for (unsigned priority = 0; priority < PRIORITY_LEVELS; ++priority)
    has_residual = has_residual ||
                   level_rms_output[priority * agent_stride + agent] >
                       SOFT_TOLERANCE;
  status[agent] = has_residual || clipped > 0 ? 2 : 1;
}

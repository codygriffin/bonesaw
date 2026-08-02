# Bonesaw reconstruction-exposure CPU authority · r120

## Outcome

**PASS.** A fixed-state, policy-/physics-free Rust audit sweeps reconstruction
exposure from 0 to 10 ms. It calls the same typed error-growth primitive used by
the live boundary, intersects the joint-stopping interval over every
`q ± error, v ± error` corner, and executes the actual floating WBC with a tight
self-collision barrier. No state is integrated and no output is called plant
response.

The production live prediction horizon ends at 5 ms. The 7.5 and 10 ms rows are
diagnostic continuations of the growth curve only; live reconstruction would
withhold them before hard rows.

| exposure ms | live-eligible | q error mrad | point error mm | stopping upper loss rad/s² | robust self margin mm | required normal accel m/s² | bound ns/call | nominal/robust p50 µs | nominal/robust p99 µs |
|---|---|---|---|---|---|---|---|---|---|
| 0.0 | yes | 0.00000 | 0.00000 | 0.000000 | 10.00000 | 3.000000 | 51.90 | 11.892 / 11.903 | 18.024 / 17.933 |
| 0.5 | yes | 0.04075 | 0.02513 | 1.057176 | 9.94975 | 3.005025 | 55.19 | 12.032 / 12.063 | 18.304 / 18.124 |
| 1.0 | yes | 0.08300 | 0.05050 | 2.131490 | 9.89900 | 3.010100 | 52.96 | 11.912 / 11.942 | 16.872 / 16.982 |
| 2.5 | yes | 0.21875 | 0.12813 | 5.457612 | 9.74375 | 3.025625 | 52.57 | 11.813 / 11.852 | 16.972 / 17.112 |
| 5.0 | yes | 0.47500 | 0.26250 | 11.347379 | 9.47500 | 3.052500 | 53.12 | 11.823 / 11.853 | 17.333 / 17.373 |
| 7.5 | no · diagnostic only | 0.76875 | 0.40313 | 17.674619 | 9.19375 | 3.080625 | 53.72 | 11.823 / 11.862 | 16.771 / 16.972 |
| 10.0 | no · diagnostic only | 1.10000 | 0.55000 | 24.445561 | 8.90000 | 3.110000 | 52.71 | 11.822 / 11.862 | 16.842 / 16.812 |

## Five-millisecond live boundary

| signal | result |
|---|---|
| joint position / velocity error | 0.475000 mrad / 0.030000 rad/s |
| joint-stopping upper-bound loss | 11.347379 rad/s² |
| self-collision raw → robust margin | 10.000000 → 9.475000 mm |
| barrier required acceleration | 3.000000 → 3.052500 m/s² |
| error-bound cost | 53.12 ns/call |
| nominal / robust p50 | 11.823 / 11.853 µs (Δ +0.030) |
| nominal / robust p99 | 17.333 / 17.373 µs (Δ +0.040) |
| nominal / robust p99−p50 | 5.510 / 5.520 µs |

The alternating A/B loop changes call order every repeat so warmup and scheduler
drift do not systematically favor nominal or robust. Timing is descriptive for
this host. A small signed delta is not evidence that uncertainty makes the
dense solve intrinsically faster or slower; both paths emit the same number of
rows and differ only in scalar margin values.

## Allocation, memory, and determinism

| boundary | result |
|---|---|
| error growth · 200,000 calls/exposure | 0 allocation calls · 0 bytes |
| paired WBC · 5,000 nominal + robust calls/exposure | 0 allocation calls · 0 bytes |
| complete generalized-acceleration replay | bitwise identical at every exposure |
| reconstruction evidence value | 144 bytes |
| growth configuration value | 128 bytes |
| derived error-bound value | 56 bytes |

These are fixed Rust values embedded beside already preallocated WBC scratch and
output; the audit does not infer memory from process RSS. It directly counts
allocator calls and bytes around the hot primitives.

## Interpretation

- Error and consumed margins grow continuously and monotonically with exposure.
- The raw 10 mm collision geometry is invariant; only robust authority shrinks.
- At 5 ms, the represented-point radius consumes 0.525 mm and increases the
  required outward acceleration from 3.0000 to 3.0525 m/s².
- The near-limit joint witness loses 11.3474 rad/s² of its safe upper
  acceleration interval at 5 ms, unlike the live squat trace whose ±200 rad/s²
  cap masked this effect.
- The growth rates remain caller-authored deterministic contracts—not
  covariance, statistical confidence, calibrated estimator residuals, network
  evidence, hardware safety certification, or realized plant behavior.

## Retained audit

```json
{
  "claims": {
    "calibrated_estimator_or_probability": false,
    "hot_path_allocation_free": true,
    "monotone_error_growth": true,
    "monotone_joint_stopping_consumption": true,
    "monotone_self_collision_margin_consumption": true,
    "plant_response": false
  },
  "error_bound_repeats": 200000,
  "execution": "fixed_state_without_policy_physics_or_integration",
  "fixed_value_bytes": {
    "error_bound": 56,
    "error_growth": 128,
    "reconstruction_evidence": 144
  },
  "model": "models/tight_avoidance_toy.urdf",
  "nominal": {
    "allocated_bytes": 0,
    "allocation_calls": 0,
    "bitwise_repeat": true,
    "collision_margin_m": 0.009999999999999998,
    "required_normal_acceleration_mps2": 3.0,
    "timing_us": {
      "jitter_p99_minus_p50": 17.202999999999996,
      "max": 49.574,
      "mean": 15.98031519999989,
      "p50": 11.993,
      "p95": 24.506,
      "p99": 29.195999999999998
    }
  },
  "process_exit_code": 0,
  "process_stderr": "",
  "revision": "observation-uncertainty-exposure-r120",
  "schema": 1,
  "solve_repeats": 5000,
  "status": "pass",
  "sweep": [
    {
      "achieved_normal_acceleration_mps2": 3.0,
      "error_bound": {
        "center_of_mass_position_error_m": 0.0,
        "joint_position_error_rad": 0.0,
        "joint_velocity_error_rad_s": 0.0,
        "represented_point_position_error_m": 0.0,
        "root_rotation_error_rad": 0.0,
        "root_translation_error_m": 0.0
      },
      "error_bound_allocated_bytes": 0,
      "error_bound_allocation_calls": 0,
      "error_bound_ns_per_call": 51.901735,
      "exposure_ns": 0,
      "generalized_acceleration_slide_rad_s2": 3.0,
      "inside_live_prediction_horizon": true,
      "joint_stopping_interval_rad_s2": [
        -200.0,
        13.302180005079922
      ],
      "joint_stopping_upper_erosion_rad_s2": 0.0,
      "required_normal_acceleration_mps2": 3.0,
      "self_collision_margin_erosion_m": 0.0,
      "self_collision_raw_margin_m": 0.009999999999999998,
      "self_collision_robust_margin_m": 0.009999999999999998,
      "solve": {
        "allocated_bytes": 0,
        "allocation_calls": 0,
        "bitwise_repeat": true,
        "nominal_timing_us": {
          "jitter_p99_minus_p50": 6.131999999999996,
          "max": 28.464000000000002,
          "mean": 12.300276200000027,
          "p50": 11.892000000000001,
          "p95": 16.06,
          "p99": 18.023999999999997
        },
        "p50_delta_us": 0.010999999999997456,
        "p99_delta_us": -0.09099999999999753,
        "robust_timing_us": {
          "jitter_p99_minus_p50": 6.030000000000001,
          "max": 25.207,
          "mean": 12.323559200000053,
          "p50": 11.902999999999999,
          "p95": 16.121,
          "p99": 17.933
        }
      }
    },
    {
      "achieved_normal_acceleration_mps2": 3.0050250000000003,
      "error_bound": {
        "center_of_mass_position_error_m": 1.50625e-05,
        "joint_position_error_rad": 4.075e-05,
        "joint_velocity_error_rad_s": 0.003,
        "represented_point_position_error_m": 2.5125e-05,
        "root_rotation_error_rad": 2.0125e-05,
        "root_translation_error_m": 1.5100000000000001e-05
      },
      "error_bound_allocated_bytes": 0,
      "error_bound_allocation_calls": 0,
      "error_bound_ns_per_call": 55.19481,
      "exposure_ns": 500000,
      "generalized_acceleration_slide_rad_s2": 3.0050250000000003,
      "inside_live_prediction_horizon": true,
      "joint_stopping_interval_rad_s2": [
        -200.0,
        12.245004190952624
      ],
      "joint_stopping_upper_erosion_rad_s2": 1.057175814127298,
      "required_normal_acceleration_mps2": 3.0050250000000003,
      "self_collision_margin_erosion_m": 5.024999999999995e-05,
      "self_collision_raw_margin_m": 0.009999999999999997,
      "self_collision_robust_margin_m": 0.009949749999999997,
      "solve": {
        "allocated_bytes": 0,
        "allocation_calls": 0,
        "bitwise_repeat": true,
        "nominal_timing_us": {
          "jitter_p99_minus_p50": 6.2719999999999985,
          "max": 48.422,
          "mean": 12.62978539999989,
          "p50": 12.032,
          "p95": 16.752000000000002,
          "p99": 18.304
        },
        "p50_delta_us": 0.030999999999998806,
        "p99_delta_us": -0.17999999999999616,
        "robust_timing_us": {
          "jitter_p99_minus_p50": 6.0610000000000035,
          "max": 30.307,
          "mean": 12.627905599999986,
          "p50": 12.062999999999999,
          "p95": 16.712,
          "p99": 18.124000000000002
        }
      }
    },
    {
      "achieved_normal_acceleration_mps2": 3.0101000000000004,
      "error_bound": {
        "center_of_mass_position_error_m": 3.025e-05,
        "joint_position_error_rad": 8.300000000000001e-05,
        "joint_velocity_error_rad_s": 0.006,
        "represented_point_position_error_m": 5.05e-05,
        "root_rotation_error_rad": 4.05e-05,
        "root_translation_error_m": 3.04e-05
      },
      "error_bound_allocated_bytes": 0,
      "error_bound_allocation_calls": 0,
      "error_bound_ns_per_call": 52.964745,
      "exposure_ns": 1000000,
      "generalized_acceleration_slide_rad_s2": 3.0101000000000004,
      "inside_live_prediction_horizon": true,
      "joint_stopping_interval_rad_s2": [
        -200.0,
        11.170690020300889
      ],
      "joint_stopping_upper_erosion_rad_s2": 2.1314899847790336,
      "required_normal_acceleration_mps2": 3.0101000000000004,
      "self_collision_margin_erosion_m": 0.0001010000000000004,
      "self_collision_raw_margin_m": 0.009999999999999998,
      "self_collision_robust_margin_m": 0.009898999999999998,
      "solve": {
        "allocated_bytes": 0,
        "allocation_calls": 0,
        "bitwise_repeat": true,
        "nominal_timing_us": {
          "jitter_p99_minus_p50": 4.959999999999999,
          "max": 29.296,
          "mean": 12.052967600000132,
          "p50": 11.912,
          "p95": 12.153,
          "p99": 16.872
        },
        "p50_delta_us": 0.02999999999999936,
        "p99_delta_us": 0.10999999999999943,
        "robust_timing_us": {
          "jitter_p99_minus_p50": 5.039999999999999,
          "max": 37.521,
          "mean": 12.089812599999938,
          "p50": 11.942,
          "p95": 12.163,
          "p99": 16.982
        }
      }
    },
    {
      "achieved_normal_acceleration_mps2": 3.0256250000000002,
      "error_bound": {
        "center_of_mass_position_error_m": 7.65625e-05,
        "joint_position_error_rad": 0.00021875,
        "joint_velocity_error_rad_s": 0.015,
        "represented_point_position_error_m": 0.000128125,
        "root_rotation_error_rad": 0.000103125,
        "root_translation_error_m": 7.75e-05
      },
      "error_bound_allocated_bytes": 0,
      "error_bound_allocation_calls": 0,
      "error_bound_ns_per_call": 52.57035,
      "exposure_ns": 2500000,
      "generalized_acceleration_slide_rad_s2": 3.0256250000000002,
      "inside_live_prediction_horizon": true,
      "joint_stopping_interval_rad_s2": [
        -200.0,
        7.84456810191605
      ],
      "joint_stopping_upper_erosion_rad_s2": 5.457611903163873,
      "required_normal_acceleration_mps2": 3.0256250000000002,
      "self_collision_margin_erosion_m": 0.00025624999999999953,
      "self_collision_raw_margin_m": 0.009999999999999998,
      "self_collision_robust_margin_m": 0.009743749999999999,
      "solve": {
        "allocated_bytes": 0,
        "allocation_calls": 0,
        "bitwise_repeat": true,
        "nominal_timing_us": {
          "jitter_p99_minus_p50": 5.158999999999997,
          "max": 185.41,
          "mean": 12.066354399999982,
          "p50": 11.813,
          "p95": 12.062999999999999,
          "p99": 16.971999999999998
        },
        "p50_delta_us": 0.038999999999997925,
        "p99_delta_us": 0.14000000000000412,
        "robust_timing_us": {
          "jitter_p99_minus_p50": 5.260000000000003,
          "max": 27.763,
          "mean": 12.063801400000102,
          "p50": 11.851999999999999,
          "p95": 12.083,
          "p99": 17.112000000000002
        }
      }
    },
    {
      "achieved_normal_acceleration_mps2": 3.0525,
      "error_bound": {
        "center_of_mass_position_error_m": 0.00015624999999999998,
        "joint_position_error_rad": 0.000475,
        "joint_velocity_error_rad_s": 0.03,
        "represented_point_position_error_m": 0.0002625,
        "root_rotation_error_rad": 0.00021250000000000002,
        "root_translation_error_m": 0.00015999999999999999
      },
      "error_bound_allocated_bytes": 0,
      "error_bound_allocation_calls": 0,
      "error_bound_ns_per_call": 53.12174,
      "exposure_ns": 5000000,
      "generalized_acceleration_slide_rad_s2": 3.0525,
      "inside_live_prediction_horizon": true,
      "joint_stopping_interval_rad_s2": [
        -200.0,
        1.9548008171139664
      ],
      "joint_stopping_upper_erosion_rad_s2": 11.347379187965956,
      "required_normal_acceleration_mps2": 3.0525,
      "self_collision_margin_erosion_m": 0.0005249999999999994,
      "self_collision_raw_margin_m": 0.009999999999999997,
      "self_collision_robust_margin_m": 0.009474999999999997,
      "solve": {
        "allocated_bytes": 0,
        "allocation_calls": 0,
        "bitwise_repeat": true,
        "nominal_timing_us": {
          "jitter_p99_minus_p50": 5.510000000000002,
          "max": 48.101,
          "mean": 12.063130600000022,
          "p50": 11.823,
          "p95": 12.043,
          "p99": 17.333000000000002
        },
        "p50_delta_us": 0.02999999999999936,
        "p99_delta_us": 0.03999999999999915,
        "robust_timing_us": {
          "jitter_p99_minus_p50": 5.520000000000001,
          "max": 31.219,
          "mean": 12.08625160000002,
          "p50": 11.853,
          "p95": 12.093,
          "p99": 17.373
        }
      }
    },
    {
      "achieved_normal_acceleration_mps2": 3.0806250000000004,
      "error_bound": {
        "center_of_mass_position_error_m": 0.00023906250000000002,
        "joint_position_error_rad": 0.0007687500000000001,
        "joint_velocity_error_rad_s": 0.045000000000000005,
        "represented_point_position_error_m": 0.00040312500000000005,
        "root_rotation_error_rad": 0.000328125,
        "root_translation_error_m": 0.0002475
      },
      "error_bound_allocated_bytes": 0,
      "error_bound_allocation_calls": 0,
      "error_bound_ns_per_call": 53.72047,
      "exposure_ns": 7500000,
      "generalized_acceleration_slide_rad_s2": 3.0806250000000004,
      "inside_live_prediction_horizon": false,
      "joint_stopping_interval_rad_s2": [
        -200.0,
        -4.372438801887846
      ],
      "joint_stopping_upper_erosion_rad_s2": 17.674618806967768,
      "required_normal_acceleration_mps2": 3.0806250000000004,
      "self_collision_margin_erosion_m": 0.0008062499999999997,
      "self_collision_raw_margin_m": 0.009999999999999997,
      "self_collision_robust_margin_m": 0.009193749999999997,
      "solve": {
        "allocated_bytes": 0,
        "allocation_calls": 0,
        "bitwise_repeat": true,
        "nominal_timing_us": {
          "jitter_p99_minus_p50": 4.948,
          "max": 33.413,
          "mean": 12.000098999999755,
          "p50": 11.823,
          "p95": 12.013,
          "p99": 16.771
        },
        "p50_delta_us": 0.0389999999999997,
        "p99_delta_us": 0.20099999999999696,
        "robust_timing_us": {
          "jitter_p99_minus_p50": 5.109999999999998,
          "max": 22.201999999999998,
          "mean": 12.02145880000004,
          "p50": 11.862,
          "p95": 12.052000000000001,
          "p99": 16.971999999999998
        }
      }
    },
    {
      "achieved_normal_acceleration_mps2": 3.1100000000000003,
      "error_bound": {
        "center_of_mass_position_error_m": 0.000325,
        "joint_position_error_rad": 0.0011,
        "joint_velocity_error_rad_s": 0.06,
        "represented_point_position_error_m": 0.00055,
        "root_rotation_error_rad": 0.00045000000000000004,
        "root_translation_error_m": 0.00033999999999999997
      },
      "error_bound_allocated_bytes": 0,
      "error_bound_allocation_calls": 0,
      "error_bound_ns_per_call": 52.706605,
      "exposure_ns": 10000000,
      "generalized_acceleration_slide_rad_s2": 3.1100000000000003,
      "inside_live_prediction_horizon": false,
      "joint_stopping_interval_rad_s2": [
        -200.0,
        -11.143381282591047
      ],
      "joint_stopping_upper_erosion_rad_s2": 24.44556128767097,
      "required_normal_acceleration_mps2": 3.1100000000000003,
      "self_collision_margin_erosion_m": 0.0011000000000000003,
      "self_collision_raw_margin_m": 0.009999999999999998,
      "self_collision_robust_margin_m": 0.008899999999999998,
      "solve": {
        "allocated_bytes": 0,
        "allocation_calls": 0,
        "bitwise_repeat": true,
        "nominal_timing_us": {
          "jitter_p99_minus_p50": 5.02,
          "max": 57.96,
          "mean": 11.980394999999916,
          "p50": 11.822,
          "p95": 11.933000000000002,
          "p99": 16.842
        },
        "p50_delta_us": 0.040000000000000924,
        "p99_delta_us": -0.029999999999997584,
        "robust_timing_us": {
          "jitter_p99_minus_p50": 4.950000000000001,
          "max": 28.462999999999997,
          "mean": 12.010600000000068,
          "p50": 11.862,
          "p95": 12.013,
          "p99": 16.812
        }
      }
    }
  ]
}
```

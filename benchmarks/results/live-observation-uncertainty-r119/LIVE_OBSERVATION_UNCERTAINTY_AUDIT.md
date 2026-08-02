# Bonesaw live observation uncertainty authority · r119

**PASS.** A caller-authored deterministic error-growth envelope now propagates
from canonical reconstruction into the actual Rust WBC hard rows. The eval uses
the real WebSocket/WBC path and guided state without a policy or physics rollout.
Raw geometric/state margins remain visible beside robust margins; no aggregate
health score or probabilistic confidence claim is introduced.

| mode | exposure ms | q / v error (mrad / rad/s) | point / CoM error (mm) | self / world / support erosion (mm) | stopping erosion rad/s² | selection |
|---|---:|---:|---:|---:|---:|---|
| exact | 0.000 | 0.000000 / 0.000000 | 0.000000 / 0.000000 | 0.000000 / 0.000000 / 0.000000 | 0.000000–0.000000 | `{"contingency": 0, "primary": 20, "rejected": 0}` |
| interpolated | 2.500 | 0.218750 / 0.015000 | 0.128125 / 0.076562 | 0.256250 / 0.128125 / 0.076562 | 0.000000–0.000000 | `{"contingency": 5, "primary": 15, "rejected": 0}` |
| predicted | 5.000 | 0.475000 / 0.030000 | 0.262500 / 0.156250 | 0.525000 / 0.262500 / 0.156250 | 0.000000–0.000000 | `{"contingency": 8, "primary": 12, "rejected": 0}` |

## Propagation contract

- Joint stopping intersects the safe acceleration interval over all four
  corners of the `q ± error`, `v ± error` box.
- Finite-support rows add the CoM-position error radius to the authored erosion.
- Self-collision rows add two represented-point radii; world-SDF rows add one.
- Exact reconstruction has zero configured exposure and bit-identical raw and
  robust margins. Interpolation uses distance to the nearest bracket endpoint.
  Prediction grows monotonically from the newest source sample.
- Held or stale evidence remains ineligible and never reaches these hard rows.

## Retained audit

```json
{
  "contract_source": "bonesaw-tools/flat-foot-editor-r119",
  "execution": "guided_preview_without_policy_or_physics_rollout",
  "modes": {
    "exact": {
      "command_admission": {
        "maximum_us": 2006.641,
        "p50_us": 1435.2459999999999,
        "p99_us": 2006.641
      },
      "error_bound": {
        "center_of_mass_position_m": 0.0,
        "joint_position_rad": 0.0,
        "joint_velocity_rad_s": 0.0,
        "point_position_m": 0.0,
        "root_rotation_rad": 0.0,
        "root_translation_m": 0.0
      },
      "exposure_ns": 0,
      "frames": 20,
      "margin_erosion": {
        "joint_position_rad": {
          "maximum": 0.0,
          "minimum": 0.0
        },
        "joint_stopping_rad_s2": {
          "maximum": 0.0,
          "minimum": 0.0
        },
        "self_collision_m": {
          "maximum": 0.0,
          "minimum": 0.0
        },
        "support_m": {
          "maximum": 0.0,
          "minimum": 0.0
        },
        "world_collision_m": {
          "maximum": 0.0,
          "minimum": 0.0
        }
      },
      "minimum_robust_margins": {
        "joint_position_rad": 0.25,
        "joint_stopping_rad_s2": 196.5901593652091,
        "self_collision_m": 0.030000000000000002,
        "support_m": 0.02,
        "world_collision_m": 0.04640933182979967
      },
      "provenance": "exact",
      "selection_counts": {
        "contingency": 0,
        "primary": 20,
        "rejected": 0
      },
      "solve": {
        "maximum_us": 2828.503,
        "p50_us": 1932.3855000000003,
        "p99_us": 2828.503
      }
    },
    "interpolated": {
      "command_admission": {
        "maximum_us": 1571.228,
        "p50_us": 1425.7275,
        "p99_us": 1571.228
      },
      "error_bound": {
        "center_of_mass_position_m": 7.65625e-05,
        "joint_position_rad": 0.00021875,
        "joint_velocity_rad_s": 0.015,
        "point_position_m": 0.000128125,
        "root_rotation_rad": 0.000103125,
        "root_translation_m": 7.75e-05
      },
      "exposure_ns": 2500000,
      "frames": 20,
      "margin_erosion": {
        "joint_position_rad": {
          "maximum": 0.00021874999999998979,
          "minimum": 0.00021874999999998979
        },
        "joint_stopping_rad_s2": {
          "maximum": 0.0,
          "minimum": 0.0
        },
        "self_collision_m": {
          "maximum": 0.00025624999999999953,
          "minimum": 0.00025624999999999953
        },
        "support_m": {
          "maximum": 7.656250000000198e-05,
          "minimum": 7.65624999999985e-05
        },
        "world_collision_m": {
          "maximum": 0.00012812499999999977,
          "minimum": 0.00012812499999999977
        }
      },
      "minimum_robust_margins": {
        "joint_position_rad": 0.24978125,
        "joint_stopping_rad_s2": 195.63803446342632,
        "self_collision_m": 0.029743750000000003,
        "support_m": 0.0308286788806807,
        "world_collision_m": 0.04628120682979968
      },
      "provenance": "interpolated",
      "selection_counts": {
        "contingency": 5,
        "primary": 15,
        "rejected": 0
      },
      "solve": {
        "maximum_us": 2167.334,
        "p50_us": 1725.509,
        "p99_us": 2167.334
      }
    },
    "predicted": {
      "command_admission": {
        "maximum_us": 2009.4650000000001,
        "p50_us": 1433.6485,
        "p99_us": 2009.4650000000001
      },
      "error_bound": {
        "center_of_mass_position_m": 0.00015624999999999998,
        "joint_position_rad": 0.000475,
        "joint_velocity_rad_s": 0.03,
        "point_position_m": 0.0002625,
        "root_rotation_rad": 0.00021250000000000002,
        "root_translation_m": 0.00015999999999999999
      },
      "exposure_ns": 5000000,
      "frames": 20,
      "margin_erosion": {
        "joint_position_rad": {
          "maximum": 0.0004750000000000032,
          "minimum": 0.0004750000000000032
        },
        "joint_stopping_rad_s2": {
          "maximum": 0.0,
          "minimum": 0.0
        },
        "self_collision_m": {
          "maximum": 0.0005250000000000012,
          "minimum": 0.0005250000000000012
        },
        "support_m": {
          "maximum": 0.00015624999999999667,
          "minimum": 0.00015624999999999667
        },
        "world_collision_m": {
          "maximum": 0.00026249999999999885,
          "minimum": 0.00026249999999999885
        }
      },
      "minimum_robust_margins": {
        "joint_position_rad": 0.249525,
        "joint_stopping_rad_s2": 194.22053886234897,
        "self_collision_m": 0.030000000000000002,
        "support_m": 0.03378303567893175,
        "world_collision_m": 0.04640933182979967
      },
      "provenance": "predicted",
      "selection_counts": {
        "contingency": 8,
        "primary": 12,
        "rejected": 0
      },
      "solve": {
        "maximum_us": 3188.6529999999993,
        "p50_us": 1925.5209999999997,
        "p99_us": 3188.6529999999993
      }
    }
  },
  "program_fingerprint": "04780b6be2bc0784fff0d24badd805fa8da4ffd226f1ed7f382c392ea8192b54",
  "recovery_tick": 64,
  "revision": "live-observation-uncertainty-r119",
  "schema": 1,
  "stale_withheld": {
    "newest_sample_time_ns": 1235000000,
    "query_time_ns": 1245000000,
    "reason": "robot observation reconstruction withheld WBC input: robot observation prediction is disabled or exceeds its horizon",
    "tick": 62,
    "transport_mode": "stale",
    "type": "observation_withheld"
  },
  "status": "pass",
  "url": "ws://127.0.0.1:8819/ws"
}
```

The configured growth numbers are conservative editor contracts, not calibrated
estimator covariance, confidence intervals, network statistics, or hardware
safety certification. Command-trajectory collision admission retains its
separate root-prediction/error witness and is not relabeled as plant response.

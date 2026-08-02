# Bonesaw command reconstruction-uncertainty authority · r121

**PASS.** Canonical reconstruction uncertainty now reaches the independently
validated Primary and braking command trajectories. This live Python eval uses
the production Rust WebSocket boundary with guided state-local queries and no
policy, rigid-body physics, or plant integration.

| mode | exposure ms | point radius mm | self sampled / continuous loss mm | world sampled / continuous loss mm | root translation / rotation radius (mm / mrad) | selection |
|---|---:|---:|---:|---:|---:|---|
| exact | 0.000 | 0.000000 | 0.000000 / 0.000000 | 0.000000 / 0.000000 | 0.900000 / 0.680000 | `{"contingency": 0, "primary": 20, "rejected": 0}` |
| interpolated | 2.500 | 0.128125 | 0.256250 / 0.256250 | 0.128125 / 0.128125 | 0.977500 / 0.783125 | `{"contingency": 5, "primary": 15, "rejected": 0}` |
| predicted | 5.000 | 0.262500 | 0.525000 / 0.525000 | 0.262500 / 0.262500 | 1.060000 / 0.892500 | `{"contingency": 8, "primary": 12, "rejected": 0}` |

## Admission contract

- Self-collision consumes two body-local represented-point radii for both the
  sampled grid and bounded between-sample certificate.
- World collision carries root translation/rotation reconstruction error in the
  root-prediction initial radius, then consumes one body-local point radius
  through the immutable field Lipschitz bound.
- Raw command geometry, robust command geometry, tracking mismatch, scene
  validity, solver status, and final Primary/brake/reject selection remain
  distinct witnesses in the example authority stack.
- Exact reconstruction is the zero-error compatibility case. Held, stale, or
  horizon-invalid reconstruction is withheld before command admission.

## Retained audit

```json
{
  "contract_source": "bonesaw-tools/flat-foot-editor-r121",
  "execution": "guided_state_local_queries_without_policy_physics_or_plant_integration",
  "modes": {
    "exact": {
      "command_admission": {
        "maximum_us": 2359.037,
        "p50_us": 1762.7655,
        "p99_us": 2359.037
      },
      "command_margin_erosion": {
        "brake_self_continuous_m": {
          "maximum": 0.0,
          "minimum": 0.0
        },
        "brake_self_sampled_m": {
          "maximum": 0.0,
          "minimum": 0.0
        },
        "brake_world_continuous_m": {
          "maximum": 0.0,
          "minimum": 0.0
        },
        "brake_world_sampled_m": {
          "maximum": 0.0,
          "minimum": 0.0
        },
        "self_continuous_m": {
          "maximum": 0.0,
          "minimum": 0.0
        },
        "self_sampled_m": {
          "maximum": 0.0,
          "minimum": 0.0
        },
        "world_continuous_m": {
          "maximum": 0.0,
          "minimum": 0.0
        },
        "world_sampled_m": {
          "maximum": 0.0,
          "minimum": 0.0
        }
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
      "minimum_robust_command_margin": {
        "brake_self_continuous_m": 0.04981292249813626,
        "brake_self_sampled_m": 0.049999999999986805,
        "brake_world_continuous_m": 0.06460156978179495,
        "brake_world_sampled_m": 0.06466091801905668,
        "self_continuous_m": 0.049796829666802135,
        "self_sampled_m": 0.04999999999998564,
        "world_continuous_m": 0.0646015694333295,
        "world_sampled_m": 0.06466091786485131
      },
      "provenance": "exact",
      "raw_wbc": {
        "maximum_us": 3159.7810000000004,
        "p50_us": 2280.5935000000004,
        "p99_us": 3159.7810000000004
      },
      "root_prediction_error": {
        "maximum_world_clearance_erosion_m": 0.0017484129840778632,
        "rotation_radius_rad": 0.0006799999999999999,
        "translation_radius_m": 0.0009000000000000001
      },
      "selection_counts": {
        "contingency": 0,
        "primary": 20,
        "rejected": 0
      },
      "tracking_action_counts": {
        "contingency": 0,
        "nominal": 20,
        "rejected": 0
      }
    },
    "interpolated": {
      "command_admission": {
        "maximum_us": 2138.029,
        "p50_us": 1648.6605,
        "p99_us": 2138.029
      },
      "command_margin_erosion": {
        "brake_self_continuous_m": {
          "maximum": 0.00025624999999999953,
          "minimum": 0.00025624999999999953
        },
        "brake_self_sampled_m": {
          "maximum": 0.00025624999999999953,
          "minimum": 0.00025624999999999953
        },
        "brake_world_continuous_m": {
          "maximum": 0.0001281250000000067,
          "minimum": 0.0001281250000000067
        },
        "brake_world_sampled_m": {
          "maximum": 0.0001281250000000067,
          "minimum": 0.0001281250000000067
        },
        "self_continuous_m": {
          "maximum": 0.00025624999999999953,
          "minimum": 0.00025624999999999953
        },
        "self_sampled_m": {
          "maximum": 0.00025624999999999953,
          "minimum": 0.00025624999999999953
        },
        "world_continuous_m": {
          "maximum": 0.0001281250000000067,
          "minimum": 0.0001281250000000067
        },
        "world_sampled_m": {
          "maximum": 0.0001281250000000067,
          "minimum": 0.0001281250000000067
        }
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
      "minimum_robust_command_margin": {
        "brake_self_continuous_m": 0.04587108806772232,
        "brake_self_sampled_m": 0.04587282894707226,
        "brake_world_continuous_m": 0.06907451484456399,
        "brake_world_sampled_m": 0.06916125017553813,
        "self_continuous_m": 0.04570786318726367,
        "self_sampled_m": 0.04580643909791171,
        "world_continuous_m": 0.06940864577464159,
        "world_sampled_m": 0.06949651247089468
      },
      "provenance": "interpolated",
      "raw_wbc": {
        "maximum_us": 2609.08,
        "p50_us": 2067.932,
        "p99_us": 2609.08
      },
      "root_prediction_error": {
        "maximum_world_clearance_erosion_m": 0.0019545785561117302,
        "rotation_radius_rad": 0.000783125,
        "translation_radius_m": 0.0009775
      },
      "selection_counts": {
        "contingency": 5,
        "primary": 15,
        "rejected": 0
      },
      "tracking_action_counts": {
        "contingency": 5,
        "nominal": 15,
        "rejected": 0
      }
    },
    "predicted": {
      "command_admission": {
        "maximum_us": 1733.7150000000001,
        "p50_us": 1460.0785,
        "p99_us": 1733.7150000000001
      },
      "command_margin_erosion": {
        "brake_self_continuous_m": {
          "maximum": 0.0005249999999999977,
          "minimum": 0.0005249999999999977
        },
        "brake_self_sampled_m": {
          "maximum": 0.0005249999999999977,
          "minimum": 0.0005249999999999977
        },
        "brake_world_continuous_m": {
          "maximum": 0.00026249999999999885,
          "minimum": 0.00026249999999999885
        },
        "brake_world_sampled_m": {
          "maximum": 0.00026249999999999885,
          "minimum": 0.00026249999999999885
        },
        "self_continuous_m": {
          "maximum": 0.0005249999999999977,
          "minimum": 0.0005249999999999977
        },
        "self_sampled_m": {
          "maximum": 0.0005249999999999977,
          "minimum": 0.0005249999999999977
        },
        "world_continuous_m": {
          "maximum": 0.00026249999999999885,
          "minimum": 0.00026249999999999885
        },
        "world_sampled_m": {
          "maximum": 0.00026249999999999885,
          "minimum": 0.00026249999999999885
        }
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
      "minimum_robust_command_margin": {
        "brake_self_continuous_m": 0.04481900970508152,
        "brake_self_sampled_m": 0.044819031189443956,
        "brake_world_continuous_m": 0.08755461661191667,
        "brake_world_sampled_m": 0.08759398500852708,
        "self_continuous_m": 0.044815293445471936,
        "self_sampled_m": 0.044818927819610554,
        "world_continuous_m": 0.08756897204678675,
        "world_sampled_m": 0.08761023982807414
      },
      "provenance": "predicted",
      "raw_wbc": {
        "maximum_us": 2552.112,
        "p50_us": 2018.042,
        "p99_us": 2552.112
      },
      "root_prediction_error": {
        "maximum_world_clearance_erosion_m": 0.002173542041602195,
        "rotation_radius_rad": 0.0008924999999999998,
        "translation_radius_m": 0.00106
      },
      "selection_counts": {
        "contingency": 8,
        "primary": 12,
        "rejected": 0
      },
      "tracking_action_counts": {
        "contingency": 8,
        "nominal": 12,
        "rejected": 0
      }
    }
  },
  "program_fingerprint": "04780b6be2bc0784fff0d24badd805fa8da4ffd226f1ed7f382c392ea8192b54",
  "recovery_tick": 65,
  "revision": "live-command-observation-uncertainty-r121",
  "schema": 1,
  "stale_withheld": {
    "newest_sample_time_ns": 1250000000,
    "query_time_ns": 1260000000,
    "reason": "robot observation reconstruction withheld WBC input: robot observation prediction is disabled or exceeds its horizon",
    "tick": 63,
    "transport_mode": "stale",
    "type": "observation_withheld"
  },
  "status": "pass",
  "url": "ws://127.0.0.1:8819/ws"
}
```

These caller-authored deterministic radii are not estimator covariance,
probability, calibrated hardware safety, or evidence that the plant realizes an
admitted command.

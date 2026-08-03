# Upkie scalar reference audit (R310)

**Measured:** 2026-08-03 on the local Ryzen 7 3700X host  
**Bonesaw source:** `9a217def1a23254132e073ee7f7a40d7b6e6e25c`  
**Status:** scalar reference and rigid-body products pass; full-body Upkie WBC
reference parity remains open.

This audit records the highest-value reference checks that are available without
changing the live MuJoCo host. It deliberately separates three boundaries:

1. the pinned upstream Upkie `WheelBalancer.cpp` law versus the typed Rust law;
2. independent Pinocchio 4.0 rigid-body products for the same URDFs; and
3. the still-missing independent full-body Upkie WBC closed-loop reference.

The first two are reference evidence. They are not evidence that a scalar wheel
velocity law and Bonesaw's floating inverse-dynamics WBC produce equivalent
torques, contacts, or physical trajectories.

## Reproduction

The fresh 20,000-sample scalar run used the cached upstream checkout and the
already-built release worker:

```bash
python3 python/evals/upkie_controller_comparison.py \
  --ticks 20000 \
  --output /tmp/bonesaw-reference-audit \
  --upkie-worker /tmp/bonesaw-upkie-reference/bazel-bin/bonesaw_oracle/upkie_wheel_balancer_oracle \
  --upkie-source /tmp/bonesaw-upkie-reference
```

The independent product checks used Pinocchio 4.0 in its isolated environment:

```bash
/tmp/bonesaw-pinocchio/bin/python scripts/pinocchio-oracle.py \
  --model models/upkie/upkie.urdf \
  --fixture-bin target/release/bonesaw-oracle-fixture \
  --samples 50 \
  --output-json /tmp/bonesaw-reference-audit/pinocchio-upkie-r310.json

/tmp/bonesaw-pinocchio/bin/python scripts/pinocchio-oracle.py \
  --model benchmarks/cache/unitree-g1/g1_23dof_mode_10.urdf \
  --fixture-bin target/release/bonesaw-oracle-fixture \
  --samples 50 \
  --output-json /tmp/bonesaw-reference-audit/pinocchio-g1-r310.json
```

The scalar worker's complete raw traces and metrics were retained in the
temporary directory above. The tracked `metrics.json` beside this report is a
compact extraction; the full source report is reproducible with the command.

## Scalar WheelBalancer result

The corpus contains 20,000 sequential 5 ms samples over zero hold, smooth
coupled motion, bounded steps, and seeded colored noise. The official-aligned
Rust profile uses the upstream gains, limits, wheel radius, update order, and
timestep. The live profile is intentionally tuned for the Bonesaw integrated
path.

| Worker | Command parity | p50 / p99 / p99.9 / max (µs) | absolute adjacent jitter p99 (µs) | peak RSS | CPU / wall | hot-loop allocations |
|---|---|---:|---:|---:|---:|---:|
| Upkie C++ `read`/`write` | reference | 1.383 / 2.425 / 4.729 / 17.102 | 0.061 | 5.01 MiB | 0.996 | n/a |
| Bonesaw Rust, official gains | **0 canonical mismatches** | 0.030 / 0.031 / 0.031 / 0.150 | 0.011 | 4.18 MiB | 0.830 | 0 calls / 0 bytes |
| Bonesaw Rust, live gains | intentional policy delta | 0.030 / 0.031 / 0.031 / 0.702 | 0.011 | 4.33 MiB | 0.978 | 0 calls / 0 bytes |

The official command comparison is exact for both wheels: 0 mismatches over
20,000 samples. The live profile differs by design (RMS `0.0931499545 m/s`,
maximum `0.3772297891 m/s`, correlation `0.9949896622`). Its regional RMS/max
deltas are `0.141946/0.377230 m/s` for bounded steps,
`0.091049/0.194910 m/s` for smooth coupled motion, and
`0.079178/0.206668 m/s` for colored noise; zero hold is bitwise exact.

The C++ timing includes the upstream dictionary read/write adapter and gain
writes. The Rust timing is only the typed law call. Therefore the ~46x p50 and
~78x p99 scalar timing ratios are boundary-cost measurements, not a whole-body
controller speedup.

## Pinocchio product result

| Model | Samples / frame samples | Gate | Maximum floating bias error | Maximum floating inverse-dynamics error |
|---|---:|---|---:|---:|
| Upkie | 50 / 2,050 | PASS | `2.132e-14` | `2.132e-14` |
| Unitree G1 mode-10 | 50 / 1,500 | PASS | `1.705e-13` | `1.705e-13` |

All fixed/floating frame, rotation, Jacobian, CoM, mass, gravity, centroidal,
and inverse-dynamics thresholds pass at roundoff. Pinocchio remains a product
oracle, not a controller or a physical trajectory reference.

## Reference boundary still open

The repository currently has no independent full-body Upkie WBC that consumes
the same measured MuJoCo state and emits comparable joint torques/contact
forces. PlaCo is a useful independent fixed-base toy-humanoid behavior
comparator; the upstream WheelBalancer is a scalar rolling law. Neither is a
valid full-body Upkie trajectory reference. R309/R312 MuJoCo traces are
candidate-versus-baseline Bonesaw experiments, not external-reference parity.

The next high-value comparison should freeze one shared Upkie physical corpus
(250 Hz MuJoCo observations, 50 Hz commands, disturbances, contact labels,
targets, and reset/reconnect semantics) and run an independent full-body
controller through it. PlaCo/Pink or an independently authored constrained
Pinocchio QP are plausible references. That experiment should retain tracking
error, contact/support outcomes, authority/limit outcomes, per-step latency and
jitter, RSS/CPU, and execution-over-time windows. Until then, this report keeps
the scalar parity claim narrow rather than ranking the scalar law against the
floating WBC.

## Provenance and checks

- Upkie checkout: commit `1abdf373bbe8ee7d9f081293024f961ae9a8e2c1`.
- `WheelBalancer.cpp` SHA-256:
  `4b65d9356f9e6f925b9470b587133f96d4119b0ef774b220e9c39bcd945ca984`.
- `WheelBalancer.h` SHA-256:
  `0028aa465c91feae2b8a9b713fa09365ab02aecdc32f4bbe8754a9fcdce6d20e`.
- Upkie URDF SHA-256:
  `d15965215067276203599a850e5c7ebb319ed6815506f0a2721dae78abac2483`.
- Scalar corpus SHA-256:
  `57608e4a4ed6bc40215196f2a624995180596c0dc5ac7483cd95d6b076d89416`.
- Scalar raw NPZ SHA-256:
  `71872ea26afa35dc75f8dfb29f163975bddff402ff3208f2cdcd7772081fa519`.
- Full source metrics JSON SHA-256 (temporary run artifact):
  `236cfbcb4f9cf32b880e8b234b8bb94c27b41a094b13ec4ccd317879b33d2e62`.
- Tracked compact summary SHA-256: `d7ef7724b725dd2450d24467fb3f8f92395a63f72b944ab057cc79c399ce78dd`.
- Pinocchio environment: `pinocchio==4.0.0`, Python 3.12.3.
- Report renderer test: `PYTHONPATH=python/evals python3 -m unittest
  python/evals/test_cpu_reference_report.py` — 2 tests passed.
- No live server, tunnel, or MuJoCo worker was changed by this audit; all
  generated traces were written under `/tmp`.

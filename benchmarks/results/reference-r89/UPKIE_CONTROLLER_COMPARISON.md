# Bonesaw ↔ official Upkie wheel-controller comparison

This is a direct implementation-oracle test. The reference worker links the
pinned upstream `WheelBalancer.cpp` class unchanged; the Rust worker calls
Bonesaw's typed balance law. Python owns the shared corpus, process isolation,
statistics, artifacts, and this report. No Python code runs inside either
measured controller loop.

## Scope and interpretation

- Shared subset: continuous floor contact, zero requested yaw, stationary target
  ground position, and pitch below the upstream fall threshold.
- The official-aligned Rust profile uses Upkie's exact default gains, limits,
  wheel radius, update order, and 5 ms timestep. This is the parity gate.
- The live Rust profile uses the gains and 5 cm wheel radius selected by the
  integrated Bonesaw WBC corpus. Its delta is intentional and reported rather
  than hidden.
- Upkie latency includes its `palimpsest::Dictionary` read/write adapter and hip/
  knee gain writes. Rust latency is the typed law call. Use these numbers to
  understand boundary cost, not as a whole-robot WBC speed comparison.

## Provenance

| Item | Value |
|---|---|
| Upkie commit | `1abdf373bbe8ee7d9f081293024f961ae9a8e2c1` |
| WheelBalancer.cpp SHA-256 | `4b65d9356f9e6f925b9470b587133f96d4119b0ef774b220e9c39bcd945ca984` |
| WheelBalancer.h SHA-256 | `0028aa465c91feae2b8a9b713fa09365ab02aecdc32f4bbe8754a9fcdce6d20e` |
| Corpus | 100,000 sequential 5 ms samples · seed `0xB0E5A7` |
| Host | AMD Ryzen 7 3700X 8-Core Processor · Linux-6.18.7-76061807-generic-x86_64-with-glibc2.39 |

## Command equivalence

| Candidate | Canonical bitwise | mismatches | max abs command error | RMS error | relative L2 | correlation |
|---|---:|---:|---:|---:|---:|---:|
| Rust, official parameters | PASS | 0 | 0.000e+00 rad/s | 0.000e+00 rad/s | 0.000e+00 | 1.000000000 |
| Rust, live tuned parameters | intentionally different | — | 3.772e-01 m/s | 1.177e-01 m/s | 1.370e-01 | 0.995085451 |

The parity gate compares the two emitted wheel-velocity commands. Signed zero
is canonicalized; every nonzero value is compared by its exact IEEE-754 bytes.
The inferred upstream ground velocity (`left × radius`) can differ from the
Rust law's pre-division value by one rounding step, so it is retained as a
separate diagnostic rather than mislabelled as the actuator command.

## Latency and jitter

| Worker | mean µs | std µs | MAD µs | p50 µs | p95 µs | p99 µs | p99.9 µs | p99.99 µs | max µs | jitter p99 µs |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Official Upkie C++ read/write | 1.469 | 0.327 | 0.001 | 1.393 | 2.225 | 2.385 | 5.000 | 13.195 | 17.563 | 0.250 |
| Bonesaw Rust typed · official gains | 0.027 | 0.032 | 0.000 | 0.030 | 0.030 | 0.031 | 0.040 | 0.041 | 5.029 | 0.011 |
| Bonesaw Rust typed · live gains | 0.027 | 0.015 | 0.000 | 0.030 | 0.030 | 0.031 | 0.040 | 0.041 | 4.359 | 0.011 |

## Behavior by corpus region

| Region | samples | official command mismatches | live delta RMS m/s | live delta max m/s | Upkie p99 µs | Rust live p99 µs |
|---|---:|---:|---:|---:|---:|---:|
| bounded_steps | 25,000 | 0 | 0.1419 | 0.3772 | 1.403 | 0.040 |
| seeded_colored_noise | 25,000 | 0 | 0.1657 | 0.3324 | 1.733 | 0.031 |
| smooth_coupled | 25,000 | 0 | 0.0884 | 0.2007 | 2.475 | 0.031 |
| zero_hold | 25,000 | 0 | 0.0000 | 0.0000 | 2.405 | 0.031 |

## Temporal drift by execution window

| Window | samples | Upkie p50/p99 µs | Rust official p50/p99 µs | Rust live p50/p99 µs | live delta RMS/max m/s |
|---:|---:|---:|---:|---:|---:|
| 0 | 0–9,999 | 1.393/2.585 | 0.030/0.040 | 0.030/0.031 | 0.0000/0.0000 |
| 1 | 10,000–19,999 | 1.392/1.483 | 0.030/0.040 | 0.030/0.031 | 0.0000/0.0000 |
| 2 | 20,000–29,999 | 1.393/1.423 | 0.030/0.040 | 0.030/0.031 | 0.0644/0.1949 |
| 3 | 30,000–39,999 | 1.393/2.485 | 0.030/0.031 | 0.030/0.031 | 0.0879/0.2007 |
| 4 | 40,000–49,999 | 1.393/2.524 | 0.030/0.040 | 0.030/0.040 | 0.0877/0.1949 |
| 5 | 50,000–59,999 | 1.383/1.413 | 0.030/0.040 | 0.030/0.031 | 0.1419/0.3772 |
| 6 | 60,000–69,999 | 1.383/1.403 | 0.030/0.031 | 0.030/0.031 | 0.1419/0.3772 |
| 7 | 70,000–79,999 | 1.393/1.512 | 0.030/0.031 | 0.030/0.040 | 0.1946/0.3772 |
| 8 | 80,000–89,999 | 1.393/1.733 | 0.030/0.031 | 0.030/0.040 | 0.1786/0.3324 |
| 9 | 90,000–99,999 | 1.393/1.743 | 0.030/0.040 | 0.030/0.031 | 0.0945/0.2238 |

## Process resources

| Worker | wall s | CPU/wall | peak RSS | minor faults | major faults | voluntary ctx | involuntary ctx |
|---|---:|---:|---:|---:|---:|---:|---:|
| Official Upkie C++ | 0.3983 | 0.997 | 8.64 MiB | 1,832 | 0 | 1 | 4 |
| Bonesaw Rust · official gains | 0.0827 | 0.989 | 8.68 MiB | 1,538 | 0 | 1 | 0 |
| Bonesaw Rust · live gains | 0.0827 | 0.995 | 8.16 MiB | 1,536 | 0 | 1 | 1 |

## Allocation and artifacts

- Bonesaw official-aligned loop: **0 calls / 0 bytes**.
- Bonesaw live-tuned loop: **0 calls / 0 bytes**.
- `upkie-controller-corpus.tsv`: exact sequential input corpus.
- `upkie-controller-raw.npz`: both command traces, both Rust integral-state
  traces, region labels, and all per-call latency samples.
- `upkie-controller-metrics.json`: complete machine-readable metrics.

## What this proves—and what it does not

The official-aligned parity result proves that the shared PI controller law in
Bonesaw has the same sequential floating-point behavior as the pinned upstream
C++ implementation over this corpus. It does not claim that the surrounding
systems are identical: Bonesaw lowers wheel velocity into bounded acceleration
tasks and a lexicographic floating WBC, while Upkie writes servo velocity
offsets. Full-body model products remain covered independently by Pinocchio,
and the fixed-base task behavior remains compared with PlaCo.

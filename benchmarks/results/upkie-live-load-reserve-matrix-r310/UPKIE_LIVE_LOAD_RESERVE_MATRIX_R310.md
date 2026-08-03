# Upkie load-reserve matrix R310

Generated: 2026-08-03T08:53:51.374767+00:00

Status: **qualified for public default**

This policy-free A/B composes R309's support-preserving posture hierarchy with
a gentler Rust wheel-load reserve profile. The matrix holds model, public
50/250/50 cadence, limits, contact evidence, solver budget, and application
point fixed while crossing both lateral signs, four force magnitudes, and five
durations.

| Measure | Result |
|---|---:|
| matrix | 40 cases |
| baseline falls | 20 |
| candidate falls / flight cases | 0 / 0 |
| candidate contact-loss / recovered cases | 6 / 6 |
| maximum tilt | 0.183863 rad |
| minimum root height | 0.520974 m |
| maximum torque utilization | 0.267187 |
| maximum WBC p99 | 192.478 µs |
| maximum worker p99 | 2961.252 µs |
| maximum reserve-action p99 | 7.430 µs |
| hot allocations | 0 |

## Gates

- PASS — nominal completes bilateral
- PASS — nominal action is dormant
- PASS — matrix has baseline failures
- PASS — candidate completes every case
- PASS — candidate never enters flight
- PASS — every support loss reacquires
- PASS — every case has stable 2s tail
- PASS — zero nonadmission or numeric reset
- PASS — hard residual below 1e minus 8
- PASS — controller and worker p99 within deadline
- PASS — zero hot path allocations
- PASS — bounded tilt height and torque
- PASS — representative replay exact

## Architectural conclusion

A gentler continuous reserve profile composes with R309 posture authority: all 40 mirrored force-duration cases finish upright, transient single support never becomes flight, and every loss has a measured bilateral tail. The R308 landing layer remains default-off because it is unnecessary and non-monotonic here.

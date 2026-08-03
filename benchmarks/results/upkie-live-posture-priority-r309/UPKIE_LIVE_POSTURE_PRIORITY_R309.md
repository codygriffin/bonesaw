# Upkie posture priority R309

Generated: 2026-08-03T09:07:31.649457+00:00

Status: **qualified for default**

This A/B changes only the exact WBC hierarchy: the six-coordinate posture row
moves from priority 1 to priority 0 at weight 1.0. Gains, limits, model,
contact evidence, 50 Hz controller, 250 Hz MuJoCo plant, and 8 N wrench remain fixed.

| Measure | legacy nominal | candidate nominal | legacy disturbed | candidate disturbed |
|---|---:|---:|---:|---:|
| completed ticks | 82 | 300 | 57 | 300 |
| terminal | fall | None | fall | None |
| first contact loss | 61 | None | 32 | None |
| maximum tilt | 0.239262 | 0.013400 | 0.974199 | 0.046177 |
| minimum root height | 0.298825 | 0.520974 | 0.287250 | 0.520974 |
| peak torque utilization | 0.629580 | 0.152005 | 0.449056 | 0.152005 |
| controller p99 | 149.995 µs | 154.053 µs | 158.237 µs | 154.161 µs |
| worker p99 | 2261.717 µs | 2344.140 µs | 2407.760 µs | 2116.788 µs |
| controller jitter p99 | 22.804 µs | 16.977 µs | 52.440 µs | 13.523 µs |
| root tracking RMS | 0.034684 m | 0.001980 m | 0.049667 m | 0.003669 m |
| CoM tracking RMS | 0.026035 m | 0.001531 m | 0.028939 m | 0.002566 m |

## Qualification gates

- PASS — nominal completes 6s
- PASS — nominal remains bilateral
- PASS — disturbed completes 6s
- PASS — disturbed remains bilateral
- PASS — zero nonadmitted steps
- PASS — hard residual below 1e minus 8
- PASS — controller p99 below 5ms
- PASS — worker p99 below 20ms
- PASS — zero wbc allocations
- PASS — peak torque below 20pct
- PASS — semantic replay exact

## Architectural conclusion

Priority-1 leg posture is starved by the root/contact hierarchy and causes nominal hip/knee divergence. Moving the same bounded posture request to priority 0 removes nominal and disturbed contact loss without extra torque, allocation, or policy.

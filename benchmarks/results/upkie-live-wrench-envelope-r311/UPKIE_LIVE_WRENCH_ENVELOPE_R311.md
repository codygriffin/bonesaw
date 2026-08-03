# Upkie wrench capability envelope R311

Generated: 2026-08-03T09:14:34.520158+00:00

Status: **valid measured capability envelope**

This deterministic, policy-free matrix applies the same lateral force at the
base center of mass and 25 cm above/below it. A single 400 ms pull is compared
with four spaced pulls in one trajectory. Terminal falls are capability
outcomes; protocol rejection, numeric reset, missed scheduled loads, or hard
residual error would invalidate the benchmark.

| Measure | Result |
|---|---:|
| matrix | 48 cases |
| stable upright | 32 |
| completed outside upright envelope | 5 |
| terminal falls | 7 |
| completed with WBC nonadmission | 4 |
| maximum commanded moment | 2.000 N·m |
| maximum tilt among completed cases | 0.342129 rad |
| maximum WBC p99 | 201.396 µs |
| maximum worker p99 | 2802.342 µs |
| hot allocations | 0 |

## Harness validity gates

- PASS — complete matrix and both schedules
- PASS — declared moment matches force cross offset
- PASS — completed cases receive every scheduled tick
- PASS — upper high moment boundary is exposed
- PASS — lower high moment has stable examples
- PASS — terminal boundary replay is exact

## Controller quality gates

- PASS — centered repeated pulls finish with upright tail
- PASS — low moment envelope stays nonterminal
- FAIL — zero nonadmission or numeric reset
- PASS — admitted hard residual below 1e minus 8
- PASS — controller and worker p99 within deadline
- PASS — zero hot path allocations

## Architectural conclusion

The production CPU controller handles repeated centered pulls and the full tested <=1 N.m moment envelope, but upper-base pulls expose a repeatable 1.5 N.m-class terminal boundary. This is retained as a measured feasibility limit, not misreported as a solver or transport failure.

# Live application-point wrench audit · live-wrench-application-r132

Admission: PASS. Equal 2 N world-X forces are applied for three 20 ms stream frames at the base origin and at ±200 mm world-Z offsets. MuJoCo maps each point force through mj_applyFT; the worker separately streams the COM-relative moment and lever while Rust limits the point to 750 mm from the latest body origin.

The expected differential moment is Δτ = Δp × F = 0.4 m × 2 N = 0.8 N·m. This audit requires both that wrench identity and a signed, ordered physical pitch consequence; it does not infer application-point semantics from a force arrow.

## Repeated physical consequence

| trial | τy −200 mm N·m | τy center N·m | τy +200 mm N·m | pitch −200 mm rad | pitch center rad | pitch +200 mm rad | rate −200 mm rad/s | rate center rad/s | rate +200 mm rad/s |
|---|---|---|---|---|---|---|---|---|---|
| 1 | -0.400765 | 0.000289 | 0.401712 | -0.021522 | 0.030826 | 0.083378 | -0.734179 | 0.960618 | 2.576460 |
| 2 | -0.400765 | 0.000289 | 0.401712 | -0.021522 | 0.030826 | 0.083378 | -0.734179 | 0.960618 | 2.576460 |
| 3 | -0.400765 | 0.000289 | 0.401712 | -0.021522 | 0.030826 | 0.083378 | -0.734179 | 0.960618 | 2.576460 |
| 4 | -0.400765 | 0.000289 | 0.401712 | -0.021522 | 0.030826 | 0.083378 | -0.734179 | 0.960618 | 2.576460 |
| 5 | -0.400765 | 0.000289 | 0.401712 | -0.021522 | 0.030826 | 0.083378 | -0.734179 | 0.960618 | 2.576460 |

Each condition owns a fresh isolated MuJoCo and persistent Rust capture/WBC session. Timing remains descriptive and is gated separately from the physical ordering.

## Admission gates

| gate | observed | pass |
|---|---|---|
| streamed moment has the expected sign | low -0.400765; center max abs 0.000289; high 0.401712 N·m | True |
| point-force differential matches cross product | 0.802477–0.802477 N·m against 0.800000 N·m | True |
| signed pitch consequence is ordered | low -0.021522; center 0.030826; high 0.083378 rad | True |
| signed pitch-rate consequence is ordered | low -0.734179; center 0.960618; high 2.576460 rad/s | True |
| physical outputs repeat across fresh sessions | max pitch spread 0.000e+00 rad; moment spread 0.000e+00 N·m | True |
| all bounded conditions remain upright | 0 | True |
| excessive lever is rejected and cleared | True | True |
| controller maximum below 1 ms | 325.5 µs | True |
| four-tick worker maximum below 5 ms | 2071.0 µs | True |

## Ownership and limits

Rust owns finite schema validation, body existence/readiness, the 8 N force cap, the 750 mm body-origin point lease, TTL, correlation, and clearing a rejected push. Python owns MuJoCo topology and therefore independently checks the exact body-COM lever before calling mj_applyFT. The browser now displays force and measured moment separately.

This is one sagittal Upkie body, one force direction, three points, ideal observation, and 60 ms of forcing. It does not establish lateral recovery, arbitrary body surfaces, sustained/repeated wrench safety, friction or slope robustness, delay/noise tolerance, thermal limits, hardware behavior, or browser frame time.

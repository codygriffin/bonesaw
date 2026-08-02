# Live Upkie plant gateway audit · live-upkie-plant-gateway-r131

Admission: PASS. This is a repeated end-to-end transport and plant-response audit. Browser-protocol commands enter the Rust WebSocket server, which validates force and expiry, supervises a per-session Python MuJoCo worker, and streams measured transforms plus Rust capture/WBC evidence back. TARGET preview and PUSH physics remain separate sockets and command types.

PUSH drag → Rust validation/TTL → Python MuJoCo plant → observed q/v/root → Rust capture + WBC → torque → MuJoCo → plant_state.

## Repeated response

| trial | peak Δx m | peak tilt Δ° | capture | final tilt ° | final station mm | release ms | expiry ms | Rust p99 µs | worker p99 µs | stream p99 ms |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 0.1076 | 25.09 | 1.000 | 0.150 | -42.17 | 19.3 | 162.4 | 458.3 | 4235.5 | 40.9 |
| 2 | 0.1076 | 25.09 | 1.000 | 0.150 | -42.17 | 19.4 | 161.2 | 304.5 | 2854.1 | 40.9 |
| 3 | 0.1076 | 25.09 | 1.000 | 0.150 | -42.17 | 21.7 | 160.6 | 337.0 | 3110.6 | 40.8 |
| 4 | 0.1076 | 25.09 | 1.000 | 0.150 | -42.17 | 20.4 | 159.9 | 320.0 | 3117.1 | 40.9 |
| 5 | 0.1076 | 25.09 | 1.000 | 0.150 | -42.17 | 19.9 | 159.8 | 339.9 | 3056.3 | 41.5 |

Each trial starts a fresh isolated plant session, applies 4 N to the base for five 20 ms stream frames, releases it, observes 2.1 seconds of recovery, verifies one-shot force expiry, injects an over-limit command, proves the stream survives, and performs a correlated reset.

## Admission gates

| gate | observed | pass |
|---|---|---|
| typed ownership boundary | ['python_mujoco_plant__rust_capture_wbc'] | True |
| physical push is observable | min Δx 0.1076 m; min tilt 25.09 deg | True |
| capture pressure is exercised | 1.0 | True |
| release and one-shot expiry are fail-safe | max release 21.7 ms; expiry 159.8–162.4 ms | True |
| invalid command does not kill stream | True | True |
| tilt and odom station recover | max final tilt 0.150 deg; max absolute station error 42.17 mm | True |
| correlated reset restores seed | max root error 0.000e+00 m; max ack 40.6 ms | True |
| no numerical auto-reset | 0 | True |
| Rust controller p99 below 1 ms | 458.3 µs | True |
| four-control-tick worker p99 below 5 ms | 4235.5 µs | True |
| 50 Hz stream p99 below 60 ms | 41.5 ms | True |

## Deliberate limits

This admits one sagittal body-center force on one Upkie soft-contact plant with ideal state observation. It does not establish arbitrary application-point torque, lateral recovery, slopes/friction variation, delay/noise, multiple simultaneous clients sharing one physical robot, thermal/hardware safety, or browser frame time. The in-app browser was unavailable for this CLI run, so visual/touch/mobile acceptance remains separate from these protocol and plant gates.

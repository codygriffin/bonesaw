# Bonesaw Upkie live editor acceptance · r122

**PASS for HTTP/WebSocket identity, command acknowledgement, visible torso and
joint motion, typed limit saturation, reset acknowledgement, and release over
both local port 8777 and the current Cloudflare Quick Tunnel.**

| path | drag ack | torso displacement | knee displacement | snapshot p50 / p95 / max | solve sample |
|---|---:|---:|---:|---:|---:|
| local | 30.11 ms | 14.33 mm | 8.61 mm | 20.22 / 30.12 / 30.12 ms | 666.73 µs |
| Cloudflare | 102.77 ms | 14.33 mm | 8.61 mm | 18.91 / 66.33 / 66.33 ms | 613.04 µs |

The dependency-free Python client verifies the public HTML, WebSocket upgrade,
Upkie identity (6 DOF / 41 bodies), `control_world · odom · map`, torso target,
active-frame acknowledgement, visible returned-frame displacement, continuous
state delivery while a deliberately excessive target is clamped, explicit
`guided_preview_wbc_admitted=false`, monotonic reset epoch, a downstream knee
target after reset, and final release.

This is transport/controller evidence, not browser raster timing. The CLI
session has no in-app browser target, so the complaint's viewport draw p95 gate
is **NOT RUN**. Open the hosted URL with `?perf=1` to collect frame interval,
draw, geometry, snapshot, command RTT, telemetry, coalescing, and local
pointer-to-camera distributions in the actual browser.

Reproduce locally:

```bash
python3 python/evals/live_editor_smoke.py \
  --url http://127.0.0.1:8777 --exercise-drag
```

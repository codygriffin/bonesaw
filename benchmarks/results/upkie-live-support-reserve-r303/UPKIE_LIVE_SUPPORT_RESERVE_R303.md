# R303 live measured support-reserve witness

R303 is a causal telemetry gate for the unresolved R302 recovery complaint. It
does not issue a policy, wrench, or actuator command. Python evaluates the
measured wheel normal forces produced by the live five-substep MuJoCo window;
the future action boundary must still be implemented and admitted by Rust.

The fail-closed witness requires bilateral measured contact, two finite
nonnegative wheel loads, a weaker-wheel fraction at or below `0.45`, and a
strictly falling fraction from the preceding 50 Hz sample. On the frozen
8 N lateral base-COM wrench trace:

| quantity | tick |
| --- | ---: |
| reserve witness | 28 |
| witness loads (left/right) | 28.958 / 23.426 N |
| weaker-wheel load fraction | 0.4472 |
| first 250 Hz MuJoCo contact loss | 31 |
| first 50 Hz WBC-observed loss | 32 |
| lead before physics loss | 3 control ticks / 12 ms |

The undisturbed control trace has no false trigger. All force samples are
finite and nonnegative, and the witness never claims actuator authority. This
closes the measurement/localization seam: the first support failure is
visible in the force reserve before the binary contact mask arrives. It does
not close recovery, because no reserve action is yet promoted. The next slice
is a bounded Rust request generated from this witness, with ordinary WBC
admission, residual, timing, and physical recovery gates.

For comparison, the separate `support_load_guard_enabled` evaluation override
ramps a damping/stiffness guard from the same measured stream. In the recorded
run it moves the first physics loss to tick 32 and the fall boundary to tick
76 with zero Rust allocation, but the bounded solver reports `MaxIterations`
at ticks 61, 67, and 68 and never satisfies the strict recovery predicate. Host
timing is also noisy (controller p99 was 5.319 ms in this run), so this probe
is not a production latency claim. It remains default-off, evaluation-only,
and unpromoted; it is evidence that reserve timing helps localize the failure,
not a recovery claim.

Reproduce the exact gate with:

```bash
PYTHONPATH=python:python/evals \
  /tmp/bonesaw-mujoco/bin/python \
  python/evals/upkie_live_support_reserve_r303.py --maximum-ticks 80
```

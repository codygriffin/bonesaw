# Bonesaw Upkie state-local task authority · r135

> No policy, state rollout, simulator, or physics engine participates. Python authors 196 immutable requests; one persistent Rust session emits model products, rolling/contact rows, rigid-body dynamics, strict hierarchy, bounded effort, and diagnostics.

## Outcome

The probe makes the lateral limitation explicit instead of blaming a late solver status. At the nominal two-wheel state, a ±250 rad/s² roll request is clipped to a small feasible response while pitch and yaw retain materially different authority. The value is a local acceleration witness, not a recovery guarantee or a learned-policy score.

Angular rows use rad/s²; translation rows use m/s². `±1 local gain` is the worse signed response fraction over the two request signs. `maximum achieved` is the largest directional response anywhere on the 1/10/50/250 request curve.

| rolling-wheel axis | ±1 local gain | maximum achieved | −250 achieved | +250 achieved | 250 minimum fraction | max effort use | status |
|---|---|---|---|---|---|---|---|
| roll | 0.8068 | 15.2984 | -15.2495 | 13.8712 | 0.0555 | 16.03% | SolvedWithSlack |
| pitch | 1.0000 | 9.2787 | -9.2787 | 7.1253 | 0.0285 | 24.92% | SolvedWithSlack |
| yaw | 0.9999 | 116.7128 | -116.7128 | 76.4385 | 0.3058 | 50.91% | SolvedWithSlack |
| forward | 0.9367 | 1.1059 | -1.1059 | 0.7851 | 0.0031 | 25.64% | SolvedWithSlack |
| lateral | 0.1933 | 8.8103 | -6.4040 | 7.4594 | 0.0256 | 16.20% | SolvedWithSlack |
| vertical | 1.0000 | 3.9900 | -3.9900 | 3.7326 | 0.0149 | 18.74% | SolvedWithSlack |

## Contact-mode comparison at the saturation request

| mode | axis | ±1 local gain | maximum achieved | −250 achieved | +250 achieved | 250 minimum fraction | max task RMS |
|---|---|---|---|---|---|---|---|
| locked_point | roll | 0.8068 | 13.1575 | -13.1053 | 13.0965 | 0.0524 | 136.7776 |
| locked_point | pitch | 0.1660 | 1.7286 | -1.7286 | -0.5141 | -0.0021 | 144.6344 |
| locked_point | yaw | 0.9999 | 77.0772 | -77.0772 | 77.0770 | 0.3083 | 99.8384 |
| locked_point | forward | -0.0806 | 0.2319 | -0.2319 | -0.0919 | -0.0004 | 176.8417 |
| locked_point | lateral | 0.1933 | 6.4421 | -6.3901 | 6.3901 | 0.0256 | 172.2582 |
| locked_point | vertical | 0.3801 | 13.6318 | -10.4173 | 13.6318 | 0.0417 | 239.5827 |
| normal_point | roll | 1.0000 | 32.7899 | -32.4609 | 32.4609 | 0.1298 | 125.5963 |
| normal_point | pitch | 1.0000 | 25.2951 | -17.8666 | 25.2374 | 0.0715 | 134.0223 |
| normal_point | yaw | 1.0000 | 249.9972 | -249.9972 | 85.1824 | 0.3407 | 95.1575 |
| normal_point | forward | 1.0000 | 6.6840 | -2.7006 | 1.9233 | 0.0077 | 175.4167 |
| normal_point | lateral | 1.0000 | 5.9592 | -5.9259 | 5.9259 | 0.0237 | 172.5865 |
| normal_point | vertical | 1.0000 | 12.0199 | -6.1080 | 12.0199 | 0.0244 | 243.8920 |
| rolling_point | roll | 0.8068 | 15.2609 | -15.2609 | 12.2253 | 0.0489 | 137.2793 |
| rolling_point | pitch | 1.0000 | 49.6280 | -17.1443 | 25.2374 | 0.0686 | 134.4393 |
| rolling_point | yaw | 0.9999 | 246.4925 | -57.3984 | 246.4925 | 0.2296 | 111.1987 |
| rolling_point | forward | 1.0000 | 2.7897 | -2.7897 | 1.9233 | 0.0077 | 175.4167 |
| rolling_point | lateral | 0.1933 | 7.5118 | -7.4664 | 5.0090 | 0.0200 | 173.2348 |
| rolling_point | vertical | 1.0000 | 12.0199 | -6.1079 | 12.0199 | 0.0244 | 243.8921 |
| rolling_wheel | roll | 0.8068 | 15.2984 | -15.2495 | 13.8712 | 0.0555 | 136.3304 |
| rolling_wheel | pitch | 1.0000 | 9.2787 | -9.2787 | 7.1253 | 0.0285 | 140.2238 |
| rolling_wheel | yaw | 0.9999 | 116.7128 | -116.7128 | 76.4385 | 0.3058 | 100.2070 |
| rolling_wheel | forward | 0.9367 | 1.1059 | -1.1059 | 0.7851 | 0.0031 | 176.2215 |
| rolling_wheel | lateral | 0.1933 | 8.8103 | -6.4040 | 7.4594 | 0.0256 | 172.2484 |
| rolling_wheel | vertical | 1.0000 | 3.9900 | -3.9900 | 3.7326 | 0.0149 | 246.2674 |

## Admission and execution evidence

| witness | value |
|---|---|
| statuses | {"Solved": 22, "SolvedWithSlack": 174} |
| maximum hard violation | 1.712e-12 |
| maximum dynamics residual | 1.712e-12 |
| maximum contact residual | 2.461e-13 |
| Rust allocations in timed calls | 0 calls / 0 bytes |
| Rust solve p50 / p99 | 139.3 / 191.1 µs |

## Interpretation contract

- `SolvedWithSlack` means hard dynamics/contact/bounds are admitted while at least one soft priority cannot realize its request.
- Signed fraction is achieved acceleration along the requested axis divided by the request; it is deliberately threshold-free.
- Orthogonal leakage, per-layer RMS/clipping, friction margin, effort utilization, and hard residual remain separate in the JSON rather than being collapsed into one health score.
- The r133 MuJoCo envelope remains the plant-consequence gate. This report explains a local authority boundary; it does not promote the experimental planar steering law or claim lateral recovery.

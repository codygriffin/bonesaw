# Bonesaw paired terminal-state tube boundary · r256

> Mechanism **PASS** · profile **NOT FROZEN** · authority **NOT ADMITTED**.

R256 bounds candidate-minus-baseline consequence directly over a shared terminal-state tube. Each candidate is the same uncertain baseline plus a candidate delta; Rust never subtracts independent score uppers. Tilt, angular rate, joint-position pressure, joint-velocity pressure, actuator effort, raw headroom loss, and aggregate consequence remain separate.

A fixed 3-candidate × 4-hypothesis G1-shaped batch repeats 500 times at p50/p99 380.52/431.98 µs with zero measured Rust allocation. All 384 independently sampled paired points are contained; minimum slack is 0. The baseline is exactly zero by construction.

This freezes only the correlation-preserving mechanism. No contact-law/estimator tube has been fitted, no action profile is selected, and authority remains closed. The next evaluation must fit a causal tube on spent state evidence before any new-law holdout.

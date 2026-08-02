# Bonesaw causal contact/state tube replay · r256

> Mechanism **PASS** · terminal coverage **288/288 (100.000%)** · profile **NOT FROZEN** · authority **NOT ADMITTED**.

R256 consumes immutable R254 states and constructs four authored directional contact-law/estimator hypotheses per candidate. Rust owns the point response, directional velocity-jump outer bound, complete terminal-state-box scorer, and paired candidate-minus-baseline delta envelope; Python only lifts the finite velocity tube into q/clearance/attitude intervals and reports coverage. The ballistic clearance/vertical-speed columns are unioned across candidates so impact-speed deltas remain explicitly candidate-invariant; candidate/baseline rows retain the same hypothesis index.

The audit performs 96 MuJoCo kinematic forwards, zero physics steps, zero policy steps, 96 evaluation-only paired selectors, and zero plant actions. Contact/state-box/paired timed Rust allocation is zero; semantic replay is exact. Upper component coverage is 100.000% and headroom coverage is 100.000%, with maximum excess/shortfall 0/0.

This is a causal shared-hypothesis construction replay, not a calibrated transfer certificate. The finite hypothesis family, q-drift lift, and absolute consequence coverage must be frozen on spent evidence, then tested once on new contact laws/offsets with paired lower/upper deltas before any action or authority can be considered.

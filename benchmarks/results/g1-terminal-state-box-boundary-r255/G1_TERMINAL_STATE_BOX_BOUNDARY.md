# Bonesaw complete terminal-state box boundary · r255

> Mechanism **PASS** · profile **NOT FROZEN** · authority **NOT ADMITTED**.

R255 adds a Rust-owned complete state interval before support-free terminal propagation. Clearance, vertical speed, roll/pitch, angular rate, joint position, and joint velocity are all bounded; pressure fields are upper bounds and joint headroom is a lower bound. The query performs no policy step, physics step, selector call, plant action, probability assignment, or command admission.

A G1-shaped 64-row batch repeats 500 times with p50/p99 50.72/75.50 µs and zero measured Rust allocation. 2,048 independently scored box points are contained; minimum upper/headroom slack is -4.44e-16/-2.78e-17.

This boundary closes only the interval-composition mechanism. Independent candidate boxes cannot prove a paired candidate-minus-baseline improvement because their errors may be correlated. A causal shared-hypothesis or paired state tube must be frozen on spent contact-law/estimator evidence before another new-law holdout.

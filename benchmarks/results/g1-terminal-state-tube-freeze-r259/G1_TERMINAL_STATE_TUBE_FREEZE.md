# Bonesaw complete terminal-state tube freeze · r259

> Mechanism **PASS** · profile **REJECTED** · authority **NOT ADMITTED**.

R259 fits complete terminal attitude, angular-rate, joint-position, and joint-velocity residual tubes from immutable R258 state arrays. Candidate and baseline remain paired through the R256 Rust boundary. This evaluator executes zero policy steps, physics steps, or plant actions; R258's spent extraction is a separate provenance layer.

Source component/aggregate coverage is 100.000%/100.000%. The selector chooses 0 nonzero rows; selected component/aggregate regressions are 0/0, and p99 is 409.42 µs.

Coverage is complete but not useful: the smallest nonzero candidate joint-position upper is 18.4863, the smallest worst-component upper is 18.4863, and the smallest aggregate upper is 9.45105. The coarse group/law componentwise state boxes discard too much cross-coordinate and state-local structure.

The unchanged profile is rehearsed on already-spent R254 diagnostics: component/aggregate coverage is 98.553%/97.917%, with 0 nonzero selections and 0/0 selected component/aggregate regressions.

Only complete source coverage plus a useful strictly nonregressing action can freeze the profile. The spent rehearsal may reject but cannot serve as a fresh holdout or admit authority.

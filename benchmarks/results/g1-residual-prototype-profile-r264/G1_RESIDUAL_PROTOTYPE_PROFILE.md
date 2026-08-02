# Bonesaw cross-law residual prototype profile · r264

> Mechanism **PASS** · spent four-law profile **PASS** · fresh holdout **NOT RUN** · authority **NOT ADMITTED**.

R264 fits one causal profile from the already-spent R258 and R254 laws. The query uses 55 complete observed-state coordinates, restricts nearest-neighbor lookup to the existing closing-speed/tilt cell, adds asymmetric cell calibration extensions, and fails closed outside the spent nearest-distance envelope. Rust owns the bounded 192-prototype lookup, box construction, distance gate, and conservative selection. This evaluator runs zero policy steps, physics steps, and plant actions.

The leave-one-law-out construction covers 100.000% of component values and 100.000% of aggregate values across all four spent families. It selects 3 nonzero actions; all 3 are strictly nonregressing and improving, with zero selected component or aggregate regression.

All 192 rows remain inside their causal distance gates. The allocation-free Rust query runs at 1.23 µs p99 and repeats every non-timing output exactly. Maximum fitted candidate component/aggregate upper extensions are 38.317/19.137; these broad cells explain why only three actions survive.

The profile is frozen for exactly one new-law/offset holdout. No additional calibration, widening, candidate change, plant command, or authority is allowed before that holdout.

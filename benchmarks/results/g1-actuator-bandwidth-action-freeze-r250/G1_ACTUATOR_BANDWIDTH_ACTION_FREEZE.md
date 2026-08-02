# Bonesaw actuator-bandwidth action freeze · r250

> Mechanism **PASS** · useful safe profile **NOT FOUND** · authority **NOT ADMITTED**.

The rejected R248 states are deliberately spent design evidence. Candidate zero is now exact fixed-zero-effort WBC realization. Two nonzero candidates pass the frozen WBC effort through Rust's persistent bandwidth/slew model at each plant substep, then exact fixed-effort WBC predicts the average realized acceleration. All three candidates are rolled out on the spent states to fit one global componentwise plant-residual tube per candidate. The state-local Rust selector sees only those frozen tubes; a minimum-improvement guard may only fall back to zero and cannot switch to an outcome-fitted action.

| profile | family | raw selected · zero / zero-WBC / third | safe threshold | nonzero / actually improved | decision |
|---|---|---|---|---|---|
| bandwidth_25hz_slew_1000_nm_s | zero_wbc_and_velocity_damping | 96 / 0 / 0 | NONE | 0 / 0 | REJECT |
| bandwidth_25hz_slew_1000_nm_s | zero_wbc_and_neutral_recovery | 96 / 0 / 0 | NONE | 0 / 0 | REJECT |

A profile passes only with zero component regressions on all 96 spent plant states, at least one nonzero action, at least one actual aggregate-score improvement, zero MuJoCo warnings, admitted raw and fixed-effort WBC, and zero Rust allocation in WBC and realization hot paths. Any frozen profile must next face new laws and offsets without retuning; this report cannot admit authority.

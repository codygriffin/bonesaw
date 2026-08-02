# Bonesaw passive damping action audit · r252

> Passivity mechanism **PASS** · useful strict action **NOT FOUND** · authority **NOT ADMITTED**.

R252 evaluates actuator-coordinate `tau_request = -gain * velocity` at every 4 ms plant step behind the R249 25 Hz / 1,000 N·m/s realization and a Rust zero-positive-power cap. The 96 R248 states are spent design evidence; every nonzero branch runs twice for bitwise semantic replay. No policy runs.

| gain Nm/(rad/s) | strict nonregression | aggregate improved | kinetic energy reduced | power clamps | p99 µs | action decision |
|---|---|---|---|---|---|---|
| 1.0 | 24 / 96 | 89 / 96 | 90 / 96 | 289 | 0.88 | REJECT |
| 2.0 | 23 / 96 | 89 / 96 | 91 / 96 | 546 | 0.70 | REJECT |
| 5.0 | 26 / 96 | 89 / 96 | 94 / 96 | 765 | 0.84 | REJECT |
| 10.0 | 28 / 96 | 87 / 96 | 95 / 96 | 945 | 0.83 | REJECT |

The mechanism passes only if every applied actuator coordinate has non-positive observed mechanical power, every repeated plant trace is bitwise exact, timed Rust allocation is zero, and MuJoCo emits no warnings. An action would additionally require zero regression in every terminal component on all 96 states. Passivity is therefore visible evidence, not a substitute for the terminal consequence gate.

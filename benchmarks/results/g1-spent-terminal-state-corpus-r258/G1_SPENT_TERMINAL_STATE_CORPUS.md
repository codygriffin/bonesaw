# Bonesaw spent terminal-state corpus · r258

> Extraction **PASS** · profile **NOT FROZEN** · authority **NOT ADMITTED**.

R258 replays the already-spent R250 laws, offsets, 25 Hz actuator realization, and neutral-recovery candidate family solely to retain the terminal state that the historical artifact omitted: root clearance/vertical speed, roll/pitch and angular rate, every joint position, and every generalized velocity. This is corpus extraction, not a fresh holdout.

The extraction executes 480 WBC queries and 1440 MuJoCo steps over 96 × 3 rows, with zero policy steps and zero plant actions. All WBC queries admit, MuJoCo warnings are 0, and an independent allocation-free Rust rescore reproduces all 4,896 terminal diagnostics bit-for-bit at p99 0.36 µs.

The new state arrays are spent design evidence. R259 must fit and evaluate the paired tube from these immutable arrays with zero physics and zero policy; no authority follows from extraction itself.

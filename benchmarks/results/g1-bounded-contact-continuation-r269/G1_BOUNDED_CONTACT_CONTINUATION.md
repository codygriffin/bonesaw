# G1 bounded contact continuation · R269

**Mechanism PASS / walking profile REJECTED.** No policy or physics simulator is involved. The Rust controller integrates only accepted outputs; typed hold ticks preserve the represented state exactly while an identical exhausted hard problem receives another bounded feasibility slice.

## Result

| profile | first contingency | first global release | full root RMS m | max attitude deg | p99 ms |
|---|---|---|---|---|---|
| r268 low gain baseline | 863 | 869 | 18.879 | 40.48 | 4.911 |
| r269 bounded continuation | 863 | 888 | 17.026 | 120.38 | 4.844 |
| r269 localized handoff control | 511 | 584 | 22.423 | 84.93 | 6.097 |

The low-gain frontier still enters NormalFallback at tick 863, but the first all-contact release moves from tick 869 to tick 888—a 19-tick (0.095 s) extension. This is a failure-handling improvement, not nominal tracking progress.

This diagnostic profile predeclares target 1 (the right foot) as the first contact eligible for normal-only demotion. That is evaluator-authored fault isolation, not a claim that the controller inferred which foot was bad. At tick 869, localized handoff removes that failed right support while the left foot remains Locked with 336.5 N. The accepted partial-support solution has hard residuals below 1e-8.

Ticks 876–887 are status 8 (`contact_solve_hold`). State is bitwise constant across q, v, root pose, tracked points, and CoM. Cumulative feasibility work advances 16, 32, …, 192 sweeps. The configured cap is eight sweeps per solver query, while this WBC path can issue a primary query plus one retry, so the observable aggregate ceiling is 16 sweeps per WBC tick. Rejected residuals on hold ticks are diagnostic only and are never integrated.

The independent centroidal control exercises status 9 (`localized_contact_handoff`) at tick 529: the failed right support is removed while the incoming left support remains active with 227.3 N and 1.30e-10/1.26e-12 residuals. All later global-release ticks clear rejected residual diagnostics to zero.

## Decision

The candidate remains red: first contingency is unchanged at tick 863, global release still occurs after only 0.095 s of additional degraded operation, full-run root RMS is 17.026 m, and maximum attitude error is 120.38°. Candidate p99 is 4.844 ms, but the independent localized-handoff control is 6.097 ms and misses the 5 ms target.

Defaults and execution authority remain unchanged. Cross-tick continuation requires seed reuse, a finite per-call sweep cap, and an explicit hold timeout. The next large controller slice is continuous recovery from persistent NormalFallback with post-touchdown root/attitude tracking—not more solver work and not weaker contact constraints.

## Contract

- Reference SHA-256: `429baba94dae0c3b4e490dc7f0bc107416d3c0559fbfbad4df6e009fac3e48c6`.
- Witness SHA-256: `f1521acd7a0cc777f61407e49865f5953355d4382035421fb2582f2fa0164295`.
- Status 8 is a bounded, non-integrating hold; status 9 is a physically checked partial-support solution.
- Status 5 remains global free-body release and carries zero dynamics/contact residual witnesses.
- The continuation is opt-in and capped at 12 hold ticks in this evaluation; each solve is capped at eight sweeps and the current WBC retry path can issue two solves per tick.
- Target 1/right-foot fallback is explicitly authored by this diagnostic profile; production fault localization remains open.
- No future oracle WBC force, acceleration, status, or policy outputs are consumed.

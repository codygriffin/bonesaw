# Bonesaw Upkie synthetic contact-ingress replay · upkie-synthetic-contact-ingress-r298

> Evaluation **PASS**. This report drives the existing Python MuJoCo worker and persistent Rust adapter with a deterministic synthetic contact buffer; it adds no policy, solver change, or server.

## Contract

The harness replays 10 WBC ticks at 50 Hz. Each tick consumes exactly 5 source frames at 250 Hz and selects the newest frame; each phase edge is placed on that newest frame to exercise the sampling boundary. The sequence covers `11`, `10`, `01`, and `00`; Rust's default three-sample activation and two-sample deactivation debounce is observed at the WBC boundary.

The replacement is limited to `measured_wheel_ground_contacts_into`: the worker still performs its normal five MuJoCo integration substeps and calls the existing `RustWbcAdapter.solve` path. The synthetic buffer is consumed in one extractor call at the 50 Hz boundary; it does not prove per-substep production sampling, measured MuJoCo transfer, or a hardware contact estimator.

## Trace

| tick | source mask | debounced | hard | status | provenance | flags | support | reset epoch |
|---:|:---:|:---:|:---:|---:|---:|---:|---:|---:|
| 1 | `1 1` | `0 0` | `0 0` | 0 | 0 | 128 | 0 | 0 |
| 2 | `1 1` | `0 0` | `0 0` | 0 | 0 | 128 | 0 | 0 |
| 3 | `1 1` | `1 1` | `1 1` | 0 | 0 | 256 | 2 | 0 |
| 4 | `1 0` | `1 1` | `1 0` | 0 | 0 | 128 | 1 | 0 |
| 5 | `1 0` | `1 0` | `1 0` | 0 | 0 | 256 | 1 | 0 |
| 6 | `0 1` | `1 0` | `0 0` | 0 | 0 | 128 | 0 | 0 |
| 7 | `0 1` | `0 0` | `0 0` | 0 | 0 | 384 | 0 | 0 |
| 8 | `0 1` | `0 1` | `0 1` | 0 | 0 | 256 | 1 | 0 |
| 9 | `0 0` | `0 1` | `0 0` | 0 | 0 | 128 | 0 | 0 |
| 10 | `0 0` | `0 0` | `0 0` | 0 | 0 | 256 | 0 | 0 |
| 11 (paused) | — | `0 0` | `0 0` | 0 | 0 | 256 | 0 | 0 |

## Gates

| gate | result |
|:---|:---:|
| rate_split_declared | PASS |
| all_contact_patterns | PASS |
| raw_mask_passthrough | PASS |
| source_consumes_exactly_five_frames_per_tick | PASS |
| source_chunks_select_newest_frame | PASS |
| source_replays_edges_inside_control_window | PASS |
| activation_debounce | PASS |
| deactivation_debounce | PASS |
| hard_rows_intersect_raw | PASS |
| hard_rows_fail_closed_on_zero_contact | PASS |
| running_observation_is_exact | PASS |
| pause_fails_closed_without_reset | PASS |
| no_reset_during_trace | PASS |
| exact_replay | PASS |

The paused heartbeat intentionally publishes no observed-contact availability, debounced mask, hard rows, or support count. The trace must complete without a numeric/fall reset or a reset-epoch change.

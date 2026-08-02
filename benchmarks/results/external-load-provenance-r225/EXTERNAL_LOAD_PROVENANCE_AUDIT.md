# External-load provenance audit · external-load-provenance-r225

Admission: **PASS**. Protocol 2 types declared external wrenches separately from measured impact impulse and unobserved-model reserve.

## Rejected command categories

| attempt | response | reject ms | stream | worker unchanged |
|---|---|---|---|---|
| missing_provenance | invalid plant command: missing field `provenance` | 22.155 | True | True |
| measured_impact_impulse | invalid plant command: external-load class is evidence-only and cannot execute | 66.842 | True | True |
| unobserved_model_reserve | invalid plant command: external-load class is evidence-only and cannot execute | 26.990 | True | True |

## Accepted declared sources

| source | class | ack ms | exact echo | released |
|---|---|---|---|---|
| evaluation_harness | declared_continuous_wrench | 46.435 | True | True |
| interactive_operator | declared_continuous_wrench | 111.712 | True | True |

The Rust gateway rejects missing provenance and refuses to execute evidence-only impact/reserve classes before mutating the worker command. The Python MuJoCo worker independently revalidates the executable class, source, and both world frames. Browser and evaluation sources are distinct and echoed in measured plant state.

This is provenance and transport evidence, not an impact estimator, model-error calibration, authenticated operator identity, or a claim that an unobserved-model reserve is zero. Both unavailable quantities remain explicit unavailable records.

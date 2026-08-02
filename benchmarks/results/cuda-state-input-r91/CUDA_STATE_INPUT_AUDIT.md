# Bonesaw CUDA mirror state input · r91

## Outcome

**DEVICE UNAVAILABLE.** R91 is the first real device implementation boundary after CPU concept admission. `bonesaw-cuda` now embeds a fixed PTX state-input kernel, dynamically loads the CUDA Driver API, owns a context/module/five fixed device buffers, validates finite `f32` state on-device, zeros malformed and padded agents, and exposes a runtime fingerprint. The execution method has no CPU fallback.

This is deliberately a **StateInput-only** claim. FK, Jacobians, dynamics, task/constraint emission, hierarchy, actuation, integration, CUDA Graph capture, and `CudaThroughputF32` remain unimplemented on device.

## Compiled boundary

| signal | value |
|---|---|
| runtime probe | no_device |
| driver version | None |
| visible devices | 0 |
| kernel manifest bits | 1 |
| kernel SHA-256 | 45652bbb5db690174ece50f579fe350527db76e15b580eecf66bbe11d93a567d |
| ptxas syntax validation | unavailable |
| profile capability | state input=True · full mirror=False · Graph=False |
| layout | 18 coordinates · 17 active capacity · stride 32 |
| hidden CPU fallback | False |

The PTX assigns one thread to one complete agent, uses no atomics, performs no cross-agent reduction, and writes deterministic zeroes for inactive or invalid lanes. Driver symbols are resolved only during executor construction, so a CPU-only process can load the crate and receive typed unavailability.

## CPU mirror witness

| gate | result |
|---|---|
| active agents typed OK | True |
| padding inactive and zero | True |
| allocation-free 100-call Rust unit sentinel | True |
| bitwise repeated CPU output | True |

Device D1/D2, launch, timing, and isolation gates are **NOT RUN**. The report does not substitute CPU execution or call this a CUDA pass.

## Admission boundary

R91 does not promote `CudaMirrorF32` as a backend. It promotes only the compiled StateInput stage and the runtime probe. A CUDA-capable CI host must still JIT the retained kernel and pass D1 repeated bytes, D2 CPU-mirror bytes, malformed-agent isolation, padding, permutation/chunking, direct-versus-Graph, allocation/memory, error-injection, and timing gates. The current host reports `no_device`; therefore all device gates remain unavailable rather than emulated.

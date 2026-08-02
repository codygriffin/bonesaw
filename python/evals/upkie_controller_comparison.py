#!/usr/bin/env python3
"""Compare Bonesaw's typed Rust balance law with Upkie's pinned C++ class.

The shared corpus stays inside the common, physically relevant subset:
continuous floor contact, zero requested yaw and a stationary ground-position
target. The upstream worker executes WheelBalancer::read/write unchanged.
Bonesaw is run once with upstream parameters for an implementation-parity gate
and once with the tuned live parameters to quantify the intentional policy
delta. All controller loops execute in C++ or Rust; Python owns corpus design,
process isolation, statistics and report rendering.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import pathlib
import platform
import resource
import subprocess
import time
from dataclasses import asdict, dataclass
from typing import Any

import numpy as np
import psutil


DT = 0.005


@dataclass(frozen=True)
class ProcessMetrics:
    wall_seconds: float
    user_cpu_seconds: float
    system_cpu_seconds: float
    cpu_to_wall: float
    peak_rss_bytes: int
    minor_faults: int
    major_faults: int
    voluntary_context_switches: int
    involuntary_context_switches: int
    stderr: str


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ticks", type=int, default=100_000)
    parser.add_argument(
        "--output", default="benchmarks/results/reference-latest"
    )
    parser.add_argument(
        "--rust-worker",
        default="target/release/bonesaw-upkie-balance-worker",
    )
    parser.add_argument("--upkie-worker", required=True)
    parser.add_argument("--upkie-source", required=True)
    parser.add_argument("--model", default="models/upkie/upkie.urdf")
    return parser.parse_args()


def build_corpus(ticks: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    if ticks < 400:
        raise ValueError("--ticks must be at least 400")
    time_s = np.arange(ticks, dtype=np.float64) * DT
    ground = np.zeros(ticks, dtype=np.float64)
    pitch = np.zeros(ticks, dtype=np.float64)
    regions = np.empty(ticks, dtype="U24")
    edges = np.linspace(0, ticks, 5, dtype=int)

    regions[edges[0] : edges[1]] = "zero_hold"

    sl = slice(edges[1], edges[2])
    local = time_s[sl] - time_s[edges[1]]
    ground[sl] = 0.035 * np.sin(2.0 * math.pi * 0.37 * local)
    pitch[sl] = (
        0.045 * np.sin(2.0 * math.pi * 0.71 * local)
        + 0.015 * np.cos(2.0 * math.pi * 0.11 * local)
    )
    regions[sl] = "smooth_coupled"

    sl = slice(edges[2], edges[3])
    local_index = np.arange(edges[3] - edges[2])
    block = (local_index // 250) % 4
    ground[sl] = np.choose(block, [0.06, -0.04, -0.06, 0.04])
    pitch[sl] = np.choose(block, [0.08, 0.04, -0.08, -0.04])
    regions[sl] = "bounded_steps"

    rng = np.random.default_rng(0xB0E5A7)
    sl = slice(edges[3], edges[4])
    count = edges[4] - edges[3]
    ground_noise = rng.normal(0.0, 0.0025, count)
    pitch_noise = rng.normal(0.0, 0.0035, count)
    for index in range(1, count):
        ground_noise[index] += 0.985 * ground_noise[index - 1]
        pitch_noise[index] += 0.978 * pitch_noise[index - 1]
    ground[sl] = np.clip(ground_noise, -0.09, 0.09)
    pitch[sl] = np.clip(pitch_noise, -0.12, 0.12)
    regions[sl] = "seeded_colored_noise"
    return ground, pitch, regions


def write_corpus(path: pathlib.Path, ground: np.ndarray, pitch: np.ndarray) -> None:
    with path.open("w", encoding="utf-8") as stream:
        stream.write("ground_position\tpitch\n")
        for position, angle in zip(ground, pitch, strict=True):
            stream.write(f"{position:.17e}\t{angle:.17e}\n")


def usage_delta(after: resource.struct_rusage, before: resource.struct_rusage) -> dict[str, Any]:
    return {
        "user_cpu_seconds": after.ru_utime - before.ru_utime,
        "system_cpu_seconds": after.ru_stime - before.ru_stime,
        "minor_faults": after.ru_minflt - before.ru_minflt,
        "major_faults": after.ru_majflt - before.ru_majflt,
        "voluntary_context_switches": after.ru_nvcsw - before.ru_nvcsw,
        "involuntary_context_switches": after.ru_nivcsw - before.ru_nivcsw,
    }


def run_worker(command: list[str]) -> ProcessMetrics:
    before = resource.getrusage(resource.RUSAGE_CHILDREN)
    started = time.perf_counter()
    process = subprocess.Popen(
        command,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
    )
    child = psutil.Process(process.pid)
    peak_rss = 0
    while process.poll() is None:
        try:
            peak_rss = max(peak_rss, child.memory_info().rss)
        except psutil.Error:
            pass
        time.sleep(0.001)
    _, stderr = process.communicate()
    wall = time.perf_counter() - started
    if process.returncode:
        raise subprocess.CalledProcessError(
            process.returncode, command, stderr=stderr
        )
    after = resource.getrusage(resource.RUSAGE_CHILDREN)
    delta = usage_delta(after, before)
    cpu = delta["user_cpu_seconds"] + delta["system_cpu_seconds"]
    return ProcessMetrics(
        wall_seconds=wall,
        user_cpu_seconds=delta["user_cpu_seconds"],
        system_cpu_seconds=delta["system_cpu_seconds"],
        cpu_to_wall=cpu / wall,
        peak_rss_bytes=peak_rss,
        minor_faults=delta["minor_faults"],
        major_faults=delta["major_faults"],
        voluntary_context_switches=delta["voluntary_context_switches"],
        involuntary_context_switches=delta["involuntary_context_switches"],
        stderr=stderr.strip(),
    )


def load_trace(path: pathlib.Path) -> dict[str, np.ndarray]:
    table = np.genfromtxt(path, names=True, delimiter="\t")
    return {name: table[name] for name in table.dtype.names or ()}


def distribution(values_ns: np.ndarray) -> dict[str, float]:
    values = values_ns.astype(np.float64) / 1_000.0
    jitter = np.abs(np.diff(values))
    median = float(np.median(values))
    return {
        "mean_us": float(np.mean(values)),
        "std_us": float(np.std(values)),
        "mad_us": float(np.median(np.abs(values - median))),
        "p50_us": median,
        "p90_us": float(np.percentile(values, 90)),
        "p95_us": float(np.percentile(values, 95)),
        "p99_us": float(np.percentile(values, 99)),
        "p99_9_us": float(np.percentile(values, 99.9)),
        "p99_99_us": float(np.percentile(values, 99.99)),
        "max_us": float(np.max(values)),
        "jitter_p99_us": float(np.percentile(jitter, 99)),
    }


def canonical_bits(values: np.ndarray) -> np.ndarray:
    canonical = values.copy()
    canonical[canonical == 0.0] = 0.0
    return canonical.view(np.uint64)


def comparison(reference: np.ndarray, candidate: np.ndarray) -> dict[str, Any]:
    delta = candidate - reference
    denominator = np.linalg.norm(reference)
    return {
        "numerically_equal": bool(np.array_equal(reference, candidate)),
        "canonical_bitwise_equal": bool(
            np.array_equal(canonical_bits(reference), canonical_bits(candidate))
        ),
        "canonical_bit_mismatches": int(
            np.count_nonzero(canonical_bits(reference) != canonical_bits(candidate))
        ),
        "maximum_absolute_error": float(np.max(np.abs(delta))),
        "rms_error": float(np.sqrt(np.mean(np.square(delta)))),
        "relative_l2_error": float(np.linalg.norm(delta) / denominator)
        if denominator
        else 0.0,
        "correlation": float(np.corrcoef(reference, candidate)[0, 1])
        if np.std(reference) > 0 and np.std(candidate) > 0
        else 1.0,
    }


def region_metrics(
    regions: np.ndarray,
    upkie: dict[str, np.ndarray],
    rust_official: dict[str, np.ndarray],
    rust_live: dict[str, np.ndarray],
) -> dict[str, Any]:
    output = {}
    for name in np.unique(regions):
        mask = regions == name
        output[str(name)] = {
            "samples": int(np.count_nonzero(mask)),
            "official_command": comparison(
                np.concatenate(
                    [
                        upkie["left_wheel_velocity"][mask],
                        upkie["right_wheel_velocity"][mask],
                    ]
                ),
                np.concatenate(
                    [
                        rust_official["left_wheel_velocity"][mask],
                        rust_official["right_wheel_velocity"][mask],
                    ]
                ),
            ),
            "live_ground_velocity": comparison(
                upkie["ground_velocity"][mask],
                rust_live["ground_velocity"][mask],
            ),
            "latency": {
                "upkie_cpp": distribution(upkie["elapsed_ns"][mask]),
                "bonesaw_official": distribution(
                    rust_official["elapsed_ns"][mask]
                ),
                "bonesaw_live": distribution(rust_live["elapsed_ns"][mask]),
            },
        }
    return output


def temporal_windows(
    upkie: dict[str, np.ndarray],
    rust_official: dict[str, np.ndarray],
    rust_live: dict[str, np.ndarray],
    count: int = 10,
) -> list[dict[str, Any]]:
    output = []
    for window, indexes in enumerate(
        np.array_split(np.arange(len(upkie["ground_velocity"])), count)
    ):
        live_delta = (
            rust_live["ground_velocity"][indexes]
            - upkie["ground_velocity"][indexes]
        )
        output.append(
            {
                "window": window,
                "start": int(indexes[0]),
                "stop": int(indexes[-1] + 1),
                "upkie_p50_us": float(
                    np.percentile(upkie["elapsed_ns"][indexes], 50) / 1_000.0
                ),
                "upkie_p99_us": float(
                    np.percentile(upkie["elapsed_ns"][indexes], 99) / 1_000.0
                ),
                "rust_official_p50_us": float(
                    np.percentile(
                        rust_official["elapsed_ns"][indexes], 50
                    )
                    / 1_000.0
                ),
                "rust_official_p99_us": float(
                    np.percentile(
                        rust_official["elapsed_ns"][indexes], 99
                    )
                    / 1_000.0
                ),
                "rust_live_p50_us": float(
                    np.percentile(rust_live["elapsed_ns"][indexes], 50)
                    / 1_000.0
                ),
                "rust_live_p99_us": float(
                    np.percentile(rust_live["elapsed_ns"][indexes], 99)
                    / 1_000.0
                ),
                "live_delta_rms_mps": float(
                    np.sqrt(np.mean(np.square(live_delta)))
                ),
                "live_delta_max_mps": float(np.max(np.abs(live_delta))),
            }
        )
    return output


def hash_file(path: pathlib.Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def fmt_bytes(value: int) -> str:
    return f"{value / (1024 * 1024):.2f} MiB"


def render_report(metrics: dict[str, Any]) -> str:
    parity = metrics["comparisons"]["official_aligned"]
    live = metrics["comparisons"]["live_tuned"]
    lines = [
        "# Bonesaw ↔ official Upkie wheel-controller comparison",
        "",
        "This is a direct implementation-oracle test. The reference worker links the",
        "pinned upstream `WheelBalancer.cpp` class unchanged; the Rust worker calls",
        "Bonesaw's typed balance law. Python owns the shared corpus, process isolation,",
        "statistics, artifacts, and this report. No Python code runs inside either",
        "measured controller loop.",
        "",
        "## Scope and interpretation",
        "",
        "- Shared subset: continuous floor contact, zero requested yaw, stationary target",
        "  ground position, and pitch below the upstream fall threshold.",
        "- The official-aligned Rust profile uses Upkie's exact default gains, limits,",
        "  wheel radius, update order, and 5 ms timestep. This is the parity gate.",
        "- The live Rust profile uses the gains and 5 cm wheel radius selected by the",
        "  integrated Bonesaw WBC corpus. Its delta is intentional and reported rather",
        "  than hidden.",
        "- Upkie latency includes its `palimpsest::Dictionary` read/write adapter and hip/",
        "  knee gain writes. Rust latency is the typed law call. Use these numbers to",
        "  understand boundary cost, not as a whole-robot WBC speed comparison.",
        "",
        "## Provenance",
        "",
        "| Item | Value |",
        "|---|---|",
        f"| Upkie commit | `{metrics['provenance']['upkie_commit']}` |",
        f"| WheelBalancer.cpp SHA-256 | `{metrics['provenance']['wheel_balancer_cpp_sha256']}` |",
        f"| WheelBalancer.h SHA-256 | `{metrics['provenance']['wheel_balancer_h_sha256']}` |",
        f"| Corpus | {metrics['ticks']:,} sequential 5 ms samples · seed `0xB0E5A7` |",
        f"| Host | {metrics['environment']['cpu']} · {metrics['environment']['platform']} |",
        "",
        "## Command equivalence",
        "",
        "| Candidate | Canonical bitwise | mismatches | max abs command error | RMS error | relative L2 | correlation |",
        "|---|---:|---:|---:|---:|---:|---:|",
        f"| Rust, official parameters | {'PASS' if parity['canonical_bitwise_equal'] else 'FAIL'} | {parity['canonical_bit_mismatches']:,} | {parity['maximum_absolute_error']:.3e} rad/s | {parity['rms_error']:.3e} rad/s | {parity['relative_l2_error']:.3e} | {parity['correlation']:.9f} |",
        f"| Rust, live tuned parameters | intentionally different | — | {live['maximum_absolute_error']:.3e} m/s | {live['rms_error']:.3e} m/s | {live['relative_l2_error']:.3e} | {live['correlation']:.9f} |",
        "",
        "The parity gate compares the two emitted wheel-velocity commands. Signed zero",
        "is canonicalized; every nonzero value is compared by its exact IEEE-754 bytes.",
        "The inferred upstream ground velocity (`left × radius`) can differ from the",
        "Rust law's pre-division value by one rounding step, so it is retained as a",
        "separate diagnostic rather than mislabelled as the actuator command.",
        "",
        "## Latency and jitter",
        "",
        "| Worker | mean µs | std µs | MAD µs | p50 µs | p95 µs | p99 µs | p99.9 µs | p99.99 µs | max µs | jitter p99 µs |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for name, label in [
        ("upkie_cpp", "Official Upkie C++ read/write"),
        ("bonesaw_official", "Bonesaw Rust typed · official gains"),
        ("bonesaw_live", "Bonesaw Rust typed · live gains"),
    ]:
        d = metrics["latency"][name]
        lines.append(
            f"| {label} | {d['mean_us']:.3f} | {d['std_us']:.3f} | "
            f"{d['mad_us']:.3f} | {d['p50_us']:.3f} | {d['p95_us']:.3f} | "
            f"{d['p99_us']:.3f} | {d['p99_9_us']:.3f} | "
            f"{d['p99_99_us']:.3f} | {d['max_us']:.3f} | "
            f"{d['jitter_p99_us']:.3f} |"
        )
    lines += [
        "",
        "## Behavior by corpus region",
        "",
        "| Region | samples | official command mismatches | live delta RMS m/s | live delta max m/s | Upkie p99 µs | Rust live p99 µs |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for name, region in metrics["regions"].items():
        lines.append(
            f"| {name} | {region['samples']:,} | "
            f"{region['official_command']['canonical_bit_mismatches']:,} | "
            f"{region['live_ground_velocity']['rms_error']:.4f} | "
            f"{region['live_ground_velocity']['maximum_absolute_error']:.4f} | "
            f"{region['latency']['upkie_cpp']['p99_us']:.3f} | "
            f"{region['latency']['bonesaw_live']['p99_us']:.3f} |"
        )
    lines += [
        "",
        "## Temporal drift by execution window",
        "",
        "| Window | samples | Upkie p50/p99 µs | Rust official p50/p99 µs | Rust live p50/p99 µs | live delta RMS/max m/s |",
        "|---:|---:|---:|---:|---:|---:|",
    ]
    for window in metrics["temporal_windows"]:
        lines.append(
            f"| {window['window']} | {window['start']:,}–{window['stop'] - 1:,} | "
            f"{window['upkie_p50_us']:.3f}/{window['upkie_p99_us']:.3f} | "
            f"{window['rust_official_p50_us']:.3f}/{window['rust_official_p99_us']:.3f} | "
            f"{window['rust_live_p50_us']:.3f}/{window['rust_live_p99_us']:.3f} | "
            f"{window['live_delta_rms_mps']:.4f}/{window['live_delta_max_mps']:.4f} |"
        )
    lines += [
        "",
        "## Process resources",
        "",
        "| Worker | wall s | CPU/wall | peak RSS | minor faults | major faults | voluntary ctx | involuntary ctx |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for name, label in [
        ("upkie_cpp", "Official Upkie C++"),
        ("bonesaw_official", "Bonesaw Rust · official gains"),
        ("bonesaw_live", "Bonesaw Rust · live gains"),
    ]:
        d = metrics["process"][name]
        lines.append(
            f"| {label} | {d['wall_seconds']:.4f} | {d['cpu_to_wall']:.3f} | "
            f"{fmt_bytes(d['peak_rss_bytes'])} | {d['minor_faults']:,} | "
            f"{d['major_faults']:,} | {d['voluntary_context_switches']:,} | "
            f"{d['involuntary_context_switches']:,} |"
        )
    allocation = metrics["bonesaw_hot_loop_allocations"]
    lines += [
        "",
        "## Allocation and artifacts",
        "",
        f"- Bonesaw official-aligned loop: **{allocation['official']['allocation_calls']} calls / "
        f"{allocation['official']['allocated_bytes']} bytes**.",
        f"- Bonesaw live-tuned loop: **{allocation['live']['allocation_calls']} calls / "
        f"{allocation['live']['allocated_bytes']} bytes**.",
        "- `upkie-controller-corpus.tsv`: exact sequential input corpus.",
        "- `upkie-controller-raw.npz`: both command traces, both Rust integral-state",
        "  traces, region labels, and all per-call latency samples.",
        "- `upkie-controller-metrics.json`: complete machine-readable metrics.",
        "",
        "## What this proves—and what it does not",
        "",
        "The official-aligned parity result proves that the shared PI controller law in",
        "Bonesaw has the same sequential floating-point behavior as the pinned upstream",
        "C++ implementation over this corpus. It does not claim that the surrounding",
        "systems are identical: Bonesaw lowers wheel velocity into bounded acceleration",
        "tasks and a lexicographic floating WBC, while Upkie writes servo velocity",
        "offsets. Full-body model products remain covered independently by Pinocchio,",
        "and the fixed-base task behavior remains compared with PlaCo.",
        "",
    ]
    return "\n".join(lines)


def parse_allocator(stderr: str) -> dict[str, int]:
    for line in reversed(stderr.splitlines()):
        try:
            value = json.loads(line)
        except json.JSONDecodeError:
            continue
        if "allocation_calls" in value:
            return {
                "allocation_calls": int(value["allocation_calls"]),
                "allocated_bytes": int(value["allocated_bytes"]),
            }
    raise ValueError(f"allocator report missing from worker stderr: {stderr}")


def main() -> None:
    args = parse_args()
    output = pathlib.Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    corpus_path = output / "upkie-controller-corpus.tsv"
    ground, pitch, regions = build_corpus(args.ticks)
    write_corpus(corpus_path, ground, pitch)

    upkie_output = output / "upkie-controller-cpp.tsv"
    rust_official_output = output / "upkie-controller-rust-official.tsv"
    rust_live_output = output / "upkie-controller-rust-live.tsv"
    process = {
        "upkie_cpp": run_worker(
            [str(pathlib.Path(args.upkie_worker).resolve()), str(corpus_path), str(upkie_output)]
        ),
        "bonesaw_official": run_worker(
            [
                str(pathlib.Path(args.rust_worker).resolve()),
                str(corpus_path),
                str(rust_official_output),
                str(pathlib.Path(args.model).resolve()),
                "official",
            ]
        ),
        "bonesaw_live": run_worker(
            [
                str(pathlib.Path(args.rust_worker).resolve()),
                str(corpus_path),
                str(rust_live_output),
                str(pathlib.Path(args.model).resolve()),
                "live",
            ]
        ),
    }
    upkie = load_trace(upkie_output)
    rust_official = load_trace(rust_official_output)
    rust_live = load_trace(rust_live_output)
    for trace in (upkie, rust_official, rust_live):
        if len(trace["ground_velocity"]) != args.ticks:
            raise RuntimeError("worker output length does not match corpus")

    source = pathlib.Path(args.upkie_source).resolve()
    commit = subprocess.check_output(
        ["git", "-C", str(source), "rev-parse", "HEAD"], text=True
    ).strip()
    cpu = platform.processor()
    if pathlib.Path("/proc/cpuinfo").exists():
        for line in pathlib.Path("/proc/cpuinfo").read_text().splitlines():
            if line.startswith("model name"):
                cpu = line.split(":", 1)[1].strip()
                break
    metrics: dict[str, Any] = {
        "schema": 1,
        "ticks": args.ticks,
        "dt_seconds": DT,
        "common_subset": {
            "floor_contact": True,
            "target_ground_velocity": 0.0,
            "target_yaw_velocity": 0.0,
            "target_ground_position": 0.0,
        },
        "provenance": {
            "upkie_commit": commit,
            "wheel_balancer_cpp_sha256": hash_file(
                source / "upkie/cpp/controllers/WheelBalancer.cpp"
            ),
            "wheel_balancer_h_sha256": hash_file(
                source / "upkie/cpp/controllers/WheelBalancer.h"
            ),
        },
        "environment": {
            "cpu": cpu,
            "platform": platform.platform(),
            "python": platform.python_version(),
        },
        "comparisons": {
            "official_aligned": comparison(
                np.concatenate(
                    [
                        upkie["left_wheel_velocity"],
                        upkie["right_wheel_velocity"],
                    ]
                ),
                np.concatenate(
                    [
                        rust_official["left_wheel_velocity"],
                        rust_official["right_wheel_velocity"],
                    ]
                ),
            ),
            "official_inferred_ground_velocity": comparison(
                upkie["ground_velocity"], rust_official["ground_velocity"]
            ),
            "live_tuned": comparison(
                upkie["ground_velocity"], rust_live["ground_velocity"]
            ),
            "official_left_wheel": comparison(
                upkie["left_wheel_velocity"],
                rust_official["left_wheel_velocity"],
            ),
            "official_right_wheel": comparison(
                upkie["right_wheel_velocity"],
                rust_official["right_wheel_velocity"],
            ),
        },
        "latency": {
            "upkie_cpp": distribution(upkie["elapsed_ns"]),
            "bonesaw_official": distribution(rust_official["elapsed_ns"]),
            "bonesaw_live": distribution(rust_live["elapsed_ns"]),
        },
        "process": {
            name: asdict(value) for name, value in process.items()
        },
        "bonesaw_hot_loop_allocations": {
            "official": parse_allocator(process["bonesaw_official"].stderr),
            "live": parse_allocator(process["bonesaw_live"].stderr),
        },
        "region_counts": {
            name: int(np.count_nonzero(regions == name))
            for name in np.unique(regions)
        },
        "regions": region_metrics(
            regions, upkie, rust_official, rust_live
        ),
        "temporal_windows": temporal_windows(
            upkie, rust_official, rust_live
        ),
    }
    np.savez_compressed(
        output / "upkie-controller-raw.npz",
        ground_position=ground,
        pitch=pitch,
        region=regions,
        upkie_ground_velocity=upkie["ground_velocity"],
        upkie_left_wheel_velocity=upkie["left_wheel_velocity"],
        upkie_right_wheel_velocity=upkie["right_wheel_velocity"],
        upkie_latency_ns=upkie["elapsed_ns"],
        rust_official_ground_velocity=rust_official["ground_velocity"],
        rust_official_integral_velocity=rust_official["integral_velocity"],
        rust_official_latency_ns=rust_official["elapsed_ns"],
        rust_live_ground_velocity=rust_live["ground_velocity"],
        rust_live_integral_velocity=rust_live["integral_velocity"],
        rust_live_latency_ns=rust_live["elapsed_ns"],
    )
    (output / "upkie-controller-metrics.json").write_text(
        json.dumps(metrics, indent=2) + "\n"
    )
    (output / "UPKIE_CONTROLLER_COMPARISON.md").write_text(
        render_report(metrics)
    )
    parity = metrics["comparisons"]["official_aligned"]
    if not parity["canonical_bitwise_equal"]:
        raise SystemExit(
            "official-aligned Rust output is not canonically bitwise equal "
            f"({parity['canonical_bit_mismatches']} mismatches)"
        )
    print(output / "UPKIE_CONTROLLER_COMPARISON.md")


if __name__ == "__main__":
    main()

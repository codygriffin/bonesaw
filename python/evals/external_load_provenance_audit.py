#!/usr/bin/env python3
"""End-to-end audit of typed external-load provenance at the live boundary."""

from __future__ import annotations

import argparse
import json
import pathlib
import statistics
import time
from datetime import datetime, timezone
from typing import Any

from cpu_reference_report import markdown_table, render_report_html
from live_editor_smoke import RawWebSocket, http_probe
from live_plant_gateway_smoke import body_position, receive_plant


REVISION = "external-load-provenance-r225"
DECLARED_CLASS = "declared_continuous_wrench"


def provenance(source: str, load_class: str = DECLARED_CLASS) -> dict[str, str]:
    return {
        "source": source,
        "load_class": load_class,
        "force_frame": "world",
        "application_point_frame": "world",
    }


def command(
    request_id: int,
    point: list[float],
    *,
    source: str = "evaluation_harness",
    load_class: str = DECLARED_CLASS,
    include_provenance: bool = True,
) -> dict[str, Any]:
    result: dict[str, Any] = {
        "type": "plant_push",
        "body": "base",
        "force_world": [1.0, 0.0, 0.0],
        "application_point_world": point,
        "request_id": request_id,
    }
    if include_provenance:
        result["provenance"] = provenance(source, load_class)
    return result


def receive_correlated(
    websocket: RawWebSocket,
    request_id: int,
    *,
    active: bool,
    attempts: int = 80,
) -> dict[str, Any]:
    for _ in range(attempts):
        state = receive_plant(websocket, "plant_state")
        if state.get("command_id") != request_id:
            continue
        if bool(state["external_load"]["active"]) != active:
            continue
        return state
    raise AssertionError(f"request {request_id} was not correlated")


def run_probe(base_url: str, connect_address: str | None = None) -> dict[str, Any]:
    http_probe(base_url, connect_address)
    websocket = RawWebSocket.connect(base_url, connect_address, "/plant-ws")
    try:
        hello = receive_plant(websocket, "plant_hello")
        initial = receive_plant(websocket, "plant_state")
        point = body_position(initial, "base")
        assert hello["protocol"] == 2, hello
        contract = hello["external_load_contract"]
        assert contract["executable_class"] == DECLARED_CLASS, contract

        rejected: list[dict[str, Any]] = []
        rejection_specs = [
            (701, None, False),
            (702, "measured_impact_impulse", True),
            (703, "unobserved_model_reserve", True),
        ]
        for request_id, load_class, include_provenance in rejection_specs:
            started = time.perf_counter_ns()
            websocket.send_json(
                command(
                    request_id,
                    point,
                    load_class=load_class or DECLARED_CLASS,
                    include_provenance=include_provenance,
                )
            )
            error = receive_plant(websocket, "plant_error")
            rejection_ms = (time.perf_counter_ns() - started) / 1.0e6
            survived = receive_plant(websocket, "plant_state")
            assert not survived["external_load"]["active"], survived
            assert survived.get("command_id") != request_id, survived
            rejected.append(
                {
                    "request_id": request_id,
                    "attempted_class": load_class or "missing_provenance",
                    "message": error["message"],
                    "rejection_ms": rejection_ms,
                    "stream_survived": True,
                    "worker_command_not_mutated": True,
                }
            )

        accepted: list[dict[str, Any]] = []
        for request_id, source in [
            (710, "evaluation_harness"),
            (720, "interactive_operator"),
        ]:
            started = time.perf_counter_ns()
            websocket.send_json(command(request_id, point, source=source))
            state = receive_correlated(websocket, request_id, active=True)
            ack_ms = (time.perf_counter_ns() - started) / 1.0e6
            observed = state["external_load"]
            assert observed["provenance"] == provenance(source), observed
            assert observed["force_world"] == [1.0, 0.0, 0.0], observed
            release_id = request_id + 1
            websocket.send_json({"type": "plant_release", "request_id": release_id})
            receive_correlated(websocket, release_id, active=False)
            accepted.append(
                {
                    "source": source,
                    "request_id": request_id,
                    "ack_ms": ack_ms,
                    "echo_exact": True,
                    "released": True,
                }
            )

        final = receive_plant(websocket, "plant_state")
        assert not final["external_load"]["active"]
        assert not final["measured_impact_impulse"]["available"]
        assert not final["unobserved_model_reserve"]["available"]
        return {
            "url": base_url,
            "protocol": hello["protocol"],
            "contract": contract,
            "rejected": rejected,
            "accepted": accepted,
            "maximum_rejection_ms": max(item["rejection_ms"] for item in rejected),
            "maximum_accept_ack_ms": max(item["ack_ms"] for item in accepted),
            "impact_impulse_separate": not final["measured_impact_impulse"]["available"],
            "model_reserve_separate": not final["unobserved_model_reserve"]["available"],
            "warning_count": final["simulator"]["warning_count"],
        }
    finally:
        websocket.close()


def make_report(metrics: dict[str, Any]) -> str:
    probe = metrics["probe"]
    rejection_rows = [
        [
            item["attempted_class"],
            item["message"],
            f'{item["rejection_ms"]:.3f}',
            item["stream_survived"],
            item["worker_command_not_mutated"],
        ]
        for item in probe["rejected"]
    ]
    accepted_rows = [
        [
            item["source"],
            DECLARED_CLASS,
            f'{item["ack_ms"]:.3f}',
            item["echo_exact"],
            item["released"],
        ]
        for item in probe["accepted"]
    ]
    return "\n".join(
        [
            f'# External-load provenance audit · {metrics["revision"]}',
            "",
            f'Admission: **{"PASS" if metrics["admission"] else "FAIL"}**. Protocol {probe["protocol"]} types declared external wrenches separately from measured impact impulse and unobserved-model reserve.',
            "",
            "## Rejected command categories",
            "",
            *markdown_table(
                ["attempt", "response", "reject ms", "stream", "worker unchanged"],
                rejection_rows,
            ),
            "",
            "## Accepted declared sources",
            "",
            *markdown_table(
                ["source", "class", "ack ms", "exact echo", "released"],
                accepted_rows,
            ),
            "",
            "The Rust gateway rejects missing provenance and refuses to execute evidence-only impact/reserve classes before mutating the worker command. The Python MuJoCo worker independently revalidates the executable class, source, and both world frames. Browser and evaluation sources are distinct and echoed in measured plant state.",
            "",
            "This is provenance and transport evidence, not an impact estimator, model-error calibration, authenticated operator identity, or a claim that an unobserved-model reserve is zero. Both unavailable quantities remain explicit unavailable records.",
            "",
        ]
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default="http://127.0.0.1:8777")
    parser.add_argument("--connect-address")
    parser.add_argument("--output", default=f"benchmarks/results/{REVISION}")
    parser.add_argument(
        "--web-report", default="web/EXTERNAL_LOAD_PROVENANCE_AUDIT_R225.html"
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    probe = run_probe(args.url.rstrip("/"), args.connect_address)
    admission = bool(
        probe["protocol"] == 2
        and all(item["stream_survived"] for item in probe["rejected"])
        and all(item["worker_command_not_mutated"] for item in probe["rejected"])
        and all(item["echo_exact"] and item["released"] for item in probe["accepted"])
        and probe["impact_impulse_separate"]
        and probe["model_reserve_separate"]
        and probe["warning_count"] == 0
    )
    metrics = {
        "schema_version": 1,
        "revision": REVISION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "probe": probe,
        "ack_ms": {
            "accepted_p50": statistics.median(
                item["ack_ms"] for item in probe["accepted"]
            ),
            "accepted_max": probe["maximum_accept_ack_ms"],
            "rejected_max": probe["maximum_rejection_ms"],
        },
        "admission": admission,
    }
    output = pathlib.Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    metrics_path = output / "external-load-provenance-metrics.json"
    report_path = output / "EXTERNAL_LOAD_PROVENANCE_AUDIT.md"
    metrics_path.write_text(json.dumps(metrics, indent=2) + "\n")
    report = make_report(metrics)
    report_path.write_text(report)
    pathlib.Path(args.web_report).write_text(
        render_report_html(report, title="External-load provenance")
    )
    print(
        json.dumps(
            {
                "admission": admission,
                "metrics": str(metrics_path),
                "report": str(report_path),
            },
            indent=2,
        )
    )
    return 0 if admission else 1


if __name__ == "__main__":
    raise SystemExit(main())

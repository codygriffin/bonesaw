#!/usr/bin/env python3
"""Planner-only reduced-iteration gate for the lateral multistep candidate."""

from upkie_budgeted_multistep_viability_ab import main


REVISION = "upkie-lateral-anytime-multistep-viability-ab-r153"


if __name__ == "__main__":
    main(
        revision=REVISION,
        planner_strategy="lateral_anytime_multistep_budgeted",
        web_report="web/UPKIE_LATERAL_ANYTIME_MULTISTEP_VIABILITY_AB_R153.html",
        report_title="Bonesaw lateral-authority anytime multistep viability plant A/B · r153",
        activation_description=(
            "Only roll/lateral capture pressure wakes the planner. Planner-only WBC "
            "polls use at most eight hard-feasibility iterations and may return no "
            "candidate; the final execution WBC retains the full 64-iteration budget."
        ),
        metrics_name="upkie-lateral-anytime-multistep-viability-ab-metrics.json",
        audit_name="UPKIE_LATERAL_ANYTIME_MULTISTEP_VIABILITY_AB_AUDIT.md",
    )

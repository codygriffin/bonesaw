#!/usr/bin/env python3
"""Lateral-authority activation gate for the r151 multistep planner."""

from upkie_budgeted_multistep_viability_ab import main


REVISION = "upkie-lateral-budgeted-multistep-viability-ab-r152"


if __name__ == "__main__":
    main(
        revision=REVISION,
        planner_strategy="lateral_multistep_budgeted",
        web_report="web/UPKIE_LATERAL_BUDGETED_MULTISTEP_VIABILITY_AB_R152.html",
        report_title="Bonesaw lateral-authority budgeted multistep viability plant A/B · r152",
        activation_description=(
            "Only current roll/lateral capture pressure may wake the planner. "
            "Sagittal, support-mask, rate, yaw, resource, action, and action-change "
            "pressures remain separate visible proposal vetoes; they are not wake-up authority."
        ),
        metrics_name="upkie-lateral-budgeted-multistep-viability-ab-metrics.json",
        audit_name="UPKIE_LATERAL_BUDGETED_MULTISTEP_VIABILITY_AB_AUDIT.md",
    )

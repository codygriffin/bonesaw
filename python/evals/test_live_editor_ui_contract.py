from __future__ import annotations

import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[2]


class LiveEditorUiContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.html = (ROOT / "web/index.html").read_text()
        cls.css = (ROOT / "web/styles.css").read_text()
        cls.javascript = (ROOT / "web/motion-rig-r16.js").read_text()

    def test_disconnect_ghost_and_overlay_are_wired(self) -> None:
        self.assertIn('id="disconnect-overlay"', self.html)
        self.assertIn(".viewport.disconnected #rig-canvas", self.css)
        self.assertIn("grayscale(1)", self.css)
        self.assertIn('viewport.classList.toggle("disconnected", !enabled)', self.javascript)
        self.assertIn("disconnectOverlay.hidden = enabled", self.javascript)

    def test_connection_loss_disables_every_robot_control(self) -> None:
        for control in (
            '#reset-button',
            '#geometry-toggle',
            '#rig-toggle',
        ):
            self.assertIn(f'document.querySelector("{control}")', self.javascript)
        self.assertIn("targetTool.disabled = !robotControlsEnabled", self.javascript)
        self.assertIn(
            "pushTool.disabled = !robotControlsEnabled || !plantGateway?.available",
            self.javascript,
        )
        self.assertIn("button.disabled = !robotControlsEnabled || pushing", self.javascript)
        self.assertIn("control.disabled = !enabled", self.javascript)
        self.assertIn("performance.now() - lastSocketMessageAt > 1500", self.javascript)
        self.assertIn("if (robotControlsEnabled && event.key.toLowerCase()", self.javascript)

    def test_plant_only_disconnect_clears_live_state_and_keeps_a_ghost(self) -> None:
        self.assertIn(
            "disconnectPlant({ preserveGhost: true, closeSocket: false })",
            self.javascript,
        )
        self.assertIn("function resetPlantTelemetry(status = \"disconnected · ghost\")", self.javascript)
        self.assertIn("simulatorState.textContent = \"awaiting MuJoCo\"", self.javascript)
        self.assertIn("groundContactState.textContent = \"awaiting contact state\"", self.javascript)
        self.assertIn("plantWrench.textContent = \"unavailable\"", self.javascript)
        self.assertIn("connectionLabel.textContent = \"Plant reconnecting\"", self.javascript)
        self.assertIn("setRobotControlsEnabled(false)", self.javascript)
        self.assertIn("if (!preserveGhost)", self.javascript)
        self.assertIn("measuredPlantFrames = []", self.javascript)
        self.assertIn("let plantStateFresh = false", self.javascript)
        self.assertIn("plantStateFresh = true", self.javascript)
        self.assertIn("const stalePlantSnapshot = latestSnapshot.message.source === \"plant\"", self.javascript)
        self.assertIn("const livePlantOwnsController = plantConnected && plantStateFresh", self.javascript)
        self.assertIn("? plantPresentationMetrics(plantState)", self.javascript)
        self.assertIn("if (!robotControlsEnabled && socket?.readyState === WebSocket.OPEN)", self.javascript)

    def test_handles_are_visible_and_only_visible_frames_are_hit_tested(self) -> None:
        self.assertIn('id="target-guide"', self.html)
        self.assertIn("GREEN CONTROLS", self.html)
        self.assertIn("interactionHandles.size", self.javascript)
        self.assertIn('pushing ? "#ff9d45" : "#5ee6a5"', self.javascript)
        self.assertIn("for (const handle of interactionHandles.values())", self.javascript)
        self.assertIn("if (!frame) continue", self.javascript)

    def test_controls_enable_only_after_hello_schema_arrives(self) -> None:
        main_connect = self.javascript[self.javascript.index("function connect()") :]
        open_handler = main_connect.index('socket.addEventListener("open"')
        message_handler = main_connect.index('socket.addEventListener("message"')
        hello_handler = main_connect.index('if (message.type === "hello")')
        enabled = main_connect.index("setRobotControlsEnabled(true)")
        self.assertLess(open_handler, message_handler)
        self.assertLess(message_handler, hello_handler)
        self.assertLess(hello_handler, enabled)

    def test_physical_push_is_lazy_bounded_and_fail_safe(self) -> None:
        self.assertIn('id="push-tool"', self.html)
        self.assertIn("ORANGE WRENCH", self.javascript)
        self.assertIn('new WebSocket(', self.javascript)
        self.assertIn('plantGateway.websocket_path || "/plant-ws"', self.javascript)
        self.assertIn("PUSH_FORCE_GAIN_N_PER_M", self.javascript)
        self.assertIn("magnitude > limit ? limit / magnitude : 1", self.javascript)
        self.assertIn("maximum_application_offset_m", self.html + self.javascript)
        self.assertIn("now - lastPushSentAt >= 50", self.javascript)
        self.assertIn('sendPlant({ type: "plant_release" })', self.javascript)
        self.assertIn("command_expired", self.javascript)
        self.assertIn("maximum_moment_nm", self.javascript)
        self.assertIn("N·m", self.javascript)
        self.assertIn('metrics.fallen', self.javascript)
        self.assertIn('"Plant fell · automatic reset armed"', self.javascript)
        self.assertIn('if (!plantGateway?.available || interactionMode !== "push") return', self.javascript)
        self.assertIn("setTimeout(connectPlant, 800)", self.javascript)

    def test_every_green_handle_release_commits_a_measured_target_without_overloading_push(self) -> None:
        self.assertIn('id="plant-command-state"', self.html)
        self.assertIn('type: "plant_frame_target_commit"', self.javascript)
        self.assertIn("handle_id: handle.handle_id", self.javascript)
        self.assertIn("target: { position_world_m: [...positionWorldM] }", self.javascript)
        self.assertIn("release any control to command MuJoCo", self.javascript)
        self.assertIn("flushTargetCommit", self.javascript)
        self.assertIn('target_command_contract', (ROOT / "python/evals/upkie_live_plant_worker.py").read_text())
        self.assertIn('"target_command"', (ROOT / "crates/bonesaw-tools/src/bin/server.rs").read_text())
        self.assertIn("`holding`", (ROOT / "docs/LIVE_PLANT_INTENT_WRENCH_CONTRACT.md").read_text())
        self.assertNotIn('kind === "base"', self.javascript)
        self.assertNotIn("Cartesian base target", self.javascript)
        self.assertIn("<dt>Frame targets</dt>", self.html)

    def test_pointer_completion_is_uniform_and_cancel_never_commits(self) -> None:
        self.assertIn("const committedHandle = interactionHandles.get(committedFrame)", self.javascript)
        self.assertIn("if (!cancelled && committedHandle)", self.javascript)
        self.assertIn("commitTarget(committedHandle, committedTarget)", self.javascript)
        self.assertIn(
            'canvas.addEventListener("pointercancel", (event) => finishPointer(event, true))',
            self.javascript,
        )
        cancel_guard = self.javascript.index("if (!cancelled && committedHandle)")
        commit = self.javascript.index("commitTarget(committedHandle, committedTarget)")
        self.assertLess(cancel_guard, commit)

    def test_push_mode_never_runs_target_handle_hit_testing(self) -> None:
        pointer_down = self.javascript[
            self.javascript.index('canvas.addEventListener("pointerdown"') :
            self.javascript.index('canvas.addEventListener("pointermove"')
        ]
        push_empty = pointer_down.index('if (interactionMode === "push")')
        target_pick = pointer_down.index("const frame = nearestFrame(event)")
        self.assertLess(push_empty, target_pick)
        self.assertIn("beginOrbit(event);\n    return;", pointer_down)
        self.assertIn(
            'interactionMode === "push" || event.ctrlKey',
            self.javascript,
        )

    def test_target_telemetry_and_marker_update_while_preview_is_primary(self) -> None:
        enqueue = self.javascript[
            self.javascript.index("function enqueuePlantState(message)") :
            self.javascript.index("function connectPlant()")
        ]
        self.assertIn("updatePlantTelemetry(message)", enqueue)
        self.assertIn("const admittedTarget = targetTelemetry.admitted_position_world", enqueue)
        self.assertIn("activePlantTarget = {", enqueue)
        self.assertIn("phase: targetPhase", enqueue)
        self.assertIn("intentStatus: String(targetTelemetry.intent_status", enqueue)
        self.assertIn("function drawPlantTargetMarker()", self.javascript)
        self.assertIn("phaseLabel", self.javascript)
        self.assertIn("progressLabel", self.javascript)
        self.assertIn("measured error", self.javascript)
        self.assertNotIn('if (interactionMode === "push") updatePlantTelemetry', enqueue)

    def test_committed_preview_and_admitted_marker_persist_after_release(self) -> None:
        finish = self.javascript[
            self.javascript.index("function finishPointer") :
            self.javascript.index('canvas.addEventListener("pointerdown"')
        ]
        commit = finish.index("commitTarget(committedHandle, committedTarget)")
        cancel_release = finish.index('send({ type: "release" })', commit)
        self.assertLess(commit, cancel_release)
        self.assertIn("positionWorldM: [...admittedTarget]", self.javascript)

    def test_ctrl_mesh_wrench_and_shift_pan_are_distinct(self) -> None:
        self.assertIn("if (event.ctrlKey) {", self.javascript)
        self.assertIn("pickRenderedBody(event)", self.javascript)
        self.assertIn("rayTriangleDistance", self.javascript)
        self.assertIn("applicationPoint = pick.point", self.javascript)
        self.assertIn('pushReturnMode = interactionMode', self.javascript)
        self.assertIn('if (event.shiftKey) {', self.javascript)
        self.assertIn("camera.target = camera.target.map", self.javascript)
        self.assertIn("Ctrl+drag any body to wrench", self.javascript)
        self.assertNotIn("event.shiftKey && plantGateway?.available", self.javascript)

    def test_mujoco_contact_and_rate_state_are_rendered(self) -> None:
        self.assertIn('id="simulator-state"', self.html)
        self.assertIn('id="plant-com-state"', self.html)
        self.assertIn('id="plant-ground-state"', self.html)
        self.assertIn('id="ground-contact-state"', self.html)
        self.assertIn('id="contact-cadence-state"', self.html)
        self.assertIn('id="contact-load-state"', self.html)
        self.assertIn('id="runtime-rates"', self.html)
        self.assertIn("drawPlantContactLayer", self.javascript)
        self.assertIn("message.contacts || []", self.javascript)
        self.assertIn("metrics.maximum_penetration_m", self.javascript)
        self.assertIn("message.physics_substeps_per_control", self.javascript)
        self.assertIn("drawMeasuredPlantCollisionLayer", self.javascript)
        self.assertIn("collisionGeometry = (message.geometry || [])", self.javascript)
        self.assertIn("ground_plane_point_world", self.javascript)
        self.assertIn("MUJOCO GROUND", self.javascript)
        self.assertIn("message.center_of_mass_world", self.javascript)
        self.assertIn("solver_forward_inverse", self.javascript)
        self.assertIn("wbc_observation", self.javascript)
        self.assertIn("contact_window_masks", self.javascript)
        self.assertIn("contact_window_loss_mask", self.javascript)
        self.assertIn("contact_window_gain_mask", self.javascript)
        self.assertIn("wbc_observed_wheel_normal_force_n", self.javascript)
        self.assertIn("wbc_predicted_normal_force_n", self.javascript)
        self.assertIn("wbc_hard_contact_executable", self.javascript)

    def test_support_contingency_is_a_separate_visible_authority_witness(self) -> None:
        self.assertIn("wbc_support_contingency_enabled", self.javascript)
        self.assertIn("wbc_support_contingency_selected", self.javascript)
        self.assertIn("wbc_support_contingency_admitted", self.javascript)
        self.assertIn("wbc_support_contingency_requested", self.javascript)
        self.assertIn('"CTG"', self.javascript)
        self.assertIn("contingencyState", self.javascript)

    def test_mujoco_pause_resume_reset_controls_are_explicit(self) -> None:
        for selector in ('#pause-button', '#resume-button', '#reset-button'):
            self.assertIn(f'id="{selector[1:]}"', self.html)
        self.assertIn('type: "plant_pause"', self.javascript)
        self.assertIn('type: "plant_resume"', self.javascript)
        self.assertIn('if (plantConnected) sendPlant({ type: "plant_reset" });', self.javascript)
        self.assertIn('if (interactionMode !== "push") send({ type: "reset" });', self.javascript)
        self.assertIn('plantPaused = Boolean(message.paused', self.javascript)
        self.assertIn('pauseButton.disabled = !robotControlsEnabled', self.javascript)
        self.assertIn('resumeButton.disabled = !robotControlsEnabled', self.javascript)
        self.assertIn('pushTool.disabled = !robotControlsEnabled || !plantGateway?.available || plantPaused', self.javascript)
        self.assertIn('"paused": self.paused', (ROOT / 'python/evals/upkie_live_plant_worker.py').read_text())

    def test_live_plant_owns_controller_evidence_in_every_interaction_mode(self) -> None:
        self.assertIn('source: "plant"', self.javascript)
        self.assertIn('latestSnapshot.message.source === "plant"', self.javascript)
        self.assertIn("function plantPresentationMetrics(message)", self.javascript)
        self.assertIn("livePlantOwnsController", self.javascript)
        self.assertIn("latestPreviewMetrics", self.javascript)
        self.assertIn("updatePlantTelemetry(plantState)", self.javascript)
        self.assertIn('const telemetrySource = livePlantOwnsController ? "plant" : "preview"', self.javascript)
        self.assertIn("if (plantConnected && plantStateFresh && plantState?.metrics)", self.javascript)
        self.assertIn('"authority-capture"', self.javascript)
        self.assertIn('"authority-station"', self.javascript)
        self.assertIn("drawTinyAuthorityBar", self.javascript)

    def test_measured_mujoco_rig_is_drawn_over_the_target_preview(self) -> None:
        draw = self.javascript[
            self.javascript.index("function draw()") :
            self.javascript.index("function pointerRay")
        ]
        self.assertLess(draw.index("drawGeometryLayer()"), draw.index("drawMeasuredPlantLayer()"))
        self.assertIn('context.strokeStyle = "rgba(255,173,91,0.94)"', self.javascript)
        self.assertIn('"rgba(255,157,69,0.68)"', self.javascript)

    def test_live_wbc_failures_remain_separate_visible_authority_rows(self) -> None:
        worker = (ROOT / "python/evals/upkie_live_plant_worker.py").read_text()
        for signal in (
            "wbc_raw_status",
            "wbc_allocation_calls",
            "wbc_allocated_bytes",
            "wbc_maximum_constraint_violation",
            "wbc_dynamics_residual",
            "wbc_contact_residual",
        ):
            self.assertIn(signal, worker)
            self.assertIn(signal, self.javascript)
        for authority_row in (
            '"authority-hard"',
            '"authority-support"',
            '"authority-actuator"',
            '"authority-solver"',
        ):
            self.assertIn(authority_row, self.javascript)
        self.assertIn('"RES"', self.javascript)
        self.assertIn('metrics.wbc_raw_status === "MaxIterations"', self.javascript)


if __name__ == "__main__":
    unittest.main()

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
        self.assertIn("plantStatus.textContent = \"disconnected · ghost\"", self.javascript)
        self.assertIn("plantWrench.textContent = \"unavailable\"", self.javascript)
        self.assertIn("connectionLabel.textContent = \"Plant reconnecting\"", self.javascript)
        self.assertIn("setRobotControlsEnabled(false)", self.javascript)
        self.assertIn("if (!preserveGhost)", self.javascript)
        self.assertIn("measuredPlantFrames = []", self.javascript)

    def test_handles_are_visible_and_only_visible_frames_are_hit_tested(self) -> None:
        self.assertIn('id="target-guide"', self.html)
        self.assertIn("GREEN CONTROLS", self.html)
        self.assertIn("interactionHandles.size", self.javascript)
        self.assertIn('pushing ? "#ff9d45" : "#5ee6a5"', self.javascript)
        self.assertIn("if (!showRig) return null", self.javascript)

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

    def test_plant_state_owns_rendering_and_authority_in_push_mode(self) -> None:
        self.assertIn('source: "plant"', self.javascript)
        self.assertIn('latestSnapshot.message.source === "plant"', self.javascript)
        self.assertIn("updatePlantTelemetry(plantState)", self.javascript)
        self.assertIn('"authority-capture"', self.javascript)
        self.assertIn('"authority-station"', self.javascript)
        self.assertIn("drawTinyAuthorityBar", self.javascript)


if __name__ == "__main__":
    unittest.main()

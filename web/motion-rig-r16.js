// Browser adapter for architecture revision r235.
const canvas = document.querySelector("#rig-canvas");
const context = canvas.getContext("2d");
const viewport = document.querySelector(".viewport");
const disconnectOverlay = document.querySelector("#disconnect-overlay");
const sparkline = document.querySelector("#sparkline");
const spark = sparkline.getContext("2d");
const authorityHistoryCanvas = document.querySelector("#authority-history");
const authorityHistoryContext = authorityHistoryCanvas.getContext("2d");
const connection = document.querySelector(".connection");
const connectionLabel = document.querySelector("#connection-label");
const toast = document.querySelector("#toast");
const architectureButton = document.querySelector("#architecture-button");
const architectureFooterButton = document.querySelector("#architecture-footer-button");
const architectureClose = document.querySelector("#architecture-close");
const architectureReview = document.querySelector("#architecture-review");
const architectureScrim = document.querySelector("#architecture-scrim");
const architectureContent = document.querySelector("#architecture-content");
const architectureMobileLayout = window.matchMedia("(max-width: 520px)");
const viewModeLabel = document.querySelector("#view-mode-label");
const viewAxesLabel = document.querySelector("#view-axes-label");
const mobileOpenArchitectureSections = new Set([
  "architecture-flow",
  "architecture-authority",
  "architecture-gaps",
  "architecture-feedback",
]);
let architectureMobileState = architectureMobileLayout.matches;
const observationTransportButtons = [...document.querySelectorAll("[data-observation-mode]")];
const observationTransportDetail = document.querySelector("#observation-transport-detail");
const targetTool = document.querySelector("#target-tool");
const pushTool = document.querySelector("#push-tool");
const pauseButton = document.querySelector("#pause-button");
const resumeButton = document.querySelector("#resume-button");
const viewportInstruction = document.querySelector("#viewport-instruction");
const targetGuide = document.querySelector("#target-guide");
const plantStatus = document.querySelector("#plant-status");
const simulatorState = document.querySelector("#simulator-state");
const plantRootState = document.querySelector("#plant-root-state");
const plantComState = document.querySelector("#plant-com-state");
const plantMotionState = document.querySelector("#plant-motion-state");
const plantEffortState = document.querySelector("#plant-effort-state");
const plantConstraintState = document.querySelector("#plant-constraint-state");
const previewGroundState = document.querySelector("#preview-ground-state");
const plantGroundState = document.querySelector("#plant-ground-state");
const groundContactState = document.querySelector("#ground-contact-state");
const contactCadenceState = document.querySelector("#contact-cadence-state");
const contactLoadState = document.querySelector("#contact-load-state");
const runtimeRates = document.querySelector("#runtime-rates");
const plantWrench = document.querySelector("#plant-wrench");
const plantWrenchLimit = document.querySelector("#plant-wrench-limit");

let socket;
let plantSocket;
let robotControlsEnabled = false;
let plantConnected = false;
let plantGateway = null;
let plantHello = null;
let plantState = null;
// A hello only describes the stream schema.  Controls may be re-enabled after
// a reconnect once a post-reconnect plant_state has actually arrived.
let plantStateFresh = false;
let plantPaused = false;
let interactionMode = "target";
let pushReturnMode = null;
let pushDrag = null;
let pendingPushCommand = null;
let activeForceArrow = null;
let plantContacts = [];
let measuredPlantFrames = [];
let renderedMinimumGroundClearanceM = Number.NaN;
let measuredPlantMinimumGroundClearanceM = Number.NaN;
let simulatorGroundPlane = {
  point: [0, 0, 0],
  normal: [0, 0, 1],
  source: "awaiting MuJoCo",
};
let lastPreviewGroundUpdateMs = -Infinity;
let lastPushSentAt = -Infinity;
let lastSocketMessageAt = 0;
let frames = [];
let bones = [];
let geometry = [];
let collisionGeometry = [];
let supportPatches = [];
let worldSdfPlanes = [];
let latestMetrics = null;
let frameNames = [];
let bodyNames = [];
let coordinateNames = [];
let actuatorNames = [];
let actuatorResourceModels = [];
let interactionHandles = new Map();
let selected = null;
let drag = null;
let orbitDrag = null;
let showGeometry = true;
let showRig = false;
let solveHistory = [];
let commandAuthorityHistory = [];
let transform = { centerX: 0, centerY: 0, focalPixels: 500 };
const camera = {
  // A real perspective inspection camera. The orbit target stays in the
  // robot's torso volume while yaw/pitch move the camera around it.
  yaw: -Math.PI / 6,
  pitch: -Math.PI / 14,
  distance: 1.85,
  verticalFov: 42 * Math.PI / 180,
  target: [0, 0, 0.43],
  position: [1, 0, 0.43],
  right: [0, -1, 0],
  up: [0, 0, 1],
  forward: [-1, 0, 0],
};
let previousSnapshot = null;
let latestSnapshot = null;
let renderScheduled = false;
let pendingDragCommand = null;
let lastTelemetryUpdateMs = -Infinity;
let lastTelemetryTick = null;
let snapshotPeriodMs = 20;
let lastInteractionNotice = null;
const TELEMETRY_INTERVAL_MS = 100;
const MAX_VIEWPORT_PIXEL_RATIO = 2;
const performancePanel = document.querySelector("#viewport-performance");
const performanceEnabled = new URLSearchParams(location.search).has("perf");
const viewportPerformance = {
  lastFrameAt: null,
  lastPanelAt: -Infinity,
  frameIntervals: [],
  drawDurations: [],
  geometryDurations: [],
  snapshotIntervals: [],
  commandRoundTrips: [],
  telemetryDurations: [],
  coalescedStates: 0,
  coalescedDrags: 0,
  transmittedDrags: 0,
  pendingCameraInputAt: null,
  cameraLatencies: [],
  lastDragSentAt: null,
  lastDragFrame: null,
  lastDragCommandId: null,
};
let commandRequestId = 0;
let plantRequestId = 0;
let architectureManifest = null;
let architectureSerialized = null;
let baseExecution = "raw_dynamic";
const meshSurfaceCache = new Map();
const priorityNames = ["Invariant", "Viability", "Intent", "Preference", "Style"];
const liveTaskLevelElements = new Map();
const authorityPersistence = new Map();
let authorityCapabilities = new Map();
let authorityThresholds = {
  schema: 1,
  source: "browser-fallback-r54",
  hard_residual: { warning: 1e-9, critical: 1e-8 },
  support_margin_m: null,
  joint_margin_rad: { warning: 5 * Math.PI / 180, critical: 0 },
  joint_stopping_margin_rad_s2: { warning: 20, critical: 0 },
  actuator_utilization: { warning: 0.8, critical: 1 },
  solver_wall_time_us: { warning: 16000, critical: 20000 },
};
const PUSH_FORCE_GAIN_N_PER_M = 60;

function baseExecutionLabel() {
  return baseExecution === "guided_preview"
    ? "Cartesian base target · WBC preview"
    : "Cartesian base target · Rust WBC";
}

function updateInteractionUi() {
  const pushing = interactionMode === "push";
  targetTool.classList.toggle("active", !pushing);
  pushTool.classList.toggle("active", pushing);
  targetTool.setAttribute("aria-pressed", String(!pushing));
  pushTool.setAttribute("aria-pressed", String(pushing));
  viewport.classList.toggle("plant-mode", pushing);
  viewport.classList.toggle("push-mode", pushing);
  targetGuide.classList.toggle("push-guide", pushing);
  targetGuide.querySelector("span").innerHTML = pushing
    ? "<strong>ORANGE WRENCH</strong> · Ctrl+drag any rendered body"
    : `<strong>${interactionHandles.size} GREEN PREVIEW CONTROLS</strong> · orange dashed rig is measured MuJoCo`;
  viewportInstruction.textContent = pushing
    ? "WRENCH: Ctrl+drag any body · empty drag orbits · Shift+drag pans · wheel zooms"
    : "TARGET: drag green controls · Ctrl+drag any body to wrench · Shift+drag pans · wheel zooms";
  targetTool.disabled = !robotControlsEnabled;
  pushTool.disabled = !robotControlsEnabled || !plantGateway?.available || plantPaused;
  observationTransportButtons.forEach((button) => {
    button.disabled = !robotControlsEnabled || pushing;
  });
  pauseButton.disabled = !robotControlsEnabled || !plantConnected || plantPaused;
  resumeButton.disabled = !robotControlsEnabled || !plantConnected || !plantPaused;
}

function sendPlant(message) {
  if (plantSocket?.readyState !== WebSocket.OPEN) return false;
  const requestId = message.request_id ?? ++plantRequestId;
  plantSocket.send(JSON.stringify({ ...message, request_id: requestId }));
  return requestId;
}

function resetPlantTelemetry(status = "disconnected · ghost") {
  plantStatus.textContent = status;
  simulatorState.textContent = "awaiting MuJoCo";
  plantRootState.textContent = "awaiting MuJoCo";
  plantComState.textContent = "awaiting MuJoCo";
  plantMotionState.textContent = "awaiting MuJoCo";
  plantEffortState.textContent = "awaiting MuJoCo";
  plantConstraintState.textContent = "awaiting MuJoCo";
  runtimeRates.textContent = "awaiting MuJoCo";
  plantGroundState.textContent = "awaiting simulator plane";
  groundContactState.textContent = "awaiting contact state";
  contactCadenceState.textContent = "awaiting contact cadence";
  contactLoadState.textContent = "awaiting wheel loads";
  plantWrench.textContent = "unavailable";
  for (const [id, label] of [
    ["authority-capture", "awaiting live MuJoCo state"],
    ["authority-fall-safe", "awaiting prior-command lease evidence"],
    ["authority-station", "awaiting live MuJoCo state"],
  ]) {
    setLiveAuthorityRow(id, "UNAVAILABLE", label, 0, "unavailable");
  }
}

function disconnectPlant({ preserveGhost = false, closeSocket = true } = {}) {
  plantConnected = false;
  plantHello = null;
  plantStateFresh = false;
  plantContacts = [];
  plantState = null;
  plantPaused = false;
  if (!preserveGhost) {
    measuredPlantFrames = [];
    measuredPlantMinimumGroundClearanceM = Number.NaN;
  }
  simulatorGroundPlane = {
    point: [0, 0, 0],
    normal: [0, 0, 1],
    source: "MuJoCo disconnected",
  };
  activeForceArrow = null;
  pendingPushCommand = null;
  pushDrag = null;
  pushReturnMode = null;
  if (closeSocket && plantSocket) {
    plantSocket.onclose = null;
    plantSocket.close();
    plantSocket = null;
  }
  resetPlantTelemetry();
  updateInteractionUi();
}

function plantFrames(message) {
  const source = new Map((message.frames || []).map((frame) => [frame.name, frame]));
  const current = new Map(
    (latestSnapshot?.frames?.length ? latestSnapshot.frames : frames).map(
      (frame) => [frame.name, frame],
    ),
  );
  return frameNames.map((name, id) => {
    const physical = source.get(name);
    if (physical) {
      const rotation = physical.rotation;
      return {
        id,
        name,
        translation: physical.translation,
        rotation_xyzw: [rotation[1], rotation[2], rotation[3], rotation[0]],
      };
    }
    const fallback = current.get(name);
    return fallback || {
      id,
      name,
      translation: [0, 0, 0],
      rotation_xyzw: [0, 0, 0, 1],
    };
  });
}

function enqueuePlantState(message) {
  const firstPlantState = plantState === null;
  plantStateFresh = true;
  plantState = message;
  plantPaused = Boolean(message.paused ?? message.simulator?.paused ?? message.metrics?.paused);
  plantContacts = message.contacts || [];
  measuredPlantFrames = plantFrames(message);
  const simulator = message.simulator || {};
  const groundPoint = simulator.ground_plane_point_world;
  const groundNormal = simulator.ground_plane_normal_world;
  if (Array.isArray(groundPoint) && groundPoint.length === 3
      && groundPoint.every(Number.isFinite)
      && Array.isArray(groundNormal) && groundNormal.length === 3
      && groundNormal.every(Number.isFinite)) {
    simulatorGroundPlane = {
      point: [...groundPoint],
      normal: [...groundNormal],
      source: "MuJoCo model",
    };
  }
  if (firstPlantState && interactionMode === "push") {
    previousSnapshot = null;
    latestSnapshot = null;
  }
  const metrics = message.metrics || {};
  if (interactionMode === "push") {
    enqueueState({
      type: "state",
      source: "plant",
      tick: message.tick,
      reset_epoch: message.reset_epoch,
      command_id: message.command_id,
      active_frame: message.external_load?.active ? message.external_load.body : null,
      frames: measuredPlantFrames,
      metrics: {
        solve_us: metrics.controller_step_us,
        guided_preview_wbc_admitted: metrics.wbc_admitted,
        interaction_target_clamped: false,
        maximum_torque_utilization: metrics.torque_utilization,
        minimum_support_margin_m: Number.NaN,
        minimum_joint_margin_rad: Number.NaN,
        minimum_joint_stopping_margin_rad_s2: Number.NaN,
        center_of_mass_world: message.center_of_mass_world || message.root_position,
      },
    });
  }
  const stateLabel = plantPaused
    ? "MuJoCo · paused"
    : metrics.fallen
    ? "FALL · RESET ARMED"
    : metrics.wbc_admitted ? "MuJoCo · admitted" : `MuJoCo · ${metrics.wbc_status}`;
  plantStatus.textContent = stateLabel;
  const force = message.external_load?.force_world || [0, 0, 0];
  const magnitude = Math.hypot(...force);
  const loadSource = message.external_load?.provenance?.source || "unavailable";
  plantWrench.textContent = message.external_load?.active
    ? `${magnitude.toFixed(2)} N · ${Number(message.external_load.maximum_moment_nm || 0).toFixed(2)} N·m · ${message.external_load.body} · ${loadSource.replaceAll("_", " ")}`
    : message.command_expired ? "expired · fail-safe release" : "released";
  updatePlantTelemetry(message);
  if (message.automatic_reset_reason) {
    showToast(`Plant reset after ${message.automatic_reset_reason}`);
  } else if (message.numeric_reset) {
    showToast("Plant numeric fault reset explicitly");
  } else if (metrics.fallen) {
    showToast("Plant fell · automatic reset armed");
    sendPlant({ type: "plant_release" });
    pushDrag = null;
    pendingPushCommand = null;
    activeForceArrow = null;
  }
  if (!robotControlsEnabled && socket?.readyState === WebSocket.OPEN) {
    setRobotControlsEnabled(true);
  }
  scheduleRender();
  updateInteractionUi();
}

function connectPlant() {
  if (!plantGateway?.available || socket?.readyState !== WebSocket.OPEN) return;
  if (plantSocket?.readyState === WebSocket.CONNECTING
      || plantSocket?.readyState === WebSocket.OPEN) return;
  const protocol = location.protocol === "https:" ? "wss:" : "ws:";
  plantStatus.textContent = "starting MuJoCo";
  plantSocket = new WebSocket(
    `${protocol}//${location.host}${plantGateway.websocket_path || "/plant-ws"}`,
  );
  plantSocket.onmessage = (event) => {
    const message = JSON.parse(event.data);
    if (message.type === "plant_hello") {
      plantHello = message;
      plantConnected = true;
      plantStateFresh = false;
      plantPaused = Boolean(message.paused);
      const simulator = message.simulator || {};
      if (Array.isArray(simulator.ground_plane_point_world)
          && Array.isArray(simulator.ground_plane_normal_world)) {
        simulatorGroundPlane = {
          point: [...simulator.ground_plane_point_world],
          normal: [...simulator.ground_plane_normal_world],
          source: "MuJoCo model",
        };
      }
      plantStatus.textContent = `${message.control_hz} Hz WBC · ${message.stream_hz} Hz stream`;
      simulatorState.textContent = `${simulator.backend || "MuJoCo"} ${simulator.version || ""} · ${simulator.integrator || "unknown integrator"}`.trim();
      runtimeRates.textContent = `${message.control_hz} / ${message.physics_hz} Hz · ${message.physics_substeps_per_control} substeps`;
      connectionLabel.textContent = interactionMode === "push" ? "Streaming · plant" : "Streaming";
      updateInteractionUi();
    } else if (message.type === "plant_state") {
      enqueuePlantState(message);
    } else if (message.type === "plant_error") {
      showToast(message.message || "Plant command rejected");
    } else if (message.type === "plant_unavailable") {
      showToast(message.reason || "Physical plant unavailable");
      disconnectPlant({ preserveGhost: true });
      connectionLabel.textContent = "Plant reconnecting";
      setRobotControlsEnabled(false);
      if (socket?.readyState === WebSocket.OPEN) {
        setTimeout(connectPlant, 800);
      }
    }
  };
  plantSocket.onerror = () => {
    plantStatus.textContent = "connection fault";
  };
  plantSocket.onclose = () => {
    // Keep the last measured geometry as a disconnected ghost while clearing
    // all live plant state and command leases. The main /ws preview may still
    // be healthy, but a missing MuJoCo feedback stream must fail closed for
    // every interaction mode rather than leaving stale orange telemetry and
    // enabled controls on screen.
    plantSocket = null;
    disconnectPlant({ preserveGhost: true, closeSocket: false });
    connectionLabel.textContent = "Plant reconnecting";
    setRobotControlsEnabled(false);
    if (socket?.readyState === WebSocket.OPEN) {
      setTimeout(connectPlant, 800);
    }
  };
}

function setInteractionMode(mode) {
  if (mode !== "target" && mode !== "push") return;
  if (mode === "push" && !plantGateway?.available) {
    showToast("Physical MuJoCo plant is unavailable");
    return;
  }
  if (interactionMode === mode) return;
  drag = null;
  orbitDrag = null;
  pushDrag = null;
  pendingDragCommand = null;
  pendingPushCommand = null;
  activeForceArrow = null;
  selected = null;
  interactionMode = mode;
  previousSnapshot = null;
  latestSnapshot = null;
  if (mode === "push") {
    connectionLabel.textContent = plantConnected ? "Streaming · plant" : "Plant starting";
    connectPlant();
  } else {
    sendPlant({ type: "plant_release" });
    plantStatus.textContent = plantConnected ? "MuJoCo · measured ghost" : "starting MuJoCo";
    plantWrench.textContent = "released";
    connectionLabel.textContent = "Streaming";
    setRobotControlsEnabled(socket?.readyState === WebSocket.OPEN);
  }
  updateInteractionUi();
  scheduleRender();
}

function element(tag, className, text) {
  const node = document.createElement(tag);
  if (className) node.className = className;
  if (text !== undefined) node.textContent = text;
  return node;
}

async function loadArchitecture() {
  try {
    const response = await fetch("/architecture.json", { cache: "no-store" });
    if (!response.ok) throw new Error(`architecture manifest HTTP ${response.status}`);
    const manifest = await response.json();
    const serialized = JSON.stringify(manifest);
    if (serialized === architectureSerialized) return;
    architectureManifest = manifest;
    architectureSerialized = serialized;
    document.querySelector("#architecture-revision").textContent = architectureManifest.revision;
    document.querySelector("#architecture-summary").textContent = architectureManifest.summary;
    renderArchitecture(architectureManifest);
    if (location.hash.startsWith("#architecture-")) {
      setArchitectureOpen(true);
      requestAnimationFrame(() => {
        document.querySelector(location.hash)?.scrollIntoView({ block: "start" });
      });
    }
  } catch (error) {
    document.querySelector("#architecture-revision").textContent = "unavailable";
    architectureContent.replaceChildren(
      element("p", "architecture-error", `Architecture manifest unavailable: ${error.message}`),
    );
  }
}

function renderArchitecture(manifest) {
  const meta = element("div", "architecture-meta");
  meta.append(
    element("span", "architecture-stage", manifest.stage),
    element("span", null, `Revision ${manifest.revision} · ${manifest.updated}`),
  );

  const overview = element("div", "architecture-overview");
  const provisionalCount = manifest.decisions.filter((decision) => decision.status === "provisional").length;
  [
    ["Phase", "CPU first"],
    ["Open decisions", String(provisionalCount)],
    ["Known gaps", String(manifest.gaps.length)],
  ].forEach(([label, value]) => {
    const item = element("div");
    item.append(element("span", null, label), element("strong", null, value));
    overview.append(item);
  });

  const sectionSpecs = [
    ["architecture-flow", "Flow"],
    ["architecture-modules", "Modules"],
    ["architecture-decisions", "Decisions"],
    ["architecture-authority", "Authority"],
    ["architecture-evidence", "Evidence"],
    ["architecture-gaps", "Gaps"],
    ["architecture-feedback", "Review"],
  ];
  const sectionNav = element("nav", "architecture-nav");
  sectionNav.setAttribute("aria-label", "Architecture review sections");
  sectionSpecs.forEach(([id, label]) => {
    const link = element("a", null, label);
    link.href = `#${id}`;
    link.addEventListener("click", () => {
      const section = document.querySelector(`#${id}`);
      if (section instanceof HTMLDetailsElement) section.open = true;
    });
    sectionNav.append(link);
  });

  const flowSection = reviewSection("architecture-flow", "Runtime dataflow", "The pure core owns the boxes; adapters only supply input and consume output.", true);
  const flow = element("ol", "architecture-flow");
  manifest.flow.forEach((stage, index) => {
    const item = element("li");
    item.append(
      element("span", "flow-index", String(index + 1).padStart(2, "0")),
      element("strong", null, stage.name),
      element("p", null, stage.detail),
    );
    flow.append(item);
  });
  flowSection.body.append(flow);

  const moduleSection = reviewSection("architecture-modules", "Crate boundaries", "Public names follow the Bonesaw module convention.");
  const modules = element("div", "architecture-cards");
  manifest.modules.forEach((module) => {
    const card = element("article", "architecture-card");
    card.append(
      element("h4", null, module.name),
      element("p", null, module.role),
      element("small", null, module.runtime),
    );
    modules.append(card);
  });
  moduleSection.body.append(modules);

  const decisionSection = reviewSection("architecture-decisions", "Architectural decisions", "Frozen choices require an explicit revision; provisional choices invite review.");
  const decisions = element("div", "decision-list");
  manifest.decisions.forEach((decision) => {
    const item = element("article", "decision");
    const heading = element("div", "decision-heading");
    heading.append(
      element("span", `decision-status ${decision.status}`, decision.status),
      element("h4", null, decision.title),
    );
    item.append(heading, element("p", null, decision.detail));
    decisions.append(item);
  });
  decisionSection.body.append(decisions);

  const authority = manifest.authority_stack_example;
  const authoritySection = reviewSection(
    "architecture-authority",
    "Example authority stack",
    "A measured oracle-state example of what the WBC can satisfy now, what consumes headroom, and what remains persistently unavailable.",
    true,
  );
  if (authority) {
    authoritySection.body.append(
      element("p", "authority-disclaimer", authority.note),
      authorityGroup("Lexicographic layers", authority.layers),
      authorityGroup("Physical and compute resources", authority.resources),
    );
    if (authority.persistent_resource_fixture) {
      authoritySection.body.append(
        authorityGroup(
          "Persistent resource fixture · synthetic parameters",
          authority.persistent_resource_fixture,
        ),
      );
    }
  }

  const evidenceSection = reviewSection("architecture-evidence", "Measured evidence", "These are current release-build results, not intended future properties.");
  const evidence = element("div", "evidence-table");
  manifest.evidence.forEach((entry) => {
    const row = element("article");
    const metric = element(entry.href ? "a" : "strong", null, entry.metric);
    if (entry.href) {
      metric.href = entry.href;
      metric.target = "_blank";
      metric.rel = "noopener";
      metric.title = "Open the retained report";
    }
    row.append(
      metric,
      element("output", null, entry.result),
      element("small", null, entry.scope),
    );
    evidence.append(row);
  });
  evidenceSection.body.append(evidence);

  const gapsSection = reviewSection("architecture-gaps", "Known gaps", "Items that prevent claiming the complete architecture today.", true);
  gapsSection.body.append(reviewList(manifest.gaps, "gap-list"));

  const feedbackSection = reviewSection("architecture-feedback", "Feedback requested", "The decisions where architectural input is most valuable now.", true);
  feedbackSection.section.classList.add("feedback-section");
  feedbackSection.body.append(reviewList(manifest.feedback_questions, "feedback-list"));

  architectureContent.replaceChildren(
    meta,
    overview,
    sectionNav,
    flowSection.section,
    moduleSection.section,
    decisionSection.section,
    authoritySection.section,
    evidenceSection.section,
    gapsSection.section,
    feedbackSection.section,
  );
}

function authorityGroup(title, rows) {
  const group = element("section", "authority-group");
  group.append(element("h4", null, title));
  const list = element("div", "authority-list");
  rows.forEach((row) => {
    const item = element("article", `authority-row ${row.state}`);
    const heading = element("div", "authority-row-heading");
    heading.append(
      element("strong", null, row.name),
      element("output", null, row.value),
    );
    const track = element("div", "authority-track");
    const fill = element("span");
    fill.style.width = `${Math.round(100 * row.pressure)}%`;
    track.append(fill);
    item.append(
      heading,
      element("p", null, row.detail),
      track,
      element("small", null, `${row.signal} · ${Math.round(100 * row.pressure)}% pressure`),
    );
    list.append(item);
  });
  group.append(list);
  return group;
}

function reviewSection(id, title, description, mobileOpen = false) {
  const section = element("details", "architecture-section");
  section.id = id;
  section.open = !architectureMobileLayout.matches || mobileOpen;
  const summary = element("summary", "architecture-section-summary");
  const heading = element("div");
  heading.append(element("h3", null, title), element("p", "section-description", description));
  summary.append(heading, element("span", "section-toggle", "+"));
  const body = element("div", "architecture-section-body");
  section.append(summary, body);
  return { section, body };
}

function applyArchitectureBreakpoint() {
  architectureMobileState = architectureMobileLayout.matches;
  document.querySelectorAll(".architecture-section").forEach((section) => {
    if (section instanceof HTMLDetailsElement) {
      section.open = !architectureMobileLayout.matches
        || mobileOpenArchitectureSections.has(section.id);
    }
  });
}

architectureMobileLayout.addEventListener("change", applyArchitectureBreakpoint);
window.addEventListener("resize", () => {
  if (architectureMobileLayout.matches !== architectureMobileState) {
    applyArchitectureBreakpoint();
  }
});

function reviewList(items, className) {
  const list = element("ul", className);
  items.forEach((item) => list.append(element("li", null, item)));
  return list;
}

function setArchitectureOpen(open) {
  architectureReview.hidden = !open;
  architectureScrim.hidden = !open;
  architectureButton.setAttribute("aria-expanded", String(open));
  document.body.classList.toggle("architecture-open", open);
  if (open) {
    applyArchitectureBreakpoint();
    architectureClose.focus();
  } else {
    if (location.hash.startsWith("#architecture-")) {
      history.replaceState(null, "", `${location.pathname}${location.search}`);
    }
    architectureButton.focus({ preventScroll: true });
  }
}

function updatePlantTelemetry(message) {
  const metrics = message.metrics || {};
  const simulator = message.simulator || {};
  plantStatus.textContent = plantPaused
    ? "MuJoCo · paused"
    : `${metrics.wbc_status || "unknown"} · ${Number(metrics.controller_step_us || 0).toFixed(1)} µs`;
  const forwardInverse = simulator.solver_forward_inverse || [];
  const solverResidual = Math.max(
    ...forwardInverse.map((value) => Math.abs(Number(value))),
    0,
  );
  simulatorState.textContent = `${plantPaused ? "PAUSED · " : ""}t=${Number(simulator.time_s || 0).toFixed(3)} s · solver ${Number(simulator.solver_iterations || 0)} iter / ${Number(simulator.constraint_count || 0)} rows · fwd/inv ${solverResidual.toExponential(1)} · E=${(Number(simulator.kinetic_energy_j || 0) + Number(simulator.potential_energy_j || 0)).toFixed(2)} J · warnings ${Number(simulator.warning_count || 0)}`;
  const root = message.root_position || [0, 0, 0];
  const centerOfMass = message.center_of_mass_world || root;
  const twist = message.root_twist_world || [0, 0, 0, 0, 0, 0];
  plantRootState.textContent = `xyz ${root.map((value) => Number(value).toFixed(3)).join(" · ")} m · tilt ${(Number(metrics.root_tilt_rad || 0) * 180 / Math.PI).toFixed(2)}°`;
  plantComState.textContent = `xyz ${centerOfMass.map((value) => Number(value).toFixed(3)).join(" · ")} m · projection ${(Number(centerOfMass[2]) - Number(simulatorGroundPlane.point[2])).toFixed(3)} m`;
  plantMotionState.textContent = `|v| ${Math.hypot(...twist.slice(3)).toFixed(3)} m/s · |ω| ${Math.hypot(...twist.slice(0, 3)).toFixed(3)} rad/s · joint ${Number(metrics.maximum_abs_joint_speed_rad_s || 0).toFixed(2)} rad/s`;
  const actualActuatorForce = Math.max(
    ...(message.actuator_force || []).map((value) => Math.abs(Number(value))),
    0,
  );
  plantEffortState.textContent = `${Number(metrics.maximum_abs_actuator_effort_nm || 0).toFixed(3)} N·m command · ${actualActuatorForce.toFixed(3)} actuator force · q̈ ${Number(metrics.maximum_abs_generalized_acceleration || 0).toFixed(2)} max`;
  plantConstraintState.textContent = `${Number(metrics.maximum_abs_constraint_force || 0).toFixed(2)} generalized · ${Number(metrics.maximum_abs_constraint_scalar_force || 0).toFixed(2)} scalar · |pos| ${Number(metrics.maximum_abs_constraint_position || 0).toExponential(1)} · |vel| ${Number(metrics.maximum_abs_constraint_velocity || 0).toExponential(1)}`;
  const planePoint = simulatorGroundPlane.point;
  const planeNormal = simulatorGroundPlane.normal;
  plantGroundState.textContent = `point ${planePoint.map((value) => Number(value).toFixed(3)).join(" · ")} m · normal ${planeNormal.map((value) => Number(value).toFixed(2)).join(" · ")} · ${Number.isFinite(measuredPlantMinimumGroundClearanceM) ? `${(1000 * measuredPlantMinimumGroundClearanceM).toFixed(2)} mm collision clearance` : "collision geometry pending"}`;
  groundContactState.textContent = `${Number(metrics.ground_contact_count || 0)} ground / ${Number(metrics.contact_count || 0)} total · ${Number(metrics.total_ground_normal_force_n || 0).toFixed(1)} N normal · ${(1000 * Number(metrics.maximum_penetration_m || 0)).toFixed(2)} mm penetration`;
  const contactMask = (mask) => Array.isArray(mask) && mask.length === 2
    ? mask.map((value) => Number(value) ? "1" : "0").join("")
    : "--";
  const wbcObservation = message.wbc_observation || {};
  const wbcMask = contactMask(
    wbcObservation.contact_active || message.wbc_observed_contact_active,
  );
  const wbcFrame = metrics.wbc_observed_contact_available !== false
    && Number.isFinite(Number(wbcObservation.physics_frame_index))
    ? `f${Number(wbcObservation.physics_frame_index)}`
    : "f--";
  const physicsMask = contactMask(message.physics_contact_active);
  const windowMasks = Array.isArray(simulator.contact_window_masks)
    && simulator.contact_window_valid === true
    && simulator.contact_window_masks.length >= 5
    ? simulator.contact_window_masks.slice(0, 5).map(contactMask).join("/")
    : "--/--/--/--/--";
  const aggregateLoss = contactMask(simulator.contact_window_loss_mask);
  const aggregateGain = contactMask(simulator.contact_window_gain_mask);
  contactCadenceState.textContent = `WBC ${wbcMask}/${wbcFrame} · PHY ${physicsMask} · S ${windowMasks} · L ${aggregateLoss} G ${aggregateGain}`;
  const loadPair = (value) => Array.isArray(value) && value.length === 2
    ? value.map((entry) => Math.max(0, Number(entry)))
    : [0, 0];
  const measuredLoads = loadPair(message.wbc_observed_wheel_normal_force_n);
  const predictedLoads = loadPair(message.wbc_predicted_normal_force_n);
  const measuredTotal = measuredLoads[0] + measuredLoads[1];
  const reserve = measuredTotal > 1e-9
    ? Math.min(measuredLoads[0], measuredLoads[1]) / measuredTotal
    : 0;
  const guardState = metrics.wbc_support_load_guard_enabled
    ? ` · guard ${Math.round(100 * Number(metrics.wbc_support_load_guard_authority || 0))}%`
    : "";
  contactLoadState.textContent = `measured L/R ${measuredLoads.map((entry) => entry.toFixed(1)).join("/")} N · WBC ${predictedLoads.map((entry) => entry.toFixed(1)).join("/")} N · reserve ${(100 * reserve).toFixed(1)}%${guardState}`;
  plantWrench.textContent = message.external_load?.active
    ? `${Math.hypot(...message.external_load.force_world).toFixed(2)} N · ${Number(message.external_load.maximum_moment_nm || 0).toFixed(2)} N·m · ${message.external_load.body} · ${(message.external_load.provenance?.source || "unavailable").replaceAll("_", " ")}`
    : message.command_expired ? "expired safely" : "released";
  const capture = Math.max(0, Math.min(1, Number(metrics.capture_pressure || 0)));
  setLiveAuthorityRow(
    "authority-capture",
    capture.toFixed(3),
    `full DCM pressure · error ${(1000 * Number(metrics.capture_error_m || 0)).toFixed(2)} mm · ${metrics.wbc_admitted ? "admitted" : "held/rejected"}`,
    capture,
    capture >= 1 ? "critical" : capture >= 0.65 ? "warning" : "ok",
  );
  const fallSafeMode = Math.max(0, Math.min(3, Number(metrics.fall_safe_mode || 0)));
  const fallSafePrimary = Math.max(0, Math.min(1, Number(metrics.fall_safe_primary_authority ?? 1)));
  const fallSafeFresh = Math.max(0, Math.min(1, Number(metrics.fall_safe_fresh_command_authority ?? 1)));
  const fallSafeLabels = ["PRIMARY", "DEGRADED", "CONTINGENCY", "FALLEN"];
  setLiveAuthorityRow(
    "authority-fall-safe",
    `${Math.round(100 * fallSafeFresh)}%`,
    `live stale-command lease · risk ${fallSafeLabels[fallSafeMode]} / suggested primary ${fallSafePrimary.toFixed(3)} (diagnostic only) · flags 0x${Number(metrics.fall_safe_reason_flags || 0).toString(16)}`,
    1 - fallSafeFresh,
    fallSafeFresh < 0.5 ? "critical" : fallSafeFresh < 1 ? "warning" : "ok",
  );
  const stationAuthority = Math.max(0, Math.min(1, Number(metrics.station_authority || 0)));
  const stationPressure = 1 - stationAuthority;
  setLiveAuthorityRow(
    "authority-station",
    stationAuthority.toFixed(3),
    `odom preference authority · error ${(1000 * Number(metrics.station_error_m || 0)).toFixed(2)} mm`,
    stationPressure,
    stationPressure >= 0.99 ? "critical" : stationPressure >= 0.35 ? "warning" : "ok",
  );
  const observedMask = (metrics.wbc_observed_contact_active || [0, 0]).join("");
  const debouncedMask = (metrics.wbc_debounced_contact_active || [0, 0]).join("");
  const hardMask = (metrics.wbc_hard_contact_active || [0, 0]).join("");
  const executableMask = (metrics.wbc_hard_contact_executable || [0, 0]).join("");
  const supportCount = Number(metrics.wbc_support_active_count || 0);
  const supportPressure = supportCount >= 2 ? 0 : supportCount === 1 ? 0.72 : 1;
  const contingencyEnabled = Boolean(metrics.wbc_support_contingency_enabled);
  const contingencyState = contingencyEnabled
    ? (metrics.wbc_support_contingency_selected
      ? "selected"
      : metrics.wbc_support_contingency_admitted
        ? "admitted"
        : metrics.wbc_support_contingency_requested
          ? "requested"
          : "idle")
    : "off";
  const contingencyMode = ["DS", "SS", "FL"][
    Number(metrics.wbc_support_contingency_mode)
  ] || "?";
  setLiveAuthorityRow(
    "authority-support",
    `${supportCount}/2`,
    `measured ${observedMask} · debounced ${debouncedMask} · hard ${hardMask} · exec ${executableMask} · ${Number(metrics.total_ground_normal_force_n || 0).toFixed(1)} N sampled ground load${contingencyEnabled ? ` · contingency ${contingencyState}/${contingencyMode} · ${Number(metrics.wbc_support_contingency_step_us || 0).toFixed(1)} µs` : ""}`,
    supportPressure,
    supportCount === 0 ? "critical" : supportCount === 1 ? "warning" : "ok",
  );
  const touchdownEnabled = Boolean(metrics.wbc_single_support_reacquisition_enabled);
  const touchdownDiagnostics = Array.isArray(metrics.wbc_single_support_reacquisition_diagnostics)
    ? metrics.wbc_single_support_reacquisition_diagnostics
    : [];
  const touchdownActive = Boolean(metrics.wbc_single_support_reacquisition_active);
  const touchdownAuthority = clampUnit(Number(metrics.wbc_single_support_reacquisition_authority || 0));
  const touchdownError = Number(touchdownDiagnostics[7]);
  const touchdownAcceleration = Number(touchdownDiagnostics[9]);
  const touchdownTransition = Number(touchdownDiagnostics[13]);
  setLiveAuthorityRow(
    "authority-touchdown",
    touchdownEnabled ? (touchdownActive ? `${Math.round(100 * touchdownAuthority)}%` : "ARMED") : "OFF",
    touchdownEnabled
      ? `${touchdownActive ? "measured single support" : "waiting for exact single support"} · Δz ${Number.isFinite(touchdownError) ? `${(1000 * touchdownError).toFixed(1)} mm` : "N/A"} · aᵥ ${Number.isFinite(touchdownAcceleration) ? `${touchdownAcceleration.toFixed(2)} m/s²` : "N/A"} · transition ${Number.isFinite(touchdownTransition) ? touchdownTransition : "N/A"}`
      : "default-off evaluation request · no command authority",
    touchdownEnabled ? (touchdownActive ? touchdownAuthority : 0) : 0,
    !touchdownEnabled ? "unavailable" : touchdownActive ? "warning" : "ok",
  );
  const residualParts = [
    ["dyn", Number(metrics.wbc_dynamics_residual)],
    ["contact", Number(metrics.wbc_contact_residual)],
    ["ineq", Number(metrics.wbc_maximum_constraint_violation)],
  ].filter((entry) => Number.isFinite(entry[1]));
  const hardResidual = Math.max(...residualParts.map((entry) => Math.abs(entry[1])), 0);
  const hardThresholds = authorityThresholds.hard_residual;
  const hardPressure = upperPressure(hardResidual, hardThresholds);
  setLiveAuthorityRow(
    "authority-hard",
    hardResidual.toExponential(1),
    residualParts.map((entry) => `${entry[0]} ${entry[1].toExponential(1)}`).join(" · "),
    hardPressure,
    pressureState(
      hardPressure,
      hardThresholds && hardResidual > hardThresholds.warning,
      hardThresholds && hardResidual > hardThresholds.critical,
    ),
  );
  const torqueUtilization = Number(metrics.torque_utilization);
  const torquePressure = Number.isFinite(torqueUtilization)
    ? clampUnit(torqueUtilization)
    : 0;
  setLiveAuthorityRow(
    "authority-actuator",
    Number.isFinite(torqueUtilization) ? `${Math.round(100 * torqueUtilization)}%` : "N/A",
    `${Number(metrics.maximum_abs_actuator_effort_nm || 0).toFixed(3)} N·m max measured command · thermal calibration remains unavailable`,
    torquePressure,
    !Number.isFinite(torqueUtilization)
      ? "unavailable"
      : torquePressure >= 1 ? "critical" : torquePressure >= 0.8 ? "warning" : "ok",
  );
  const solveUs = Number(metrics.controller_step_us);
  const solvePressure = upperPressure(solveUs, authorityThresholds.solver_wall_time_us);
  const solverRejected = !metrics.wbc_admitted || metrics.wbc_raw_status === "MaxIterations";
  setLiveAuthorityRow(
    "authority-solver",
    `${Number.isFinite(solveUs) ? solveUs.toFixed(1) : "N/A"} µs`,
    `${metrics.wbc_status || "unknown"} · raw ${metrics.wbc_raw_status || "unknown"} · ${Number(metrics.wbc_allocation_calls || 0)} calls / ${Number(metrics.wbc_allocated_bytes || 0)} bytes`,
    solverRejected ? 1 : solvePressure,
    solverRejected
      ? "critical"
      : pressureState(
        solvePressure,
        authorityThresholds.solver_wall_time_us
          && solveUs > authorityThresholds.solver_wall_time_us.warning,
        authorityThresholds.solver_wall_time_us
          && solveUs > authorityThresholds.solver_wall_time_us.critical,
      ),
  );
}

function connect() {
  const protocol = location.protocol === "https:" ? "wss:" : "ws:";
  socket = new WebSocket(`${protocol}//${location.host}/ws`);
  socket.addEventListener("open", () => {
    connection.classList.add("online");
    connectionLabel.textContent = "Synchronizing";
    lastSocketMessageAt = performance.now();
  });
  socket.addEventListener("close", () => {
    disconnectPlant();
    connection.classList.remove("online");
    connectionLabel.textContent = "Reconnecting";
    setRobotControlsEnabled(false);
    setTimeout(connect, 800);
  });
  socket.addEventListener("message", (event) => {
    lastSocketMessageAt = performance.now();
    const message = JSON.parse(event.data);
    if (message.type === "hello") {
      bones = message.bones;
      const authoredVisuals = (message.visual_geometry || []).map((visual) => ({
        ...visual.shape,
        rgba: visual.rgba,
      }));
      const geometrySource = authoredVisuals.length ? authoredVisuals : (message.geometry || []);
      geometry = geometrySource.map((sourceShape) => {
        const shape = { ...sourceShape };
        const surface = buildGeometrySurface(shape);
        shape.surface = surface ? prepareGeometrySurface(shape, surface) : null;
        return shape;
      });
      collisionGeometry = (message.geometry || []).map((sourceShape) => {
        const shape = { ...sourceShape };
        const surface = buildGeometrySurface(shape);
        shape.surface = surface ? prepareGeometrySurface(shape, surface) : null;
        return shape;
      });
      geometry.filter((shape) => shape.kind === "mesh").forEach(loadMeshSurface);
      supportPatches = message.support_patches || [];
      worldSdfPlanes = message.world_sdf_planes || [];
      frameNames = message.frame_names;
      bodyNames = message.body_names || [];
      coordinateNames = message.coordinate_names || [];
      actuatorNames = message.actuator_names || coordinateNames;
      actuatorResourceModels = message.actuator_resource_models || [];
      applyAuthorityContract(message.authority_contract);
      interactionHandles = new Map(
        (message.interaction_handles || []).map((handle) => [handle.frame, handle]),
      );
      plantGateway = message.plant_gateway || { available: false };
      plantStatus.textContent = plantGateway.available ? "available · idle" : "unavailable";
      plantWrenchLimit.textContent = plantGateway.available
        ? `${Number(plantGateway.maximum_force_n).toFixed(0)} N · ${Number(plantGateway.maximum_application_offset_m).toFixed(2)} m point · ${plantGateway.command_ttl_ms} ms`
        : "unavailable";
      connectionLabel.textContent = "Streaming";
      setRobotControlsEnabled(true);
      updateInteractionUi();
      connectPlant();
      baseExecution = message.base_execution || message.squat_execution || "raw_dynamic";
      authorityThresholds = message.authority_thresholds || authorityThresholds;
      document.querySelector("#model-name").textContent = message.model;
      document.querySelector("#dof-count").textContent = message.dof;
      document.querySelector("#body-count").textContent = message.bodies;
      document.querySelector("#root-frames").textContent = message.rooted_frames.join(" · ");
      const supportPointCount = supportPatches.reduce(
        (total, patch) => total + patch.points.length,
        0,
      );
      document.querySelector("#support-model").textContent = supportPatches.length
        ? `${supportPatches.length} finite patches · ${supportPointCount} points`
        : "rolling contacts · finite area N/A";
      document.querySelector("#base-execution").textContent =
        baseExecution === "guided_preview" ? "guided WBC preview" : "raw dynamics";
    } else if (message.type === "state") {
      // Network state may arrive faster than a mobile browser can paint the
      // mesh and telemetry DOM. Retain only the newest visual state and apply
      // it once at the display boundary; the Rust controller stream remains
      // unthrottled and its tick number exposes any visual coalescing.
      if (interactionMode !== "push" || !plantConnected) {
        enqueueState(message);
        scheduleRender();
      }
    } else if (message.type === "observation_withheld") {
      updateObservationTransport({
        robot_observation_transport_mode: message.transport_mode,
      });
      const newest = Number.isFinite(message.newest_sample_time_ns)
        ? `newest ${message.newest_sample_time_ns} ns`
        : "history empty";
      observationTransportDetail.textContent = `WBC withheld · query ${message.query_time_ns} ns · ${newest}`;
      setLiveAuthorityRow(
        "authority-robot-history",
        "WITHHELD",
        `${message.transport_mode.toUpperCase()} · ${message.reason}`,
        1,
        "critical",
      );
      showToast(message.reason);
    } else if (message.type === "error") {
      showToast(message.message);
    }
  });
}

function setRobotControlsEnabled(enabled) {
  robotControlsEnabled = enabled;
  viewport.classList.toggle("disconnected", !enabled);
  disconnectOverlay.hidden = enabled;
  [
    document.querySelector("#reset-button"),
    pauseButton,
    resumeButton,
    targetTool,
    document.querySelector("#geometry-toggle"),
    document.querySelector("#rig-toggle"),
  ].forEach((control) => { control.disabled = !enabled; });
  if (!enabled) {
    drag = null;
    pushDrag = null;
    orbitDrag = null;
    pendingDragCommand = null;
    pendingPushCommand = null;
    activeForceArrow = null;
    plantContacts = [];
    canvas.classList.remove("dragging", "orbiting", "joint-hover");
  } else {
    scheduleRender();
  }
  updateInteractionUi();
}

function resize() {
  // A 3x phone display otherwise asks the 2D mesh renderer to shade nine
  // physical pixels per CSS pixel. Two is visually crisp and bounds fill work.
  const ratio = Math.min(devicePixelRatio || 1, MAX_VIEWPORT_PIXEL_RATIO);
  const bounds = canvas.getBoundingClientRect();
  const pixelWidth = Math.max(1, Math.round(bounds.width * ratio));
  const pixelHeight = Math.max(1, Math.round(bounds.height * ratio));
  if (canvas.width !== pixelWidth || canvas.height !== pixelHeight) {
    canvas.width = pixelWidth;
    canvas.height = pixelHeight;
  }
  context.setTransform(ratio, 0, 0, ratio, 0, 0);
  transform.centerX = bounds.width / 2;
  transform.centerY = bounds.height * 0.52;
  transform.focalPixels = 0.5 * bounds.height / Math.tan(camera.verticalFov * 0.5);
  scheduleRender();
  drawAuthorityHistory();
}

function dot(a, b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

function updateCameraBasis() {
  const cosPitch = Math.cos(camera.pitch);
  const sinPitch = Math.sin(camera.pitch);
  const cosYaw = Math.cos(camera.yaw);
  const sinYaw = Math.sin(camera.yaw);
  camera.forward = [-cosPitch * cosYaw, -cosPitch * sinYaw, -sinPitch];
  camera.right = [sinYaw, -cosYaw, 0];
  camera.up = [
    -sinPitch * cosYaw,
    -sinPitch * sinYaw,
    cosPitch,
  ];
  camera.position = camera.target.map(
    (value, index) => value - camera.forward[index] * camera.distance,
  );
  viewModeLabel.textContent = "PERSPECTIVE ORBIT";
  viewAxesLabel.textContent = `42° FOV · YAW ${Math.round(camera.yaw * 180 / Math.PI)}° · PITCH ${Math.round(camera.pitch * 180 / Math.PI)}°`;
}

function project(position) {
  const relative = subtractVector(position, camera.position);
  const depth = dot(relative, camera.forward);
  const safeDepth = Math.max(depth, 0.02);
  return {
    x: transform.centerX
      + transform.focalPixels * dot(relative, camera.right) / safeDepth,
    y: transform.centerY
      - transform.focalPixels * dot(relative, camera.up) / safeDepth,
    depth,
    visible: depth > 0.02,
  };
}

function unprojectViewPlane(point, depth) {
  const screenRight = (point.x - transform.centerX) * depth / transform.focalPixels;
  const screenUp = -(point.y - transform.centerY) * depth / transform.focalPixels;
  return camera.position.map((value, index) => value
    + camera.forward[index] * depth
    + camera.right[index] * screenRight
    + camera.up[index] * screenUp);
}

function quaternionRotate(rotation, vector) {
  const [x, y, z, w] = rotation;
  const tx = 2 * (y * vector[2] - z * vector[1]);
  const ty = 2 * (z * vector[0] - x * vector[2]);
  const tz = 2 * (x * vector[1] - y * vector[0]);
  return [
    vector[0] + w * tx + (y * tz - z * ty),
    vector[1] + w * ty + (z * tx - x * tz),
    vector[2] + w * tz + (x * ty - y * tx),
  ];
}

function addVector(left, right) {
  return left.map((value, index) => value + right[index]);
}

function subtractVector(left, right) {
  return left.map((value, index) => value - right[index]);
}

function crossVector(left, right) {
  return [
    left[1] * right[2] - left[2] * right[1],
    left[2] * right[0] - left[0] * right[2],
    left[0] * right[1] - left[1] * right[0],
  ];
}

function boxSurface(halfExtents) {
  const [x, y, z] = halfExtents;
  return {
    vertices: [
      [-x, -y, -z], [x, -y, -z], [x, y, -z], [-x, y, -z],
      [-x, -y, z], [x, -y, z], [x, y, z], [-x, y, z],
    ],
    faces: [
      [0, 3, 2, 1], [4, 5, 6, 7], [0, 1, 5, 4],
      [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7],
    ],
  };
}

function cylinderSurface(radius, halfLength, segments = 12) {
  const vertices = [];
  for (const z of [-halfLength, halfLength]) {
    for (let segment = 0; segment < segments; segment += 1) {
      const angle = 2 * Math.PI * segment / segments;
      vertices.push([radius * Math.cos(angle), radius * Math.sin(angle), z]);
    }
  }
  const bottomCenter = vertices.push([0, 0, -halfLength]) - 1;
  const topCenter = vertices.push([0, 0, halfLength]) - 1;
  const faces = [];
  for (let segment = 0; segment < segments; segment += 1) {
    const next = (segment + 1) % segments;
    faces.push([segment, next, segments + next, segments + segment]);
    faces.push([bottomCenter, next, segment]);
    faces.push([topCenter, segments + segment, segments + next]);
  }
  return { vertices, faces };
}

function roundedSurface(radius, halfLength = 0, segments = 12, latitudeSegments = 8) {
  const vertices = [];
  for (let latitude = 0; latitude <= latitudeSegments; latitude += 1) {
    const angle = Math.PI * latitude / latitudeSegments;
    const radial = radius * Math.sin(angle);
    const axial = radius * Math.cos(angle) + (Math.cos(angle) >= 0 ? halfLength : -halfLength);
    for (let segment = 0; segment < segments; segment += 1) {
      const azimuth = 2 * Math.PI * segment / segments;
      vertices.push([radial * Math.cos(azimuth), radial * Math.sin(azimuth), axial]);
    }
  }
  const faces = [];
  for (let latitude = 0; latitude < latitudeSegments; latitude += 1) {
    for (let segment = 0; segment < segments; segment += 1) {
      const next = (segment + 1) % segments;
      const lower = latitude * segments;
      const upper = (latitude + 1) * segments;
      faces.push([lower + segment, lower + next, upper + next, upper + segment]);
    }
  }
  return { vertices, faces };
}

function buildGeometrySurface(shape) {
  if (shape.kind === "box") return boxSurface(shape.half_extents);
  if (shape.kind === "cylinder") return cylinderSurface(shape.radius, shape.half_length);
  if (shape.kind === "sphere") return roundedSurface(shape.radius);
  if (shape.kind === "capsule") return roundedSurface(shape.radius, shape.half_length);
  return null;
}

function meshAssetUrl(filename) {
  const prefix = "package://upkie_description/meshes/";
  if (!filename.startsWith(prefix)) return null;
  const path = filename.slice(prefix.length)
    .split("/")
    .map(encodeURIComponent)
    .join("/");
  return `/model-assets/upkie/meshes/${path}`;
}

function decodeBinaryStl(buffer) {
  if (buffer.byteLength < 84) throw new Error("truncated STL header");
  const view = new DataView(buffer);
  const triangleCount = view.getUint32(80, true);
  if (84 + triangleCount * 50 > buffer.byteLength) {
    throw new Error("invalid binary STL triangle count");
  }
  const minimum = [Infinity, Infinity, Infinity];
  const maximum = [-Infinity, -Infinity, -Infinity];
  for (let triangle = 0; triangle < triangleCount; triangle += 1) {
    const offset = 84 + triangle * 50 + 12;
    for (let corner = 0; corner < 3; corner += 1) {
      const vertexOffset = offset + corner * 12;
      for (let axis = 0; axis < 3; axis += 1) {
        const value = view.getFloat32(vertexOffset + axis * 4, true);
        minimum[axis] = Math.min(minimum[axis], value);
        maximum[axis] = Math.max(maximum[axis], value);
      }
    }
  }
  // A fixed spatial grid keeps the simplified surface coherent. Sampling
  // isolated triangles made high-detail motors look shredded; clustering
  // retains closed, consistently wound faces while bounding per-frame work.
  const resolution = triangleCount > 100000 ? 5 : triangleCount > 10000 ? 6 : 10;
  const stride = resolution + 1;
  const clusterByCell = new Map();
  const clusters = [];
  const faces = [];
  const faceKeys = new Set();
  for (let triangle = 0; triangle < triangleCount; triangle += 1) {
    const offset = 84 + triangle * 50 + 12;
    const indices = [];
    for (let corner = 0; corner < 3; corner += 1) {
      const vertexOffset = offset + corner * 12;
      const vertex = [
        view.getFloat32(vertexOffset, true),
        view.getFloat32(vertexOffset + 4, true),
        view.getFloat32(vertexOffset + 8, true),
      ];
      const cell = vertex.map((value, axis) => {
        const extent = maximum[axis] - minimum[axis];
        return extent > 0
          ? Math.round(resolution * (value - minimum[axis]) / extent)
          : 0;
      });
      const key = (cell[0] * stride + cell[1]) * stride + cell[2];
      let index = clusterByCell.get(key);
      if (index === undefined) {
        index = clusters.length;
        clusterByCell.set(key, index);
        clusters.push({ sum: [0, 0, 0], count: 0 });
      }
      const cluster = clusters[index];
      cluster.sum = cluster.sum.map((sum, axis) => sum + vertex[axis]);
      cluster.count += 1;
      indices.push(index);
    }
    if (new Set(indices).size !== 3) continue;
    const faceKey = [...indices].sort((left, right) => left - right).join(":");
    if (!faceKeys.has(faceKey)) {
      faceKeys.add(faceKey);
      faces.push(indices);
    }
  }
  const vertices = clusters.map((cluster) =>
    cluster.sum.map((sum) => sum / cluster.count));
  return { vertices, faces, sourceTriangleCount: triangleCount };
}

function loadMeshSurface(shape) {
  const url = meshAssetUrl(shape.filename);
  if (!url) return;
  if (!meshSurfaceCache.has(url)) {
    meshSurfaceCache.set(url, fetch(url, { cache: "force-cache" })
      .then((response) => {
        if (!response.ok) throw new Error(`mesh HTTP ${response.status}`);
        return response.arrayBuffer();
      })
      .then((buffer) => decodeBinaryStl(buffer)));
  }
  meshSurfaceCache.get(url)
    .then((surface) => {
      shape.surface = prepareGeometrySurface(shape, surface);
      scheduleRender();
    })
    .catch((error) => showToast(`Visual mesh unavailable · ${error.message}`));
}

function prepareGeometrySurface(shape, surface) {
  const bodyVertices = surface.vertices.map((vertex) => {
    const scaledVertex = shape.kind === "mesh"
      ? vertex.map((value, index) => value * shape.scale[index])
      : vertex;
    return addVector(
      shape.translation,
      quaternionRotate(shape.rotation_xyzw, scaledVertex),
    );
  });
  const bodyFaceNormals = surface.faces.map((indices) => {
    if (indices.length < 3) return [0, 0, 1];
    const edgeA = subtractVector(bodyVertices[indices[1]], bodyVertices[indices[0]]);
    const edgeB = subtractVector(bodyVertices[indices[2]], bodyVertices[indices[0]]);
    const normal = crossVector(edgeA, edgeB);
    const length = Math.max(Math.sqrt(dot(normal, normal)), 1e-12);
    return normal.map((value) => value / length);
  });
  return { ...surface, bodyVertices, bodyFaceNormals };
}

function geometryVertexWorld(body, vertex) {
  return addVector(body.translation, quaternionRotate(body.rotation_xyzw, vertex));
}

function bodyGeometryHue(name) {
  if (name.includes("wheel") || name.includes("tire")) return 210;
  if (name.includes("femur") || name.includes("shin")) return 34;
  if (name.includes("torso") || name.includes("handle")) return 14;
  if (name.includes("battery")) return 225;
  if (name.includes("contact")) return 155;
  return 202;
}

function litGeometryFill(shape, hue, lighting) {
  if (!shape.rgba) {
    return `hsl(${hue} 35% ${Math.round(20 + 24 * lighting)}%)`;
  }
  const gain = 0.42 + 0.58 * lighting;
  const rgb = shape.rgba.slice(0, 3)
    .map((channel) => Math.round(255 * Math.min(1, Math.max(0, channel * gain))));
  const alpha = Math.min(1, Math.max(0.12, shape.rgba[3] * 0.92));
  return `rgba(${rgb[0]}, ${rgb[1]}, ${rgb[2]}, ${alpha})`;
}

function drawGeometryLayer() {
  if (!showGeometry || !frames.length || !geometry.length) return;
  const faces = [];
  const light = [0.35, -0.45, 0.82];
  let minimumGroundClearanceM = Infinity;
  for (const shape of geometry) {
    if (!shape.surface) continue;
    const body = frames[shape.body];
    if (!body) continue;
    const worldVertices = shape.surface.bodyVertices.map(
      (vertex) => geometryVertexWorld(body, vertex),
    );
    for (const vertex of worldVertices) {
      minimumGroundClearanceM = Math.min(minimumGroundClearanceM, vertex[2]);
    }
    const hue = bodyGeometryHue(body.name || "");
    for (let faceIndex = 0; faceIndex < shape.surface.faces.length; faceIndex += 1) {
      const indices = shape.surface.faces[faceIndex];
      const world = indices.map((index) => worldVertices[index]);
      if (world.length < 3) continue;
      const normal = quaternionRotate(body.rotation_xyzw, shape.surface.bodyFaceNormals[faceIndex]);
      const lighting = 0.34 + 0.66 * Math.abs(dot(normal, light));
      const projected = world.map(project);
      faces.push({
        projected,
        depth: projected.reduce((sum, point) => sum + point.depth, 0) / projected.length,
        fill: litGeometryFill(shape, hue, lighting),
        penetrating: world.some((vertex) => vertex[2] < -0.001),
        active: selected?.name === body.name,
        mesh: shape.kind === "mesh",
      });
    }
  }
  faces.sort((left, right) => right.depth - left.depth);
  context.save();
  context.lineJoin = "round";
  for (const face of faces) {
    context.beginPath();
    context.moveTo(face.projected[0].x, face.projected[0].y);
    for (let index = 1; index < face.projected.length; index += 1) {
      context.lineTo(face.projected[index].x, face.projected[index].y);
    }
    context.closePath();
    context.fillStyle = face.penetrating ? "rgba(239,117,106,0.82)" : face.fill;
    context.globalAlpha = face.active ? 0.96 : 0.86;
    context.fill();
    context.globalAlpha = 1;
    if (!face.mesh || face.active) {
      context.strokeStyle = face.active ? "#bdf5dc" : "#4f5d64";
      context.lineWidth = face.active ? 1.4 : 0.65;
      context.stroke();
    }
  }
  context.restore();
  renderedMinimumGroundClearanceM = Number.isFinite(minimumGroundClearanceM)
    ? minimumGroundClearanceM
    : Number.NaN;
  const now = performance.now();
  if (now - lastPreviewGroundUpdateMs >= 100) {
    const clearanceMm = 1000 * renderedMinimumGroundClearanceM;
    const collisionClearanceMm = 1000 * Number(
      latestMetrics?.minimum_collision_ground_clearance_m,
    );
    previewGroundState.textContent = Number.isFinite(clearanceMm)
      ? `${clearanceMm.toFixed(2)} mm visual · ${Number.isFinite(collisionClearanceMm) ? `${collisionClearanceMm.toFixed(2)} mm collision` : "collision N/A"} · ${clearanceMm < -1 ? "PENETRATING" : "z=0 plane"}`
      : "geometry unavailable";
    previewGroundState.classList.toggle("critical-value", clearanceMm < -1);
    lastPreviewGroundUpdateMs = now;
  }
}

function drawPlantContactLayer() {
  if (!plantContacts.length) return;
  context.save();
  context.lineCap = "round";
  for (const contact of plantContacts) {
    const position = contact.position_world;
    const normal = contact.normal_world;
    if (!Array.isArray(position) || !Array.isArray(normal)) continue;
    const point = project(position);
    const scale = Math.min(0.08, 0.012 + Number(contact.normal_force_n || 0) * 0.00035);
    const end = project(position.map((value, axis) => value + normal[axis] * scale));
    const penetration = Math.max(0, -Number(contact.distance_m || 0));
    const color = penetration > 0.003 ? "#ef756a" : penetration > 0.001 ? "#e0b15a" : "#77d4ae";
    context.strokeStyle = color;
    context.fillStyle = color;
    context.lineWidth = 1.5;
    context.beginPath();
    context.moveTo(point.x, point.y);
    context.lineTo(end.x, end.y);
    context.stroke();
    context.beginPath();
    context.arc(point.x, point.y, contact.ground ? 3.5 : 2.5, 0, Math.PI * 2);
    context.fill();
  }
  context.restore();
}

function normalizedPlaneNormal() {
  const normal = simulatorGroundPlane.normal;
  const length = Math.max(Math.hypot(...normal), 1e-12);
  return normal.map((value) => value / length);
}

function groundSignedDistance(point) {
  const normal = normalizedPlaneNormal();
  return dot(subtractVector(point, simulatorGroundPlane.point), normal);
}

function drawMeasuredPlantCollisionLayer() {
  if (!plantConnected || !measuredPlantFrames.length || !collisionGeometry.length) return;
  let minimumClearance = Infinity;
  const safeFaces = [];
  const penetratingFaces = [];
  for (const shape of collisionGeometry) {
    if (!shape.surface) continue;
    const body = measuredPlantFrames[shape.body];
    if (!body) continue;
    const vertices = shape.surface.bodyVertices.map(
      (vertex) => geometryVertexWorld(body, vertex),
    );
    for (const vertex of vertices) {
      minimumClearance = Math.min(minimumClearance, groundSignedDistance(vertex));
    }
    const destination = vertices.some((vertex) => groundSignedDistance(vertex) < -0.001)
      ? penetratingFaces
      : safeFaces;
    for (const indices of shape.surface.faces) {
      if (indices.length < 3) continue;
      destination.push(indices.map((index) => project(vertices[index])));
    }
  }
  measuredPlantMinimumGroundClearanceM = Number.isFinite(minimumClearance)
    ? minimumClearance
    : Number.NaN;
  const strokeFaces = (faces, color, width) => {
    if (!faces.length) return;
    context.beginPath();
    for (const face of faces) {
      context.moveTo(face[0].x, face[0].y);
      for (let index = 1; index < face.length; index += 1) {
        context.lineTo(face[index].x, face[index].y);
      }
      context.closePath();
    }
    context.strokeStyle = color;
    context.lineWidth = width;
    context.stroke();
  };
  context.save();
  context.lineJoin = "round";
  strokeFaces(safeFaces, interactionMode === "target"
    ? "rgba(255,157,69,0.24)"
    : "rgba(255,181,111,0.15)", 0.7);
  strokeFaces(penetratingFaces, "rgba(239,117,106,0.95)", 1.8);

  const centerOfMass = plantState?.center_of_mass_world;
  if (Array.isArray(centerOfMass) && centerOfMass.length === 3) {
    const normal = normalizedPlaneNormal();
    const distance = groundSignedDistance(centerOfMass);
    const projection = centerOfMass.map(
      (value, axis) => value - normal[axis] * distance,
    );
    const comPoint = project(centerOfMass);
    const groundPoint = project(projection);
    context.setLineDash([3, 4]);
    context.strokeStyle = "rgba(255,181,111,0.72)";
    context.beginPath();
    context.moveTo(comPoint.x, comPoint.y);
    context.lineTo(groundPoint.x, groundPoint.y);
    context.stroke();
    context.setLineDash([]);
    context.fillStyle = "rgba(255,181,111,0.95)";
    context.beginPath();
    context.arc(groundPoint.x, groundPoint.y, 4, 0, 2 * Math.PI);
    context.fill();
  }
  context.restore();
}

function drawMeasuredPlantLayer() {
  if (interactionMode === "push" || !measuredPlantFrames.length) return;
  context.save();
  context.lineCap = "round";
  context.lineJoin = "round";
  context.setLineDash([3, 5]);
  context.strokeStyle = "rgba(255,157,69,0.54)";
  context.lineWidth = 1.5;
  for (const bone of bones) {
    const parent = measuredPlantFrames[bone.parent];
    const child = measuredPlantFrames[bone.child];
    if (!parent || !child) continue;
    const a = project(parent.translation);
    const b = project(child.translation);
    context.beginPath();
    context.moveTo(a.x, a.y);
    context.lineTo(b.x, b.y);
    context.stroke();
  }
  context.setLineDash([]);
  const base = measuredPlantFrames.find((frame) => frame.name === "base")
    || measuredPlantFrames[0];
  if (base) {
    const point = project(base.translation);
    context.fillStyle = "rgba(255,181,111,0.92)";
    context.strokeStyle = "rgba(67,38,18,0.95)";
    context.lineWidth = 1.5;
    context.beginPath();
    context.arc(point.x, point.y, 5, 0, 2 * Math.PI);
    context.fill();
    context.stroke();
    context.font = "700 9px Inter, ui-sans-serif, system-ui";
    context.textAlign = "left";
    context.fillStyle = "rgba(255,214,174,0.94)";
    context.fillText("MUJOCO MEASURED · COLLISION WIREFRAME", point.x + 9, point.y - 8);
    const twist = plantState?.root_twist_world || [0, 0, 0, 0, 0, 0];
    const velocity = twist.slice(3).map((value) => Number(value) * 0.18);
    if (Math.hypot(...velocity) > 0.002) {
      const end = project(base.translation.map((value, axis) => value + velocity[axis]));
      context.strokeStyle = "rgba(255,181,111,0.86)";
      context.beginPath();
      context.moveTo(point.x, point.y);
      context.lineTo(end.x, end.y);
      context.stroke();
    }
  }
  context.restore();
}

function drawPreviewSourceLabel() {
  if (interactionMode !== "target") return;
  const torso = frames.find((frame) => frame.name === "torso") || frames[0];
  if (!torso) return;
  const point = project(torso.translation);
  context.save();
  context.font = "700 9px Inter, ui-sans-serif, system-ui";
  context.textAlign = "left";
  context.fillStyle = "rgba(174,255,216,0.92)";
  context.fillText("WBC TARGET PREVIEW · NOT PLANT", point.x + 11, point.y + 13);
  context.restore();
}

function drawGrid(width, height) {
  context.save();
  const normal = normalizedPlaneNormal();
  const reference = Math.abs(normal[2]) < 0.9 ? [0, 0, 1] : [1, 0, 0];
  const rawAxisX = crossVector(reference, normal);
  const axisXLength = Math.max(Math.hypot(...rawAxisX), 1e-12);
  const axisX = rawAxisX.map((value) => value / axisXLength);
  const axisY = crossVector(normal, axisX);
  const planePoint = (x, y) => simulatorGroundPlane.point.map(
    (value, axis) => value + axisX[axis] * x + axisY[axis] * y,
  );
  const groundCorners = [
    planePoint(-1.2, -1.2), planePoint(1.2, -1.2),
    planePoint(1.2, 1.2), planePoint(-1.2, 1.2),
  ].map(project);
  context.beginPath();
  context.moveTo(groundCorners[0].x, groundCorners[0].y);
  for (let index = 1; index < groundCorners.length; index += 1) {
    context.lineTo(groundCorners[index].x, groundCorners[index].y);
  }
  context.closePath();
  context.fillStyle = "rgba(61,80,71,0.13)";
  context.strokeStyle = "rgba(102,128,120,0.42)";
  context.fill();
  context.stroke();
  context.lineWidth = 1;
  const extent = 1.2;
  const step = 0.1;
  for (let index = -12; index <= 12; index += 1) {
    const coordinate = index * step;
    const alongXStart = project(planePoint(-extent, coordinate));
    const alongXEnd = project(planePoint(extent, coordinate));
    const alongYStart = project(planePoint(coordinate, -extent));
    const alongYEnd = project(planePoint(coordinate, extent));
    context.strokeStyle = index === 0 ? "#34413d" : "#171d22";
    context.beginPath();
    context.moveTo(alongXStart.x, alongXStart.y);
    context.lineTo(alongXEnd.x, alongXEnd.y);
    context.stroke();
    context.beginPath();
    context.moveTo(alongYStart.x, alongYStart.y);
    context.lineTo(alongYEnd.x, alongYEnd.y);
    context.stroke();
  }
  const origin = project(simulatorGroundPlane.point);
  context.fillStyle = "#668078";
  context.beginPath();
  context.arc(origin.x, origin.y, 2, 0, Math.PI * 2);
  context.fill();
  context.fillStyle = "rgba(157,184,174,0.88)";
  context.font = "700 9px SFMono-Regular, Consolas, monospace";
  context.textAlign = "left";
  context.fillText(
    `MUJOCO GROUND · z=${Number(simulatorGroundPlane.point[2]).toFixed(3)} m`,
    origin.x + 7,
    origin.y + 14,
  );
  context.restore();
}

function drawWorldSdfLayer() {
  if (!worldSdfPlanes.length) return;
  const margin = latestMetrics?.world_collision_minimum_margin_m;
  const thresholds = authorityThresholds.world_collision_margin_m
    || authorityThresholds.command_clearance_m;
  const pressure = Number.isFinite(margin)
    ? lowerPressure(margin + (thresholds?.critical || 0), thresholds)
    : 0;
  const color = pressure >= 1 ? "239,117,106" : pressure >= 0.7 ? "224,177,90" : "102,172,197";
  context.save();
  context.lineJoin = "round";
  for (const plane of worldSdfPlanes) {
    const halfX = plane.extent_x_m * 0.5;
    const halfZ = plane.extent_z_m * 0.5;
    const [cx, cy, cz] = plane.point;
    const corners = [
      [cx - halfX, cy, cz - halfZ],
      [cx + halfX, cy, cz - halfZ],
      [cx + halfX, cy, cz + halfZ],
      [cx - halfX, cy, cz + halfZ],
    ].map(project);
    context.beginPath();
    context.moveTo(corners[0].x, corners[0].y);
    for (let index = 1; index < corners.length; index += 1) {
      context.lineTo(corners[index].x, corners[index].y);
    }
    context.closePath();
    context.fillStyle = `rgba(${color},${0.035 + 0.07 * pressure})`;
    context.strokeStyle = `rgba(${color},${0.45 + 0.35 * pressure})`;
    context.lineWidth = pressure >= 0.7 ? 1.5 : 1;
    context.fill();
    context.stroke();
    context.setLineDash([3, 5]);
    for (let step = 1; step < 8; step += 1) {
      const x = cx - halfX + plane.extent_x_m * step / 8;
      const a = project([x, cy, cz - halfZ]);
      const b = project([x, cy, cz + halfZ]);
      context.beginPath(); context.moveTo(a.x, a.y); context.lineTo(b.x, b.y); context.stroke();
    }
    for (let step = 1; step < 10; step += 1) {
      const z = cz - halfZ + plane.extent_z_m * step / 10;
      const a = project([cx - halfX, cy, z]);
      const b = project([cx + halfX, cy, z]);
      context.beginPath(); context.moveTo(a.x, a.y); context.lineTo(b.x, b.y); context.stroke();
    }
    context.setLineDash([]);
    const label = project([cx - halfX, cy, cz + halfZ]);
    context.fillStyle = `rgba(${color},0.85)`;
    context.font = "600 8px SFMono-Regular, Consolas, monospace";
    context.textAlign = "left";
    context.fillText(plane.label || "WORLD SDF", label.x + 5, label.y - 5);
  }
  context.restore();
}

function supportPointWorld(point) {
  const body = frames[point.body];
  if (!body) return null;
  return addVector(
    body.translation,
    quaternionRotate(body.rotation_xyzw, point.point_in_body),
  );
}

function drawSupportLayer() {
  if (!frames.length || !supportPatches.length) return;
  const supportMargin = latestMetrics?.minimum_support_margin_m;
  const limitingPatch = latestMetrics?.limiting_support_patch;
  const thresholds = authorityThresholds.support_margin_m;
  context.save();
  context.lineJoin = "round";
  for (const patch of supportPatches) {
    const world = patch.points.map(supportPointWorld).filter(Boolean);
    if (world.length < 3) continue;
    const projected = world.map(project);
    const limiting = patch.stable_id === limitingPatch && Number.isFinite(supportMargin);
    const pressure = limiting ? lowerPressure(supportMargin, thresholds) : 0;
    const critical = limiting && thresholds && supportMargin <= thresholds.critical + 1e-9;
    const warning = limiting && thresholds && supportMargin < thresholds.warning;
    const color = critical ? "239,117,106" : warning ? "224,177,90" : "119,212,174";
    context.beginPath();
    context.moveTo(projected[0].x, projected[0].y);
    for (let index = 1; index < projected.length; index += 1) {
      context.lineTo(projected[index].x, projected[index].y);
    }
    context.closePath();
    context.fillStyle = `rgba(${color},${limiting ? 0.10 + 0.16 * pressure : 0.07})`;
    context.strokeStyle = `rgba(${color},${limiting ? 0.95 : 0.55})`;
    context.lineWidth = limiting ? 2 : 1;
    context.fill();
    context.stroke();
    for (const point of projected) {
      context.fillStyle = `rgba(${color},0.95)`;
      context.beginPath();
      context.arc(point.x, point.y, limiting ? 3 : 2, 0, 2 * Math.PI);
      context.fill();
    }
  }
  const com = latestMetrics?.center_of_mass_world;
  if (Array.isArray(com) && com.length === 3 && com.every(Number.isFinite)) {
    const point = project(com);
    const ground = project([com[0], com[1], 0]);
    const pressure = Number.isFinite(supportMargin)
      ? lowerPressure(supportMargin, thresholds)
      : 0;
    const color = pressure >= 1 ? "#ef756a" : pressure >= 0.7 ? "#e0b15a" : "#8fe6c0";
    context.setLineDash([3, 4]);
    context.strokeStyle = color;
    context.lineWidth = 1;
    context.beginPath();
    context.moveTo(point.x, point.y);
    context.lineTo(ground.x, ground.y);
    context.stroke();
    context.setLineDash([]);
    context.fillStyle = color;
    context.beginPath();
    context.arc(ground.x, ground.y, 4, 0, 2 * Math.PI);
    context.fill();
  }
  context.restore();
}

function drawTinyAuthorityBar(x, y, label, pressure, value, unavailable = false) {
  const width = 42;
  const height = 4;
  const boundedPressure = clampUnit(pressure);
  const color = unavailable
    ? "#66717a"
    : boundedPressure >= 1
      ? "#ef756a"
      : boundedPressure >= 0.7 ? "#e0b15a" : "#77d4ae";
  context.fillStyle = "#0d1216d9";
  context.fillRect(x - 3, y - 10, width + 50, 15);
  context.fillStyle = unavailable ? "#6d7881" : "#aab6b1";
  context.font = "600 7px SFMono-Regular, Consolas, monospace";
  context.textAlign = "left";
  context.fillText(label, x, y - 3);
  context.fillStyle = "#222b31";
  context.fillRect(x + 24, y - 7, width, height);
  context.fillStyle = color;
  context.fillRect(x + 24, y - 7, unavailable ? 2 : width * boundedPressure, height);
  context.fillStyle = color;
  context.fillText(value, x + 70, y - 3);
}

function drawAuthorityAnnotations() {
  if (!latestMetrics || !frames.length) return;
  const handle = [...interactionHandles.values()].find((candidate) => candidate.kind === "base");
  const anchorFrame = handle && frames.find((candidate) => candidate.name === handle.frame);
  const com = latestMetrics.center_of_mass_world;
  const anchorWorld = anchorFrame?.translation
    || (Array.isArray(com) ? com : frames[0]?.translation);
  if (!anchorWorld) return;
  const projected = project(anchorWorld);
  const x = Math.max(8, Math.min(canvas.clientWidth - 116, projected.x + 22));
  let y = Math.max(82, Math.min(canvas.clientHeight - 66, projected.y - 36));

  if (interactionMode === "push" && plantState?.metrics) {
    const physical = plantState.metrics;
    const capturePressure = clampUnit(Number(physical.capture_pressure || 0));
    drawTinyAuthorityBar(x, y, "CAP", capturePressure, capturePressure.toFixed(2));
    y += 16;
    const stationPressure = 1 - clampUnit(Number(physical.station_authority || 0));
    drawTinyAuthorityBar(
      x,
      y,
      "STA",
      stationPressure,
      `${Math.abs(1000 * Number(physical.station_error_m || 0)).toFixed(0)}mm`,
    );
    y += 16;
    const torque = Number(physical.torque_utilization);
    drawTinyAuthorityBar(
      x,
      y,
      "ACT",
      Number.isFinite(torque) ? torque : 0,
      Number.isFinite(torque) ? `${Math.round(torque * 100)}%` : "N/A",
      !Number.isFinite(torque),
    );
    y += 16;
    const fallSafeFresh = clampUnit(Number(physical.fall_safe_fresh_command_authority ?? 1));
    drawTinyAuthorityBar(
      x,
      y,
      "LEASE",
      1 - fallSafeFresh,
      `${Math.round(100 * fallSafeFresh)}%`,
    );
    y += 16;
    const solveUs = Number(physical.controller_step_us);
    drawTinyAuthorityBar(
      x,
      y,
      "CPU",
      upperPressure(solveUs, authorityThresholds.solver_wall_time_us),
      Number.isFinite(solveUs) ? `${Math.round(solveUs)}µs` : "N/A",
      !Number.isFinite(solveUs),
    );
    y += 16;
    const residual = Math.max(
      Math.abs(Number(physical.wbc_dynamics_residual || 0)),
      Math.abs(Number(physical.wbc_contact_residual || 0)),
      Math.abs(Number(physical.wbc_maximum_constraint_violation || 0)),
    );
    drawTinyAuthorityBar(
      x,
      y,
      "RES",
      upperPressure(residual, authorityThresholds.hard_residual),
      Number.isFinite(residual) ? residual.toExponential(1) : "N/A",
      !Number.isFinite(residual),
    );
    if (Boolean(physical.wbc_support_contingency_enabled)) {
      y += 16;
      const selected = Boolean(physical.wbc_support_contingency_selected);
      const requested = Boolean(physical.wbc_support_contingency_requested);
      drawTinyAuthorityBar(
        x,
        y,
        "CTG",
        selected ? 0.72 : requested ? 0.42 : 0,
        selected ? "ON" : requested ? "REQ" : "idle",
        false,
      );
    }
    if (Boolean(physical.wbc_single_support_reacquisition_enabled)) {
      y += 16;
      const active = Boolean(physical.wbc_single_support_reacquisition_active);
      const authority = clampUnit(Number(physical.wbc_single_support_reacquisition_authority || 0));
      const diagnostics = Array.isArray(physical.wbc_single_support_reacquisition_diagnostics)
        ? physical.wbc_single_support_reacquisition_diagnostics
        : [];
      const error = Number(diagnostics[7]);
      drawTinyAuthorityBar(
        x,
        y,
        "TDN",
        active ? authority : 0,
        active && Number.isFinite(error) ? `${Math.round(error * 1000)}mm` : "idle",
        false,
      );
    }
    return;
  }

  const supportMargin = latestMetrics.minimum_support_margin_m;
  const supportUnavailable = !Number.isFinite(supportMargin);
  const supportPressure = supportUnavailable
    ? 0
    : lowerPressure(supportMargin, authorityThresholds.support_margin_m);
  drawTinyAuthorityBar(
    x,
    y,
    supportUnavailable ? "ROLL" : "SUP",
    supportPressure,
    supportUnavailable ? "N/A" : `${(supportMargin * 1000).toFixed(0)}mm`,
    supportUnavailable,
  );
  y += 16;

  const previewAdmitted = latestMetrics.guided_preview_wbc_admitted;
  if (previewAdmitted !== null && previewAdmitted !== undefined) {
    drawTinyAuthorityBar(
      x,
      y,
      latestMetrics.interaction_target_clamped ? "TGT" : "WBC",
      previewAdmitted && !latestMetrics.interaction_target_clamped ? 0 : 1,
      latestMetrics.interaction_target_clamped
        ? "LIMIT"
        : previewAdmitted ? "OK" : "NO",
    );
    y += 16;
  }

  const actuatorUse = latestMetrics.maximum_torque_utilization;
  drawTinyAuthorityBar(
    x,
    y,
    "ACT",
    Number.isFinite(actuatorUse) ? actuatorUse : 0,
    Number.isFinite(actuatorUse) ? `${Math.round(actuatorUse * 100)}%` : "N/A",
    !Number.isFinite(actuatorUse),
  );
  y += 16;

  const jointMargin = latestMetrics.minimum_joint_margin_rad;
  const jointPressure = Number.isFinite(jointMargin)
    ? lowerPressure(jointMargin, authorityThresholds.joint_margin_rad)
    : 0;
  drawTinyAuthorityBar(
    x,
    y,
    "JNT",
    jointPressure,
    Number.isFinite(jointMargin) ? `${(jointMargin * 180 / Math.PI).toFixed(0)}°` : "N/A",
    !Number.isFinite(jointMargin),
  );
  y += 16;

  const stoppingMargin = Number.isFinite(latestMetrics.minimum_joint_velocity_stopping_headroom_rad)
    ? latestMetrics.minimum_joint_velocity_stopping_headroom_rad
    : latestMetrics.minimum_joint_stopping_margin_rad_s2;
  const stoppingThresholds = Number.isFinite(latestMetrics.minimum_joint_velocity_stopping_headroom_rad)
    ? authorityThresholds.joint_margin_rad
    : authorityThresholds.joint_stopping_margin_rad_s2;
  const stoppingPressure = Number.isFinite(stoppingMargin)
    ? lowerPressure(stoppingMargin, stoppingThresholds)
    : 0;
  drawTinyAuthorityBar(
    x,
    y,
    "STP",
    stoppingPressure,
    Number.isFinite(stoppingMargin) ? `${Math.round((1 - stoppingPressure) * 100)}%` : "N/A",
    !Number.isFinite(stoppingMargin),
  );
  y += 16;

  const solveUs = latestMetrics.solve_us;
  drawTinyAuthorityBar(
    x,
    y,
    "CPU",
    upperPressure(solveUs, authorityThresholds.solver_wall_time_us),
    Number.isFinite(solveUs) ? `${Math.round(solveUs)}µs` : "N/A",
    !Number.isFinite(solveUs),
  );
}

function draw() {
  const bounds = canvas.getBoundingClientRect();
  context.clearRect(0, 0, bounds.width, bounds.height);
  drawGrid(bounds.width, bounds.height);
  drawWorldSdfLayer();
  if (!frames.length) return;
  drawMeasuredPlantLayer();
  drawSupportLayer();
  const geometryStartedAt = performance.now();
  drawGeometryLayer();
  drawMeasuredPlantCollisionLayer();
  drawPreviewSourceLabel();
  pushBounded(viewportPerformance.geometryDurations, performance.now() - geometryStartedAt);
  drawPlantContactLayer();

  context.save();
  context.lineCap = "round";
  context.lineJoin = "round";
  if (showRig) {
    const depthSortedBones = bones.map((bone) => ({
      bone,
      depth: (project(frames[bone.parent]?.translation ?? [0, 0, 0]).depth
        + project(frames[bone.child]?.translation ?? [0, 0, 0]).depth) / 2,
    })).sort((a, b) => b.depth - a.depth);
    for (const { bone } of depthSortedBones) {
      const parent = frames[bone.parent];
      const child = frames[bone.child];
      if (!parent || !child) continue;
      const a = project(parent.translation);
      const b = project(child.translation);
      const active = selected && (parent.name === selected.name || child.name === selected.name);
      context.strokeStyle = active ? "#8fe6c0" : "#606b78";
      context.lineWidth = active ? 3 : 2;
      context.shadowColor = active ? "#8fe6c088" : "transparent";
      context.shadowBlur = active ? 12 : 0;
      context.beginPath(); context.moveTo(a.x, a.y); context.lineTo(b.x, b.y); context.stroke();
    }
    context.shadowBlur = 0;
    const depthSortedFrames = [...frames].sort(
      (a, b) => project(b.translation).depth - project(a.translation).depth,
    );
    for (const frame of depthSortedFrames) {
      const point = project(frame.translation);
      const active = selected?.name === frame.name;
      context.fillStyle = active ? "#0d1713" : "#11161b";
      context.strokeStyle = active ? "#a9f3d2" : "#77818e";
      context.lineWidth = active ? 2 : 1;
      context.beginPath();
      context.arc(point.x, point.y, active ? 6 : 3.5, 0, Math.PI * 2);
      context.fill(); context.stroke();
    }
  }
  const pushing = interactionMode === "push";
  for (const handle of interactionHandles.values()) {
    const frame = frames.find((candidate) => candidate.name === handle.frame);
    if (!frame) continue;
    const point = project(frame.translation);
    const active = selected?.name === frame.name;
    const primary = handle.kind === "base";
    const radius = active ? 12 : primary ? 10 : 8;
    const pulse = 2 + 2 * (0.5 + 0.5 * Math.sin(performance.now() * 0.004));
    context.shadowColor = pushing ? "#ff9d45cc" : "#5ee6a5cc";
    context.shadowBlur = active ? 22 : 15;
    context.fillStyle = active ? (pushing ? "#ffe0bd" : "#bff8dc") : (pushing ? "#ff9d45" : "#5ee6a5");
    context.strokeStyle = active ? "#effff7" : (pushing ? "#ffd2a1" : "#c6fbe2");
    context.lineWidth = active ? 3 : 2.5;
    context.beginPath();
    context.arc(point.x, point.y, radius + pulse, 0, Math.PI * 2);
    context.globalAlpha = 0.42;
    context.stroke();
    context.globalAlpha = 1;
    context.beginPath();
    context.arc(point.x, point.y, radius, 0, Math.PI * 2);
    context.fill(); context.stroke();
    context.shadowBlur = 0;
    context.strokeStyle = active ? (pushing ? "#55321b" : "#193d2e") : (pushing ? "#4b2a15" : "#103326");
    context.lineWidth = 1;
    context.beginPath();
    context.moveTo(point.x - radius - 4, point.y);
    context.lineTo(point.x + radius + 4, point.y);
    context.moveTo(point.x, point.y - radius - 4);
    context.lineTo(point.x, point.y + radius + 4);
    context.stroke();
    context.fillStyle = pushing ? "#ffc88f" : "#9cebc9";
    context.font = "600 9px Inter, ui-sans-serif, system-ui";
    context.textAlign = "center";
    const labelWidth = context.measureText(handle.label).width + 8;
    context.fillStyle = "#07100cef";
    context.fillRect(point.x - labelWidth / 2, point.y - radius - 22, labelWidth, 14);
    context.fillStyle = pushing ? "#fff0df" : "#d5ffea";
    context.fillText(handle.label, point.x, point.y - radius - 12);
  }
  drawAuthorityAnnotations();
  if (drag) {
    const point = project(drag.target);
    context.strokeStyle = "#8fe6c0";
    context.lineWidth = 1;
    context.setLineDash([4, 4]);
    context.beginPath();
    context.arc(point.x, point.y, 11, 0, Math.PI * 2);
    context.moveTo(point.x - 16, point.y); context.lineTo(point.x + 16, point.y);
    context.moveTo(point.x, point.y - 16); context.lineTo(point.x, point.y + 16);
    context.stroke();
    context.setLineDash([]);
  }
  if (activeForceArrow) {
    const start = project(activeForceArrow.start);
    const end = project(activeForceArrow.end);
    const angle = Math.atan2(end.y - start.y, end.x - start.x);
    context.strokeStyle = "#ff9d45";
    context.fillStyle = "#ffb56f";
    context.shadowColor = "#ff9d45aa";
    context.shadowBlur = 12;
    context.lineWidth = 4;
    context.beginPath();
    context.moveTo(start.x, start.y);
    context.lineTo(end.x, end.y);
    context.stroke();
    context.beginPath();
    context.moveTo(end.x, end.y);
    context.lineTo(end.x - 13 * Math.cos(angle - 0.45), end.y - 13 * Math.sin(angle - 0.45));
    context.lineTo(end.x - 13 * Math.cos(angle + 0.45), end.y - 13 * Math.sin(angle + 0.45));
    context.closePath();
    context.fill();
    context.shadowBlur = 0;
    context.font = "700 10px Inter, ui-sans-serif, system-ui";
    context.textAlign = "left";
    context.fillText(`${activeForceArrow.magnitude.toFixed(2)} N`, end.x + 9, end.y - 8);
  }
  context.restore();
}

function pointerRay(event) {
  const point = pointerPosition(event);
  const screenRight = (point.x - transform.centerX) / transform.focalPixels;
  const screenUp = -(point.y - transform.centerY) / transform.focalPixels;
  const direction = camera.forward.map((value, axis) => value
    + camera.right[axis] * screenRight
    + camera.up[axis] * screenUp);
  const length = Math.max(Math.hypot(...direction), 1e-12);
  return { origin: camera.position, direction: direction.map((value) => value / length) };
}

function rayTriangleDistance(origin, direction, a, b, c) {
  const edgeA = subtractVector(b, a);
  const edgeB = subtractVector(c, a);
  const p = crossVector(direction, edgeB);
  const determinant = dot(edgeA, p);
  if (Math.abs(determinant) < 1e-10) return null;
  const inverse = 1 / determinant;
  const offset = subtractVector(origin, a);
  const u = dot(offset, p) * inverse;
  if (u < 0 || u > 1) return null;
  const q = crossVector(offset, edgeA);
  const v = dot(direction, q) * inverse;
  if (v < 0 || u + v > 1) return null;
  const distance = dot(edgeB, q) * inverse;
  return distance > 0.02 ? distance : null;
}

function pickRenderedBody(event) {
  if (!showGeometry || !geometry.length || !frames.length) return null;
  const ray = pointerRay(event);
  const availableBodies = plantHello?.body_names
    ? new Set(plantHello.body_names)
    : null;
  let best = null;
  for (const shape of geometry) {
    if (!shape.surface) continue;
    const frame = frames[shape.body];
    if (!frame || (availableBodies && !availableBodies.has(frame.name))) continue;
    const vertices = shape.surface.bodyVertices.map(
      (vertex) => geometryVertexWorld(frame, vertex),
    );
    for (const face of shape.surface.faces) {
      for (let triangle = 1; triangle + 1 < face.length; triangle += 1) {
        const distance = rayTriangleDistance(
          ray.origin,
          ray.direction,
          vertices[face[0]],
          vertices[face[triangle]],
          vertices[face[triangle + 1]],
        );
        if (distance === null || (best && distance >= best.distance)) continue;
        best = {
          frame,
          distance,
          point: ray.origin.map(
            (value, axis) => value + ray.direction[axis] * distance,
          ),
        };
      }
    }
  }
  return best;
}

function nearestFrame(event) {
  const bounds = canvas.getBoundingClientRect();
  const pointer = { x: event.clientX - bounds.left, y: event.clientY - bounds.top };
  let best = null;
  const touchPadding = event.pointerType === "touch" ? 10 : 0;
  let bestDistance = 30 + touchPadding;
  for (const handle of interactionHandles.values()) {
    const frame = frames.find((candidate) => candidate.name === handle.frame);
    if (!frame) continue;
    const point = project(frame.translation);
    const distance = Math.hypot(point.x - pointer.x, point.y - pointer.y);
    if (distance < bestDistance) {
      best = frame;
      bestDistance = distance;
    }
  }
  if (best) return best;
  if (!showRig) return null;
  bestDistance = 18 + touchPadding;
  for (const frame of frames) {
    const point = project(frame.translation);
    const distance = Math.hypot(point.x - pointer.x, point.y - pointer.y);
    if (distance < bestDistance) {
      best = frame;
      bestDistance = distance;
    }
  }
  return best;
}

function pointerPosition(event) {
  const bounds = canvas.getBoundingClientRect();
  return { x: event.clientX - bounds.left, y: event.clientY - bounds.top };
}

function beginDrag(event, frame, captureTarget) {
  if (!frame) return;
  event.preventDefault();
  captureTarget.setPointerCapture(event.pointerId);
  canvas.classList.add("dragging");
  captureTarget.classList.add("dragging");
  selected = frame;
  drag = {
    frame,
    target: [...frame.translation],
    captureTarget,
    pointerId: event.pointerId,
    depth: project(frame.translation).depth,
  };
  updateSelection();
  if (interactionHandles.get(frame.name)?.kind === "base") {
    showToast(baseExecutionLabel());
  }
  moveDrag(event);
}

function beginPush(event, pick) {
  if (!pick?.frame || !plantGateway?.available) return;
  event.preventDefault();
  canvas.setPointerCapture(event.pointerId);
  canvas.classList.add("dragging");
  const frame = pick.frame;
  const applicationPoint = pick.point || frame.translation;
  selected = frame;
  pushDrag = {
    frame,
    requestId: ++plantRequestId,
    pointerId: event.pointerId,
    depth: project(applicationPoint).depth,
    start: [...applicationPoint],
    target: [...applicationPoint],
    force: [0, 0, 0],
  };
  updateSelection();
  movePush(event);
}

function currentPushCommand() {
  if (!plantGateway?.available || interactionMode !== "push") return null;
  if (!pushDrag) return null;
  return {
    type: "plant_push",
    request_id: pushDrag.requestId,
    body: pushDrag.frame.name,
    force_world: pushDrag.force,
    application_point_world: pushDrag.start,
    provenance: {
      source: "interactive_operator",
      load_class: "declared_continuous_wrench",
      force_frame: "world",
      application_point_frame: "world",
    },
  };
}

function movePush(event) {
  const point = pointerPosition(event);
  const target = unprojectViewPlane(point, pushDrag.depth);
  const rawForce = target.map(
    (value, axis) => (value - pushDrag.start[axis]) * PUSH_FORCE_GAIN_N_PER_M,
  );
  const magnitude = Math.hypot(...rawForce);
  const limit = Number(plantGateway?.maximum_force_n || plantHello?.maximum_force_n || 8);
  const scale = magnitude > limit ? limit / magnitude : 1;
  pushDrag.force = rawForce.map((value) => value * scale);
  pushDrag.target = pushDrag.start.map(
    (value, axis) => value + pushDrag.force[axis] / PUSH_FORCE_GAIN_N_PER_M,
  );
  activeForceArrow = {
    start: pushDrag.start,
    end: pushDrag.target,
    magnitude: Math.hypot(...pushDrag.force),
  };
  pendingPushCommand = currentPushCommand();
  scheduleRender();
}

function beginOrbit(event) {
  event.preventDefault();
  canvas.setPointerCapture(event.pointerId);
  canvas.classList.add("orbiting");
  orbitDrag = {
    pointerId: event.pointerId,
    lastX: event.clientX,
    lastY: event.clientY,
  };
}

function finishPointer(event) {
  if (orbitDrag?.pointerId === event.pointerId) {
    if (canvas.hasPointerCapture?.(event.pointerId)) {
      canvas.releasePointerCapture(event.pointerId);
    }
    orbitDrag = null;
    canvas.classList.remove("orbiting");
    scheduleRender();
    return;
  }
  if (pushDrag?.pointerId === event.pointerId) {
    if (canvas.hasPointerCapture?.(event.pointerId)) {
      canvas.releasePointerCapture(event.pointerId);
    }
    pushDrag = null;
    pendingPushCommand = null;
    activeForceArrow = null;
    canvas.classList.remove("dragging");
    sendPlant({ type: "plant_release" });
    if (pushReturnMode) {
      const returnMode = pushReturnMode;
      pushReturnMode = null;
      setInteractionMode(returnMode);
    }
    scheduleRender();
    return;
  }
  if (!drag || drag.pointerId !== event.pointerId) return;
  if (drag.captureTarget.hasPointerCapture?.(drag.pointerId)) {
    drag.captureTarget.releasePointerCapture(drag.pointerId);
  }
  drag.captureTarget.classList.remove("dragging");
  canvas.classList.remove("dragging");
  if (pendingDragCommand) {
    send(pendingDragCommand);
    pendingDragCommand = null;
  }
  drag = null;
  send({ type: "release" });
  scheduleRender();
}

canvas.addEventListener("pointerdown", (event) => {
  if (!robotControlsEnabled) return;
  if (event.button !== 0) return;
  const bodyPick = (event.ctrlKey || interactionMode === "push")
    ? pickRenderedBody(event)
    : null;
  if (event.ctrlKey) {
    if (!plantGateway?.available) {
      showToast("Physical MuJoCo plant is unavailable");
      return;
    }
    if (!bodyPick) {
      showToast("Ctrl+drag must start on rendered robot geometry");
      return;
    }
    if (interactionMode !== "push") {
      pushReturnMode = interactionMode;
      setInteractionMode("push");
    }
    beginPush(event, bodyPick);
    return;
  }
  if (event.shiftKey) {
    beginOrbit(event);
    return;
  }
  if (interactionMode === "push" && bodyPick) {
    beginPush(event, bodyPick);
    return;
  }
  const frame = nearestFrame(event);
  if (frame) beginDrag(event, frame, canvas);
  else beginOrbit(event);
});

canvas.addEventListener("pointermove", (event) => {
  if (!robotControlsEnabled) return;
  if (pushDrag?.pointerId === event.pointerId) movePush(event);
  else if (drag?.pointerId === event.pointerId) moveDrag(event);
  else if (orbitDrag?.pointerId === event.pointerId) moveOrbit(event);
  else canvas.classList.toggle(
    "joint-hover",
    event.ctrlKey ? Boolean(pickRenderedBody(event)) : Boolean(nearestFrame(event)),
  );
});

canvas.addEventListener("pointerleave", () => {
  if (!drag && !pushDrag && !orbitDrag) canvas.classList.remove("joint-hover");
});
canvas.addEventListener("pointerup", finishPointer);
canvas.addEventListener("pointercancel", finishPointer);

function moveDrag(event) {
  const point = pointerPosition(event);
  drag.target = unprojectViewPlane(point, drag.depth);
  if (pendingDragCommand) viewportPerformance.coalescedDrags += 1;
  pendingDragCommand = { type: "drag", frame: drag.frame.name, target: drag.target };
  updateSelection(drag.target);
  scheduleRender();
}

function moveOrbit(event) {
  const deltaX = event.clientX - orbitDrag.lastX;
  const deltaY = event.clientY - orbitDrag.lastY;
  orbitDrag.lastX = event.clientX;
  orbitDrag.lastY = event.clientY;
  if (viewportPerformance.pendingCameraInputAt === null) {
    viewportPerformance.pendingCameraInputAt = performance.now();
  }
  if (event.shiftKey) {
    const metersPerPixel = camera.distance / Math.max(transform.focalPixels, 1);
    camera.target = camera.target.map((value, index) => value
      - camera.right[index] * deltaX * metersPerPixel
      + camera.up[index] * deltaY * metersPerPixel);
  } else {
    camera.yaw = ((camera.yaw - deltaX * 0.008 + Math.PI) % (Math.PI * 2)) - Math.PI;
    camera.pitch = Math.max(
      -Math.PI * 0.42,
      Math.min(Math.PI * 0.42, camera.pitch + deltaY * 0.008),
    );
  }
  updateCameraBasis();
  scheduleRender();
}

canvas.addEventListener("wheel", (event) => {
  if (!robotControlsEnabled) return;
  event.preventDefault();
  camera.distance = Math.max(
    0.7,
    Math.min(4.5, camera.distance * Math.exp(event.deltaY * 0.0012)),
  );
  updateCameraBasis();
  scheduleRender();
}, { passive: false });

function scheduleRender() {
  if (renderScheduled) return;
  renderScheduled = true;
  requestAnimationFrame(renderFrame);
}

function pushBounded(values, value, capacity = 240) {
  values.push(value);
  if (values.length > capacity) values.shift();
}

function enqueueState(message) {
  const arrivedAt = performance.now();
  const snapshot = {
    message,
    arrivedAt,
    frames: message.frames.map((frame) => ({ ...frame, name: frameNames[frame.id] })),
    presented: false,
  };
  if (latestSnapshot) {
    if (message.reset_epoch !== latestSnapshot.message.reset_epoch) {
      previousSnapshot = snapshot;
      latestSnapshot = snapshot;
      selected = null;
      return;
    }
    const interval = arrivedAt - latestSnapshot.arrivedAt;
    if (Number.isFinite(interval) && interval > 0 && interval < 1000) {
      pushBounded(viewportPerformance.snapshotIntervals, interval);
      snapshotPeriodMs = 0.9 * snapshotPeriodMs + 0.1 * interval;
    }
    if (!latestSnapshot.presented) viewportPerformance.coalescedStates += 1;
    previousSnapshot = latestSnapshot;
  } else {
    previousSnapshot = snapshot;
  }
  latestSnapshot = snapshot;
  if (
    viewportPerformance.lastDragSentAt !== null
    && message.active_frame === viewportPerformance.lastDragFrame
    && message.command_id === viewportPerformance.lastDragCommandId
  ) {
    pushBounded(
      viewportPerformance.commandRoundTrips,
      arrivedAt - viewportPerformance.lastDragSentAt,
    );
    viewportPerformance.lastDragSentAt = null;
    viewportPerformance.lastDragFrame = null;
    viewportPerformance.lastDragCommandId = null;
  }
}

function quaternionSlerp(left, right, alpha) {
  let cosine = left.reduce((sum, value, index) => sum + value * right[index], 0);
  let target = right;
  if (cosine < 0) {
    cosine = -cosine;
    target = right.map((value) => -value);
  }
  if (cosine > 0.9995) {
    const blended = left.map((value, index) => value + alpha * (target[index] - value));
    const length = Math.max(
      Math.sqrt(blended.reduce((sum, value) => sum + value * value, 0)),
      Number.EPSILON,
    );
    return blended.map((value) => value / length);
  }
  const angle = Math.acos(Math.min(1, Math.max(-1, cosine)));
  const denominator = Math.sin(angle);
  const leftWeight = Math.sin((1 - alpha) * angle) / denominator;
  const rightWeight = Math.sin(alpha * angle) / denominator;
  return left.map((value, index) => leftWeight * value + rightWeight * target[index]);
}

function interpolatedFrames(now) {
  if (!latestSnapshot) return frames;
  latestSnapshot.presented = true;
  if (!previousSnapshot || previousSnapshot === latestSnapshot) return latestSnapshot.frames;
  const interval = Math.max(latestSnapshot.arrivedAt - previousSnapshot.arrivedAt, 1);
  const presentationDelay = Math.max(16, Math.min(50, snapshotPeriodMs));
  const alpha = Math.max(
    0,
    Math.min(1, (now - presentationDelay - previousSnapshot.arrivedAt) / interval),
  );
  return latestSnapshot.frames.map((current, index) => {
    const prior = previousSnapshot.frames[index];
    if (!prior) return current;
    return {
      ...current,
      translation: current.translation.map(
        (value, axis) => prior.translation[axis] + alpha * (value - prior.translation[axis]),
      ),
      rotation_xyzw: quaternionSlerp(prior.rotation_xyzw, current.rotation_xyzw, alpha),
    };
  });
}

function percentile(values, fraction) {
  if (!values.length) return 0;
  const ordered = [...values].sort((left, right) => left - right);
  return ordered[Math.min(ordered.length - 1, Math.ceil(fraction * ordered.length) - 1)];
}

function performanceLine(label, values) {
  if (!values.length) return `${label} awaiting samples`;
  return `${label} p50 ${percentile(values, 0.5).toFixed(2)} · p95 ${percentile(values, 0.95).toFixed(2)} · p99 ${percentile(values, 0.99).toFixed(2)} · max ${Math.max(...values).toFixed(2)} ms`;
}

function updatePerformancePanel(now) {
  if (!performanceEnabled || now - viewportPerformance.lastPanelAt < 500) return;
  viewportPerformance.lastPanelAt = now;
  performancePanel.hidden = false;
  const missed = viewportPerformance.frameIntervals.filter((value) => value > 16.7).length;
  const missedPercent = viewportPerformance.frameIntervals.length
    ? 100 * missed / viewportPerformance.frameIntervals.length
    : 0;
  performancePanel.textContent = [
    "PRESENTATION PROFILER · ?perf=1",
    performanceLine("frame", viewportPerformance.frameIntervals),
    performanceLine("draw", viewportPerformance.drawDurations),
    performanceLine("geometry", viewportPerformance.geometryDurations),
    performanceLine("snapshot", viewportPerformance.snapshotIntervals),
    performanceLine("command RTT", viewportPerformance.commandRoundTrips),
    performanceLine("telemetry", viewportPerformance.telemetryDurations),
    `missed 16.7 ms ${missedPercent.toFixed(1)}% · state coalesced ${viewportPerformance.coalescedStates}`,
    `drag sent ${viewportPerformance.transmittedDrags} · coalesced ${viewportPerformance.coalescedDrags}`,
    performanceLine("pointer→camera", viewportPerformance.cameraLatencies),
    `controller solve ${Number(latestMetrics?.solve_us || 0).toFixed(1)} µs`,
  ].join("\n");
}

function renderFrame(now) {
  renderScheduled = false;
  if (viewportPerformance.lastFrameAt !== null) {
    pushBounded(viewportPerformance.frameIntervals, now - viewportPerformance.lastFrameAt);
  }
  viewportPerformance.lastFrameAt = now;
  if (latestSnapshot) {
    // Preserve the last plant frame for the disconnected ghost, but do not
    // let its metrics continue to feed authority bars or collision overlays.
    const stalePlantSnapshot = latestSnapshot.message.source === "plant"
      && !plantStateFresh;
    latestMetrics = stalePlantSnapshot ? null : latestSnapshot.message.metrics;
    frames = interpolatedFrames(now);
    if (
      latestSnapshot.message.tick !== lastTelemetryTick
      && now - lastTelemetryUpdateMs >= TELEMETRY_INTERVAL_MS
    ) {
      const telemetryStartedAt = performance.now();
      if (latestSnapshot.message.source === "plant") {
        if (plantState) updatePlantTelemetry(plantState);
      } else {
        updateObservationTransport(latestSnapshot.message.metrics);
        updateInteractionNotice(latestSnapshot.message.metrics);
        updateTelemetry(latestSnapshot.message);
      }
      pushBounded(viewportPerformance.telemetryDurations, performance.now() - telemetryStartedAt);
      lastTelemetryUpdateMs = now;
      lastTelemetryTick = latestSnapshot.message.tick;
    }
  }
  if (pendingDragCommand) {
    const sentCommandId = send(pendingDragCommand);
    viewportPerformance.transmittedDrags += 1;
    viewportPerformance.lastDragSentAt = performance.now();
    viewportPerformance.lastDragFrame = pendingDragCommand.frame;
    viewportPerformance.lastDragCommandId = sentCommandId;
    pendingDragCommand = null;
  }
  if (pushDrag && now - lastPushSentAt >= 50) {
    pendingPushCommand = currentPushCommand();
  }
  if (pendingPushCommand) {
    sendPlant(pendingPushCommand);
    pendingPushCommand = null;
    lastPushSentAt = now;
  }
  const drawStartedAt = performance.now();
  draw();
  pushBounded(viewportPerformance.drawDurations, performance.now() - drawStartedAt);
  if (viewportPerformance.pendingCameraInputAt !== null) {
    pushBounded(
      viewportPerformance.cameraLatencies,
      now - viewportPerformance.pendingCameraInputAt,
    );
    viewportPerformance.pendingCameraInputAt = null;
  }
  updatePerformancePanel(now);
  if (robotControlsEnabled || (interactionMode === "push" && plantConnected)) scheduleRender();
}

function send(message) {
  let outgoing = message;
  if (
    (message.type === "drag" || message.type === "release")
    && message.request_id === undefined
  ) {
    commandRequestId += 1;
    outgoing = { ...message, request_id: commandRequestId };
  }
  if (socket?.readyState === WebSocket.OPEN) socket.send(JSON.stringify(outgoing));
  return outgoing.request_id ?? null;
}

function updateObservationTransport(metrics) {
  const mode = metrics.robot_observation_transport_mode || "exact";
  observationTransportButtons.forEach((button) => {
    button.setAttribute("aria-pressed", String(button.dataset.observationMode === mode));
  });
  const base = {
    exact: "Every 20 ms · zero lookback",
    interpolated: "Every 20 ms · 10 ms control lookback",
    predicted: "25 Hz producer · 20 ms prediction on alternate WBC ticks",
    stale: "Producer paused · WBC fails closed after 20 ms horizon",
  }[mode] || mode;
  if (!Number.isFinite(metrics.robot_observation_transport_query_time_ns)) {
    observationTransportDetail.textContent = base;
    return;
  }
  const counts = [
    metrics.robot_observation_frame_exact_queries || 0,
    metrics.robot_observation_frame_interpolated_queries || 0,
    metrics.robot_observation_frame_predicted_queries || 0,
    metrics.robot_observation_frame_held_queries || 0,
  ];
  observationTransportDetail.textContent = `${base} · frame E/I/P/H ${counts.join("/")} · sample ${metrics.robot_observation_transport_sample_emitted ? "emitted" : "skipped"}`;
}

function updateInteractionNotice(metrics) {
  let notice = "ok";
  let message = null;
  if (metrics.interaction_target_clamped) {
    notice = "clamped";
    const errorMm = Math.abs(metrics.interaction_target_clamp_error_m || 0) * 1000;
    message = `Target limit reached · ${errorMm.toFixed(0)} mm beyond preview range · stream continues`;
  } else if (metrics.guided_preview_wbc_admitted === false) {
    notice = "wbc_infeasible";
    message = "WBC cannot admit this pose · kinematic preview continues";
  }
  if (notice !== lastInteractionNotice && message) showToast(message);
  lastInteractionNotice = notice;
}

function updateSelection(target = selected?.translation) {
  if (!selected) return;
  document.querySelector("#selection-empty").classList.add("hidden");
  document.querySelector("#selection-detail").classList.remove("hidden");
  document.querySelector("#selected-name").textContent = selected.name;
  document.querySelector("#selected-x").textContent = target[0].toFixed(3);
  document.querySelector("#selected-y").textContent = target[1].toFixed(3);
  document.querySelector("#selected-z").textContent = target[2].toFixed(3);
  const handle = interactionHandles.get(selected.name);
  document.querySelector("#selection-mode").textContent =
    handle?.kind === "base" ? baseExecutionLabel() : "2 · Intent";
}

function updateTelemetry(message) {
  const metrics = message.metrics;
  const taskNames = {
    1: "root angular",
    2: "root horizontal",
    3: "root height",
    4: "joint posture",
    5: "wheel acceleration",
    6: "center of mass",
    7: "centroidal momentum",
    8: "contact force",
    9: "actuator torque",
  };
  document.querySelector("#solve-time").textContent = Math.round(metrics.solve_us).toLocaleString();
  document.querySelector("#intent-residual").textContent = metrics.intent_residual.toExponential(2);
  const activeTasks = (metrics.task_residuals ?? []).filter((task) => task.active);
  const clippedLevels = metrics.clipped_levels ?? [];
  document.querySelector("#clipped-levels").textContent =
    clippedLevels.length
      ? clippedLevels.map((level) => {
        const maximum = activeTasks
          .filter((task) => task.priority === level)
          .reduce((largest, task) => Math.max(largest, task.rms), 0);
        return `${level} (${maximum.toExponential(1)} RMS)`;
      }).join(" · ")
      : "none";
  const dominant = activeTasks.reduce(
    (largest, task) => (!largest || task.rms > largest.rms ? task : largest),
    null,
  );
  document.querySelector("#dominant-task").textContent = dominant
    ? `${taskNames[dominant.stable_id] ?? dominant.kind} · ${dominant.priority} · ${dominant.rms.toExponential(2)}`
    : "—";
  document.querySelector("#bound-margin").textContent = Number.isFinite(metrics.min_bound_margin)
    ? metrics.min_bound_margin.toFixed(3) : "∞";
  document.querySelector("#tick-count").textContent = message.tick.toLocaleString();
  document.querySelector("#sample-count").textContent = metrics.sample_count
    ? `${metrics.sample_count} × 1 ms`
    : "1 × 20 ms";
  updateAuthorityStack(metrics, activeTasks);
  if (metrics.command_selection && Number.isFinite(metrics.primary_sampled_clearance_m)) {
    commandAuthorityHistory.push({
      sampled: Number.isFinite(metrics.primary_robust_sampled_clearance_m)
        ? metrics.primary_robust_sampled_clearance_m
        : metrics.primary_sampled_clearance_m,
      continuous: Number.isFinite(metrics.primary_robust_continuous_clearance_m)
        ? metrics.primary_robust_continuous_clearance_m
        : metrics.primary_continuous_clearance_m,
      required: metrics.command_clearance_requirement_m,
      selection: metrics.command_selection,
      refinements: metrics.primary_refinement_pair_samples,
      unresolved: metrics.primary_continuity_unresolved_intervals,
    });
    if (commandAuthorityHistory.length > 120) commandAuthorityHistory.shift();
    drawAuthorityHistory();
  }
  const pill = document.querySelector("#status-pill");
  pill.textContent = message.status.toUpperCase();
  pill.className = `status-pill ${message.status.toLowerCase()}`;
  solveHistory.push(metrics.solve_us);
  if (solveHistory.length > 120) solveHistory.shift();
  drawSparkline();
  if (selected) {
    const updated = frames.find((frame) => frame.name === selected.name);
    if (updated) {
      selected = updated;
      if (!drag) updateSelection(updated.translation);
    }
  }
}

function clampUnit(value) {
  return Math.min(1, Math.max(0, value));
}

function pressureState(pressure, warning = false, critical = false) {
  if (critical || pressure >= 1) return "critical";
  if (warning) return "warning";
  return "ok";
}

function upperPressure(value, thresholds) {
  if (!thresholds || !Number.isFinite(value)) return 0;
  if (value <= thresholds.warning) {
    return 0.7 * clampUnit(value / Math.max(thresholds.warning, Number.EPSILON));
  }
  return 0.7 + 0.3 * clampUnit(
    (value - thresholds.warning) / Math.max(thresholds.critical - thresholds.warning, Number.EPSILON),
  );
}

function lowerPressure(value, thresholds) {
  if (!thresholds || !Number.isFinite(value)) return 0;
  if (value >= thresholds.warning) return 0.7 * clampUnit(thresholds.warning / Math.max(value, Number.EPSILON));
  return 0.7 + 0.3 * clampUnit(
    (thresholds.warning - value) / Math.max(thresholds.warning - thresholds.critical, Number.EPSILON),
  );
}

function setLiveAuthorityRow(id, value, detail, pressure, state) {
  const row = document.querySelector(`#${id}`);
  row.className = `live-authority-row ${state}`;
  row.querySelector("output").textContent = value;
  const prior = authorityPersistence.get(id) || 0;
  const persistence = state === "warning" || state === "critical" ? prior + 1 : 0;
  authorityPersistence.set(id, persistence);
  row.querySelector(".live-authority-detail").textContent = persistence > 1
    ? `${detail} · ${persistence} ticks`
    : detail;
  row.querySelector("i > b").style.width = `${Math.round(100 * clampUnit(pressure))}%`;
}

function bodyPairLabel(bodyA, bodyB) {
  if (!Number.isInteger(bodyA) || !Number.isInteger(bodyB)) return "unknown bodies";
  const nameA = bodyNames[bodyA] || `body ${bodyA}`;
  const nameB = bodyNames[bodyB] || `body ${bodyB}`;
  return `${nameA} ↔ ${nameB}`;
}

const authorityRowsBySignal = {
  hard_rows: "authority-hard",
  finite_support: "authority-support",
  self_collision_avoidance: "authority-self-collision",
  world_collision_avoidance: "authority-world-collision",
  world_scene_snapshot: "authority-world-scene",
  joint_position: "authority-joint",
  joint_stopping: "authority-joint-stopping",
  actuator_effort: "authority-actuator",
  robot_observation_authority: "authority-robot-observation",
  command_tracking_authority: "authority-command-tracking",
  command_sampled_geometry: "authority-command-sampled",
  command_continuous_clearance: "authority-command-continuous",
  command_world_sampled_geometry: "authority-command-world-sampled",
  command_root_prediction: "authority-command-root-prediction",
  command_root_prediction_error: "authority-command-root-prediction-error",
  command_world_continuous_clearance: "authority-command-world-continuous",
  command_selection: "authority-command-selection",
  actuator_realization: "authority-realization",
  acceleration_realization: "authority-acceleration-realization",
  solver_budget: "authority-solver",
  thermal_reliability: "authority-thermal",
};

function applyAuthorityContract(contract) {
  authorityCapabilities = new Map(
    (contract?.signals || []).map((signal) => [signal.stable_id, signal]),
  );
  const source = document.querySelector("#authority-source");
  source.textContent = contract
    ? `Rust schema ${contract.schema} · ${contract.source}`
    : "Rust capability contract unavailable.";
  for (const [stableId, rowId] of Object.entries(authorityRowsBySignal)) {
    const capability = authorityCapabilities.get(stableId);
    if (!capability || capability.availability === "measured") continue;
    setLiveAuthorityRow(
      rowId,
      capability.availability.toUpperCase(),
      capability.reason,
      0,
      "unavailable",
    );
  }
}

function capabilityReason(stableId, fallback) {
  return authorityCapabilities.get(stableId)?.reason || fallback;
}

function updateAuthorityStack(metrics, activeTasks) {
  const dynamicAuthoritySample = metrics.authority_profile.startsWith("floating_dynamic_wbc");
  const hardParts = [
    ["dyn", metrics.dynamics_residual_linf],
    ["contact", metrics.contact_residual_linf],
    ["ineq", metrics.maximum_constraint_violation],
  ].filter((entry) => Number.isFinite(entry[1]));
  if (dynamicAuthoritySample && hardParts.length) {
    const hardResidual = Math.max(...hardParts.map((entry) => entry[1]));
    const thresholds = authorityThresholds.hard_residual;
    const pressure = upperPressure(hardResidual, thresholds);
    setLiveAuthorityRow(
      "authority-hard",
      hardResidual.toExponential(1),
      hardParts.map((entry) => `${entry[0]} ${entry[1].toExponential(1)}`).join(" · "),
      pressure,
      pressureState(pressure, hardResidual > thresholds.warning, hardResidual > thresholds.critical),
    );
  } else {
    setLiveAuthorityRow(
      "authority-hard",
      dynamicAuthoritySample ? "N/A" : "IDLE",
      dynamicAuthoritySample
        ? capabilityReason("hard_rows", "profile does not expose rigid-body rows")
        : "raw dynamic WBC not active · pull the torso to engage",
      0,
      "unavailable",
    );
  }

  if (Number.isFinite(metrics.minimum_support_margin_m)) {
    const margin = metrics.minimum_support_margin_m;
    const rawMargin = metrics.raw_minimum_support_margin_m;
    const thresholds = authorityThresholds.support_margin_m;
    const pressure = lowerPressure(margin, thresholds);
    setLiveAuthorityRow(
      "authority-support",
      `${(1000 * margin).toFixed(1)} mm`,
      Number.isInteger(metrics.limiting_support_patch)
        ? `patch ${metrics.limiting_support_patch} · raw→robust ${(1000 * rawMargin).toFixed(3)}→${(1000 * margin).toFixed(3)} mm · CoM reconstruction error erodes the declared polygon`
        : `raw→robust ${(1000 * rawMargin).toFixed(3)}→${(1000 * margin).toFixed(3)} mm · CoM reconstruction error erodes the declared polygon`,
      pressure,
      thresholds ? pressureState(pressure, margin < thresholds.warning, margin < thresholds.critical) : "unavailable",
    );
  } else {
    const capability = authorityCapabilities.get("finite_support");
    setLiveAuthorityRow(
      "authority-support",
      capability?.availability === "measured"
        ? (dynamicAuthoritySample ? "NO LOAD" : "IDLE")
        : capability?.availability?.toUpperCase() || "N/A",
      capability?.availability === "measured"
        ? (dynamicAuthoritySample
          ? "declared finite patch currently carries no positive normal load"
          : "raw dynamic WBC not active · support polygons are declared but unscored")
        : capabilityReason("finite_support", "rolling contact has no finite support patch"),
      0,
      "unavailable",
    );
  }

  const collisionThresholds = authorityThresholds.command_clearance_m;
  if (dynamicAuthoritySample && Number.isFinite(metrics.collision_barrier_minimum_distance_m)) {
    const distance = metrics.collision_barrier_minimum_distance_m;
    const margin = metrics.collision_barrier_minimum_margin_m;
    const rawMargin = metrics.raw_collision_barrier_minimum_margin_m;
    const pressure = lowerPressure(distance, collisionThresholds);
    const closest = Number.isInteger(metrics.collision_barrier_closest_pair)
      ? `closest ${metrics.collision_barrier_closest_pair} (${bodyPairLabel(metrics.collision_barrier_closest_body_a, metrics.collision_barrier_closest_body_b)})`
      : "closest pair unavailable";
    const active = metrics.collision_barrier_active_pairs || 0;
    const limiting = Number.isInteger(metrics.collision_barrier_limiting_pair)
      ? `limiting ${metrics.collision_barrier_limiting_pair} (${bodyPairLabel(metrics.collision_barrier_limiting_body_a, metrics.collision_barrier_limiting_body_b)})`
      : "no emitted barrier row";
    const acceleration = Number.isFinite(metrics.collision_barrier_residual_mps2)
      ? ` · required ${metrics.collision_barrier_required_acceleration_mps2.toFixed(2)} / achieved ${metrics.collision_barrier_achieved_acceleration_mps2.toFixed(2)} m/s² · residual ${metrics.collision_barrier_residual_mps2.toExponential(1)}`
      : "";
    const velocity = Number.isFinite(metrics.collision_barrier_relative_velocity_mps)
      ? ` · ḋ ${metrics.collision_barrier_relative_velocity_mps.toFixed(3)} m/s`
      : "";
    const unsupported = metrics.collision_barrier_unsupported_shapes > 0
      ? ` · ${metrics.collision_barrier_unsupported_shapes} unsupported shapes`
      : "";
    setLiveAuthorityRow(
      "authority-self-collision",
      `${(1000 * margin).toFixed(1)} mm`,
      `${closest} · raw→robust ${(1000 * rawMargin).toFixed(3)}→${(1000 * margin).toFixed(3)} mm · two point-error radii · ${active} active · ${limiting}${velocity}${acceleration} · ${metrics.collision_barrier_quality || "quality unavailable"}${unsupported}`,
      Math.max(pressure, unsupported ? 0.8 : 0),
      unsupported
        ? "warning"
        : pressureState(
          pressure,
          distance < collisionThresholds.warning,
          distance < collisionThresholds.critical,
        ),
    );
  } else {
    setLiveAuthorityRow(
      "authority-self-collision",
      dynamicAuthoritySample ? "N/A" : "IDLE",
      dynamicAuthoritySample
        ? capabilityReason("self_collision_avoidance", "floating collision barrier evidence unavailable")
        : "raw dynamic WBC not active · closest-feature barrier unscored",
      0,
      "unavailable",
    );
  }

  const worldThresholds = authorityThresholds.world_collision_margin_m
    || authorityThresholds.command_clearance_m;
  if (dynamicAuthoritySample && Number.isFinite(metrics.world_collision_minimum_distance_m)) {
    const distance = metrics.world_collision_minimum_distance_m;
    const margin = metrics.world_collision_minimum_margin_m;
    const rawMargin = metrics.raw_world_collision_minimum_margin_m;
    const pressure = lowerPressure(distance, worldThresholds);
    const closestBody = Number.isInteger(metrics.world_collision_closest_body)
      ? bodyNames[metrics.world_collision_closest_body] || `body ${metrics.world_collision_closest_body}`
      : "unknown body";
    const limitingBody = Number.isInteger(metrics.world_collision_limiting_body)
      ? bodyNames[metrics.world_collision_limiting_body] || `body ${metrics.world_collision_limiting_body}`
      : "unknown body";
    const closest = Number.isInteger(metrics.world_collision_closest_probe)
      ? `closest probe ${metrics.world_collision_closest_probe} (${closestBody})`
      : "closest probe unavailable";
    const active = metrics.world_collision_active_probes || 0;
    const limiting = Number.isInteger(metrics.world_collision_limiting_probe)
      ? `limiting ${metrics.world_collision_limiting_probe} (${limitingBody})`
      : "no emitted barrier row";
    const velocity = Number.isFinite(metrics.world_collision_relative_velocity_mps)
      ? ` · ḋ ${metrics.world_collision_relative_velocity_mps.toFixed(3)} m/s`
      : "";
    const acceleration = Number.isFinite(metrics.world_collision_residual_mps2)
      ? ` · required ${metrics.world_collision_required_acceleration_mps2.toFixed(2)} / achieved ${metrics.world_collision_achieved_acceleration_mps2.toFixed(2)} m/s² · residual ${metrics.world_collision_residual_mps2.toExponential(1)}`
      : "";
    const gradient = Number.isFinite(metrics.world_collision_gradient_norm)
      ? ` · |∇sdf| ${metrics.world_collision_gradient_norm.toFixed(3)}`
      : "";
    const unsupported = metrics.world_collision_unsupported_shapes > 0
      ? ` · ${metrics.world_collision_unsupported_shapes} unsupported shapes`
      : "";
    setLiveAuthorityRow(
      "authority-world-collision",
      `${(1000 * margin).toFixed(1)} mm`,
      `${closest} · raw→robust ${(1000 * rawMargin).toFixed(3)}→${(1000 * margin).toFixed(3)} mm · one point-error radius · ${active} active · ${limiting}${velocity}${acceleration}${gradient} · ${metrics.world_collision_field_source || "source unavailable"} / ${metrics.world_collision_proxy_quality || "proxy unavailable"} · outside ${metrics.world_collision_outside_policy || "unknown"}${unsupported}`,
      Math.max(pressure, unsupported ? 0.8 : 0),
      unsupported
        ? "warning"
        : pressureState(
          pressure,
          distance < worldThresholds.warning,
          distance < worldThresholds.critical,
        ),
    );
  } else {
    setLiveAuthorityRow(
      "authority-world-collision",
      dynamicAuthoritySample ? "N/A" : "IDLE",
      dynamicAuthoritySample
        ? capabilityReason("world_collision_avoidance", "world SDF evidence unavailable")
        : "raw dynamic WBC not active · body/SDF probes unscored",
      0,
      "unavailable",
    );
  }

  if (Number.isInteger(metrics.world_scene_epoch) && metrics.world_scene_validity) {
    const valid = metrics.world_scene_validity === "valid";
    const ageMs = Number.isFinite(metrics.world_scene_age_ns)
      ? metrics.world_scene_age_ns / 1e6
      : null;
    const horizonCovered = Number.isFinite(metrics.world_scene_valid_until_ns)
      && Number.isFinite(metrics.world_scene_horizon_end_ns)
      ? metrics.world_scene_valid_until_ns >= metrics.world_scene_horizon_end_ns
      : false;
    setLiveAuthorityRow(
      "authority-world-scene",
      valid ? `E${metrics.world_scene_epoch}` : metrics.world_scene_validity.toUpperCase(),
      `scene epoch ${metrics.world_scene_epoch} · source age ${ageMs === null ? "unavailable" : `${ageMs.toFixed(1)} ms`} · 20 ms horizon ${horizonCovered ? "covered" : "not covered"} · smooth control_world, not map/odom root`,
      valid ? 0.08 : 1,
      valid ? "ok" : "critical",
    );
  } else {
    setLiveAuthorityRow(
      "authority-world-scene",
      dynamicAuthoritySample ? "N/A" : "IDLE",
      dynamicAuthoritySample
        ? capabilityReason("world_scene_snapshot", "scene snapshot evidence unavailable")
        : "world scene contract not active · pull the torso to engage",
      0,
      "unavailable",
    );
  }

  if (Number.isFinite(metrics.minimum_joint_headroom_fraction)) {
    const thresholds = authorityThresholds.joint_margin_rad;
    const pressure = lowerPressure(metrics.minimum_joint_margin_rad, thresholds);
    const joint = coordinateNames[metrics.limiting_joint] || `coordinate ${metrics.limiting_joint}`;
    const degrees = metrics.minimum_joint_margin_rad * 180 / Math.PI;
    const rawDegrees = metrics.raw_minimum_joint_margin_rad * 180 / Math.PI;
    setLiveAuthorityRow(
      "authority-joint",
      `${degrees.toFixed(1)}°`,
      `${joint} · raw→robust ${rawDegrees.toFixed(3)}→${degrees.toFixed(3)}° · joint reconstruction radius`,
      pressure,
      pressureState(
        pressure,
        metrics.minimum_joint_margin_rad < thresholds.warning,
        metrics.minimum_joint_margin_rad < thresholds.critical,
      ),
    );
  } else {
    setLiveAuthorityRow(
      "authority-joint",
      "N/A",
      capabilityReason("joint_position", "no finite joint-position limit"),
      0,
      "unavailable",
    );
  }

  const stopping = authorityCapabilities.get("joint_stopping");
  if (Number.isFinite(metrics.minimum_joint_stopping_margin_rad_s2)) {
    const margin = metrics.minimum_joint_stopping_margin_rad_s2;
    const rawMargin = metrics.raw_minimum_joint_stopping_margin_rad_s2;
    const thresholds = authorityThresholds.joint_stopping_margin_rad_s2;
    const pressure = lowerPressure(margin, thresholds);
    const joint = coordinateNames[metrics.limiting_joint_stopping]
      || `coordinate ${metrics.limiting_joint_stopping}`;
    setLiveAuthorityRow(
      "authority-joint-stopping",
      `${margin.toFixed(2)} rad/s²`,
      `${joint} · raw→robust ${rawMargin.toFixed(3)}→${margin.toFixed(3)} rad/s² · all q/v error-box corners · ${metrics.joint_stopping_recovery_count || 0} recovering`,
      pressure,
      pressureState(pressure, margin < thresholds.warning, margin < thresholds.critical),
    );
  } else if (Number.isFinite(metrics.minimum_joint_velocity_stopping_headroom_rad)) {
    const margin = metrics.minimum_joint_velocity_stopping_headroom_rad;
    const thresholds = authorityThresholds.joint_margin_rad;
    const pressure = lowerPressure(margin, thresholds);
    const joint = coordinateNames[metrics.limiting_joint_velocity_stopping]
      || `coordinate ${metrics.limiting_joint_velocity_stopping}`;
    const degrees = margin * 180 / Math.PI;
    const scale = metrics.trajectory_velocity_scale;
    const steps = metrics.trajectory_backtrack_steps;
    setLiveAuthorityRow(
      "authority-joint-stopping",
      `${degrees.toFixed(2)}°`,
      `${joint} · velocity-stopping headroom ${degrees.toFixed(3)}° · trajectory scale ${Number.isFinite(scale) ? scale.toFixed(6) : "N/A"} · ${Number.isFinite(steps) ? steps : "N/A"} backtracks`,
      pressure,
      pressureState(pressure, margin < thresholds.warning, margin < thresholds.critical),
    );
  } else if (stopping?.availability !== "measured") {
    setLiveAuthorityRow(
      "authority-joint-stopping",
      stopping?.availability?.toUpperCase() || "UNAVAILABLE",
      capabilityReason("joint_stopping", "composed stopping envelope not streamed"),
      0,
      "unavailable",
    );
  } else {
    setLiveAuthorityRow(
      "authority-joint-stopping",
      dynamicAuthoritySample ? "N/A" : "IDLE",
      dynamicAuthoritySample
        ? capabilityReason("joint_stopping", "stopping envelope sample unavailable")
        : "raw dynamic WBC not active · stopping envelope unscored",
      0,
      "unavailable",
    );
  }

  if (Number.isFinite(metrics.maximum_torque_utilization)) {
    const utilization = metrics.maximum_torque_utilization;
    const thresholds = authorityThresholds.actuator_utilization;
    const pressure = upperPressure(utilization, thresholds);
    const actuator = actuatorNames[metrics.limiting_actuator]
      || `coordinate ${metrics.limiting_actuator}`;
    setLiveAuthorityRow(
      "authority-actuator",
      `${(100 * utilization).toFixed(1)}%`,
      `${actuator} · absolute solved torque / URDF effort limit`,
      pressure,
      pressureState(pressure, utilization > thresholds.warning, utilization > thresholds.critical),
    );
  } else {
    setLiveAuthorityRow(
      "authority-actuator",
      dynamicAuthoritySample ? "N/A" : "IDLE",
      dynamicAuthoritySample
        ? capabilityReason("actuator_effort", "profile does not expose actuator torque")
        : "raw dynamic WBC not active · no solved effort sample",
      0,
      "unavailable",
    );
  }

  if (metrics.robot_observation_reconstruction_provenance
      && Number.isFinite(metrics.robot_observation_reconstruction_source_age_ns)
      && Number.isFinite(metrics.robot_observation_reconstruction_source_age_headroom_ns)
      && Number.isFinite(metrics.robot_observation_reconstruction_synchronization_headroom_ns)) {
    const sourceAge = metrics.robot_observation_reconstruction_source_age_ns;
    const sourceAgeLimit = sourceAge
      + metrics.robot_observation_reconstruction_source_age_headroom_ns;
    const syncHeadroom = metrics.robot_observation_reconstruction_synchronization_headroom_ns;
    const hardEligible = metrics.robot_observation_reconstruction_hard_eligible === true;
    const rejected = metrics.robot_observation_ingest_rejected || 0;
    const pressure = Math.max(
      sourceAgeLimit > 0 ? Math.max(0, sourceAge) / sourceAgeLimit : Number(sourceAge > 0),
      syncHeadroom < 0 ? 1 : 0,
      rejected > 0 ? 1 : 0,
    );
    setLiveAuthorityRow(
      "authority-robot-history",
      metrics.robot_observation_reconstruction_provenance.toUpperCase(),
      `${(metrics.robot_observation_transport_mode || "exact").toUpperCase()} transport · frame E/I/P/H ${metrics.robot_observation_frame_exact_queries || 0}/${metrics.robot_observation_frame_interpolated_queries || 0}/${metrics.robot_observation_frame_predicted_queries || 0}/${metrics.robot_observation_frame_held_queries || 0} · error exposure ${(metrics.robot_observation_error_exposure_ns / 1e6).toFixed(3)} ms · q ${(metrics.robot_observation_joint_position_error_rad * 1e3).toFixed(3)} mrad / v ${metrics.robot_observation_joint_velocity_error_rad_s.toFixed(3)} rad/s · point ${(metrics.robot_observation_point_position_error_m * 1e3).toFixed(3)} mm · CoM ${(metrics.robot_observation_center_of_mass_position_error_m * 1e3).toFixed(3)} mm · root ${(metrics.robot_observation_root_translation_error_m * 1e3).toFixed(3)} mm / ${(metrics.robot_observation_root_rotation_error_rad * 1e3).toFixed(3)} mrad · ring ${metrics.robot_observation_history_len}/${metrics.robot_observation_history_capacity} · interval ${metrics.robot_observation_reconstruction_lower_time_ns}→${metrics.robot_observation_reconstruction_upper_time_ns} ns · sources 0x${Number(metrics.robot_observation_reconstruction_lower_source_id).toString(16)}→0x${Number(metrics.robot_observation_reconstruction_upper_source_id).toString(16)} · sequences ${metrics.robot_observation_reconstruction_lower_sequence}→${metrics.robot_observation_reconstruction_upper_sequence} · source age ${(sourceAge / 1e6).toFixed(3)} ms · age headroom ${(metrics.robot_observation_reconstruction_source_age_headroom_ns / 1e6).toFixed(3)} ms · sync headroom ${(syncHeadroom / 1e6).toFixed(3)} ms · ingest accepted/ignored/rejected ${metrics.robot_observation_ingest_accepted}/${metrics.robot_observation_ingest_ignored}/${rejected} · hard eligible ${hardEligible} · sole live floating-WBC state boundary`,
      Math.min(1, pressure),
      hardEligible && rejected === 0 ? pressureState(pressure, pressure >= 0.8, false) : "critical",
    );
  } else {
    setLiveAuthorityRow(
      "authority-robot-history",
      dynamicAuthoritySample ? "N/A" : "IDLE",
      dynamicAuthoritySample
        ? capabilityReason("robot_observation_history", "canonical reconstruction evidence unavailable")
        : "dynamic WBC not active · no canonical state reconstruction",
      0,
      "unavailable",
    );
  }

  if (Number.isFinite(metrics.robot_observation_age_ns)
      && Number.isFinite(metrics.robot_observation_age_headroom_ns)
      && Number.isFinite(metrics.robot_observation_synchronization_uncertainty_ns)
      && Number.isFinite(metrics.robot_observation_synchronization_headroom_ns)) {
    const age = metrics.robot_observation_age_ns;
    const ageLimit = age + metrics.robot_observation_age_headroom_ns;
    const sync = metrics.robot_observation_synchronization_uncertainty_ns;
    const syncLimit = sync + metrics.robot_observation_synchronization_headroom_ns;
    const valid = metrics.robot_observation_causal
      && metrics.robot_observation_age_valid
      && metrics.robot_observation_synchronization_valid;
    const pressure = Math.max(
      ageLimit > 0 ? Math.max(0, age) / ageLimit : Number(!metrics.robot_observation_age_valid),
      syncLimit > 0 ? Math.max(0, sync) / syncLimit : Number(!metrics.robot_observation_synchronization_valid),
    );
    const state = !valid
      ? "critical"
      : pressure >= 0.8 ? "warning" : "ok";
    setLiveAuthorityRow(
      "authority-robot-observation",
      valid ? "ADMITTED" : "REJECTED",
      `source 0x${Number(metrics.robot_observation_source_id).toString(16)} #${metrics.robot_observation_source_sequence} · source ${metrics.robot_observation_source_time_ns} ns · mapped ${metrics.robot_observation_mapped_time_ns} ns · age ${(age / 1e6).toFixed(3)} / ${(ageLimit / 1e6).toFixed(3)} ms · sync ±${(sync / 1e6).toFixed(3)} / ${(syncLimit / 1e6).toFixed(3)} ms · causal/age/sync ${metrics.robot_observation_causal}/${metrics.robot_observation_age_valid}/${metrics.robot_observation_synchronization_valid} · caller maps clocks; Rust reads none`,
      Math.min(1, pressure),
      state,
    );
  } else {
    setLiveAuthorityRow(
      "authority-robot-observation",
      dynamicAuthoritySample ? "N/A" : "IDLE",
      dynamicAuthoritySample
        ? capabilityReason("robot_observation_authority", "observation timing evidence unavailable")
        : "dynamic command query not active · no stamped observation",
      0,
      "unavailable",
    );
  }

  if (Number.isFinite(metrics.command_tracking_position_error)
      && Number.isFinite(metrics.command_tracking_velocity_error)
      && metrics.command_tracking_action) {
    const positionError = metrics.command_tracking_position_error;
    const velocityError = metrics.command_tracking_velocity_error;
    const positionContingency = positionError
      + metrics.command_tracking_position_contingency_headroom;
    const positionReject = positionError + metrics.command_tracking_position_reject_headroom;
    const velocityContingency = velocityError
      + metrics.command_tracking_velocity_contingency_headroom;
    const velocityReject = velocityError + metrics.command_tracking_velocity_reject_headroom;
    const pressure = Math.max(
      positionReject > 0 ? positionError / positionReject : 1,
      velocityReject > 0 ? velocityError / velocityReject : 1,
    );
    const positionActuator = actuatorNames[metrics.command_tracking_limiting_position_actuator]
      || `actuator ${metrics.command_tracking_limiting_position_actuator}`;
    const velocityActuator = actuatorNames[metrics.command_tracking_limiting_velocity_actuator]
      || `actuator ${metrics.command_tracking_limiting_velocity_actuator}`;
    const action = metrics.command_tracking_action;
    setLiveAuthorityRow(
      "authority-command-tracking",
      action.toUpperCase(),
      `|Δq| ${positionError.toFixed(3)} at ${positionActuator} · |Δv| ${velocityError.toFixed(3)} at ${velocityActuator} · brake ${positionContingency.toFixed(3)} / ${velocityContingency.toFixed(3)} · reject ${positionReject.toFixed(3)} / ${velocityReject.toFixed(3)} · direct mismatch evidence, not a plant model`,
      Math.min(1, pressure),
      action === "rejected" ? "critical" : action === "contingency" ? "warning" : "ok",
    );
  } else {
    setLiveAuthorityRow(
      "authority-command-tracking",
      dynamicAuthoritySample ? "N/A" : "IDLE",
      dynamicAuthoritySample
        ? capabilityReason("command_tracking_authority", "command tracking evidence unavailable")
        : "dynamic command query not active · pull the torso to engage",
      0,
      "unavailable",
    );
  }

  const clearanceThresholds = authorityThresholds.command_clearance_m;
  if (Number.isFinite(metrics.primary_sampled_clearance_m)) {
    const rawClearance = metrics.primary_sampled_clearance_m;
    const clearance = Number.isFinite(metrics.primary_robust_sampled_clearance_m)
      ? metrics.primary_robust_sampled_clearance_m
      : rawClearance;
    const pressure = lowerPressure(clearance, clearanceThresholds);
    const collisionBodies = Number.isInteger(metrics.primary_first_collision_body_a)
      && Number.isInteger(metrics.primary_first_collision_body_b)
      ? ` (${bodyNames[metrics.primary_first_collision_body_a] || `body ${metrics.primary_first_collision_body_a}`} ↔ ${bodyNames[metrics.primary_first_collision_body_b] || `body ${metrics.primary_first_collision_body_b}`})`
      : "";
    const minimumCollisionBodies = Number.isInteger(metrics.primary_minimum_collision_body_a)
      && Number.isInteger(metrics.primary_minimum_collision_body_b)
      ? `${bodyNames[metrics.primary_minimum_collision_body_a] || `body ${metrics.primary_minimum_collision_body_a}`} ↔ ${bodyNames[metrics.primary_minimum_collision_body_b] || `body ${metrics.primary_minimum_collision_body_b}`}`
      : "unknown bodies";
    const minimumPair = Number.isInteger(metrics.primary_minimum_collision_pair)
      ? `minimum pair ${metrics.primary_minimum_collision_pair} (${minimumCollisionBodies})`
      : "no represented pairs";
    const violation = Number.isInteger(metrics.primary_first_collision_pair)
      ? `first pair ${metrics.primary_first_collision_pair}${collisionBodies} at ${(metrics.primary_first_collision_time_ns / 1e6).toFixed(1)} ms`
      : "all 21 × 1 ms samples clear";
    setLiveAuthorityRow(
      "authority-command-sampled",
      `${(1000 * clearance).toFixed(1)} mm`,
      `${minimumPair} · ${violation} · raw→robust ${(1000 * rawClearance).toFixed(3)}→${(1000 * clearance).toFixed(3)} mm (2 × point radius) · requirement ${(1000 * metrics.command_clearance_requirement_m).toFixed(1)} mm`,
      pressure,
      pressureState(
        pressure,
        clearance < clearanceThresholds.warning,
        clearance < clearanceThresholds.critical,
      ),
    );
  } else {
    setLiveAuthorityRow(
      "authority-command-sampled",
      dynamicAuthoritySample ? "N/A" : "IDLE",
      dynamicAuthoritySample
        ? capabilityReason("command_sampled_geometry", "sampled command geometry unavailable")
        : "dynamic command query not active · pull the torso to engage",
      0,
      "unavailable",
    );
  }

  if (Number.isFinite(metrics.primary_continuous_clearance_m)) {
    const rawClearance = metrics.primary_continuous_clearance_m;
    const clearance = Number.isFinite(metrics.primary_robust_continuous_clearance_m)
      ? metrics.primary_robust_continuous_clearance_m
      : rawClearance;
    const pressure = lowerPressure(clearance, clearanceThresholds);
    const limitingBodies = bodyPairLabel(
      metrics.primary_continuous_limiting_body_a,
      metrics.primary_continuous_limiting_body_b,
    );
    const limitingPair = Number.isInteger(metrics.primary_continuous_limiting_pair)
      ? `pair ${metrics.primary_continuous_limiting_pair} (${limitingBodies})`
      : "limiting pair unavailable";
    const relativeSpeed = Number.isFinite(metrics.primary_continuous_relative_speed_m_s)
      ? ` · ≤ ${metrics.primary_continuous_relative_speed_m_s.toFixed(3)} m/s relative speed`
      : "";
    const refinement = Number.isInteger(metrics.primary_refinement_pair_samples)
      ? ` · ${metrics.primary_refinement_pair_samples} midpoint pair queries · ${metrics.primary_continuity_unresolved_intervals}/${metrics.primary_continuity_leaf_intervals} unresolved leaves · depth ${metrics.primary_continuity_maximum_subdivision_depth}`
      : "";
    setLiveAuthorityRow(
      "authority-command-continuous",
      `${(1000 * clearance).toFixed(1)} mm`,
      `${limitingPair}${relativeSpeed}${refinement} · raw→robust ${(1000 * rawClearance).toFixed(3)}→${(1000 * clearance).toFixed(3)} mm (2 × point radius) · requirement ${(1000 * metrics.command_clearance_requirement_m).toFixed(1)} mm`,
      pressure,
      pressureState(
        pressure,
        clearance < clearanceThresholds.warning,
        clearance < clearanceThresholds.critical,
      ),
    );
  } else {
    setLiveAuthorityRow(
      "authority-command-continuous",
      dynamicAuthoritySample ? "N/A" : "IDLE",
      dynamicAuthoritySample
        ? capabilityReason("command_continuous_clearance", "continuous command clearance unavailable")
        : "between-sample certificate not active · pull the torso to engage",
      0,
      "unavailable",
    );
  }

  if (Number.isFinite(metrics.primary_world_sampled_clearance_m)) {
    const rootRobustClearance = metrics.primary_world_sampled_clearance_m;
    const clearance = Number.isFinite(metrics.primary_world_robust_sampled_clearance_m)
      ? metrics.primary_world_robust_sampled_clearance_m
      : rootRobustClearance;
    const pressure = lowerPressure(clearance, clearanceThresholds);
    const primaryBody = Number.isInteger(metrics.primary_world_minimum_body)
      ? bodyNames[metrics.primary_world_minimum_body] || `body ${metrics.primary_world_minimum_body}`
      : "unknown body";
    const primaryProbe = Number.isInteger(metrics.primary_world_minimum_probe)
      ? `probe ${metrics.primary_world_minimum_probe} (${primaryBody})`
      : "minimum probe unavailable";
    const primaryUnknown = Number.isInteger(metrics.primary_world_first_unknown_probe)
      ? `UNKNOWN probe ${metrics.primary_world_first_unknown_probe} at ${(metrics.primary_world_first_unknown_time_ns / 1e6).toFixed(2)} ms`
      : "";
    const primaryViolation = Number.isInteger(metrics.primary_world_first_violation_probe)
      ? `COLLISION probe ${metrics.primary_world_first_violation_probe} at ${(metrics.primary_world_first_violation_time_ns / 1e6).toFixed(2)} ms`
      : "";
    const primaryWitness = primaryUnknown || primaryViolation || "all 21 × 1 ms samples known-clear";
    const brake = Number.isFinite(metrics.contingency_world_sampled_clearance_m)
      ? ` · brake ${(1000 * (Number.isFinite(metrics.contingency_world_robust_sampled_clearance_m) ? metrics.contingency_world_robust_sampled_clearance_m : metrics.contingency_world_sampled_clearance_m)).toFixed(1)} mm`
      : "";
    const fault = Boolean(primaryUnknown || primaryViolation);
    setLiveAuthorityRow(
      "authority-command-world-sampled",
      primaryUnknown ? "UNKNOWN" : `${(1000 * clearance).toFixed(1)} mm`,
      `${primaryProbe} · ${primaryWitness} · root-forecast→full robust ${(1000 * rootRobustClearance).toFixed(3)}→${(1000 * clearance).toFixed(3)} mm (field Lipschitz × point radius) · ${metrics.primary_world_field_source || "source unavailable"}${brake}`,
      fault ? 1 : pressure,
      fault
        ? "critical"
        : pressureState(
          pressure,
          clearance < clearanceThresholds.warning,
          clearance < clearanceThresholds.critical,
        ),
    );
  } else {
    setLiveAuthorityRow(
      "authority-command-world-sampled",
      dynamicAuthoritySample ? "N/A" : "IDLE",
      dynamicAuthoritySample
        ? capabilityReason("command_world_sampled_geometry", "sampled world-command geometry unavailable")
        : "dynamic world-command query not active · pull the torso to engage",
      0,
      "unavailable",
    );
  }

  if (Number.isFinite(metrics.primary_root_prediction_translation_m)) {
    const primaryTravel = 1000 * metrics.primary_root_prediction_translation_m;
    const primaryRotation = metrics.primary_root_prediction_rotation_rad * 180 / Math.PI;
    const brakeTravel = Number.isFinite(metrics.contingency_root_prediction_translation_m)
      ? ` · brake ${(1000 * metrics.contingency_root_prediction_translation_m).toFixed(2)} mm / ${(metrics.contingency_root_prediction_rotation_rad * 180 / Math.PI).toFixed(3)}°`
      : "";
    const speed = Number.isFinite(metrics.primary_root_prediction_max_linear_speed_m_s)
      ? ` · |v| ≤ ${metrics.primary_root_prediction_max_linear_speed_m_s.toFixed(3)} m/s · |ω| ≤ ${metrics.primary_root_prediction_max_angular_speed_rad_s.toFixed(3)} rad/s`
      : "";
    setLiveAuthorityRow(
      "authority-command-root-prediction",
      `${primaryTravel.toFixed(2)} mm`,
      `primary ${primaryTravel.toFixed(2)} mm / ${primaryRotation.toFixed(3)}°${speed}${brakeTravel} · smooth control_world · prediction witness, not root actuation`,
      Math.min(1, primaryTravel / 50),
      "ok",
    );
  } else {
    setLiveAuthorityRow(
      "authority-command-root-prediction",
      dynamicAuthoritySample ? "N/A" : "IDLE",
      dynamicAuthoritySample
        ? capabilityReason("command_root_prediction", "floating-root prediction unavailable")
        : "floating-root command prediction not active · pull the torso to engage",
      0,
      "unavailable",
    );
  }

  if (Number.isFinite(metrics.primary_root_prediction_translation_error_radius_m)) {
    const translationErrorMm = 1000 * metrics.primary_root_prediction_translation_error_radius_m;
    const rotationErrorDeg = metrics.primary_root_prediction_rotation_error_radius_rad * 180 / Math.PI;
    const primaryErosionMm = 1000 * metrics.primary_world_prediction_clearance_erosion_m;
    const brakeErosion = Number.isFinite(metrics.contingency_world_prediction_clearance_erosion_m)
      ? ` · brake erosion ${(1000 * metrics.contingency_world_prediction_clearance_erosion_m).toFixed(2)} mm`
      : "";
    const pointErosion = Number.isFinite(metrics.robot_observation_point_position_error_m)
      ? ` · local point radius ${(1000 * metrics.robot_observation_point_position_error_m).toFixed(3)} mm applied separately`
      : "";
    setLiveAuthorityRow(
      "authority-command-root-prediction-error",
      `${primaryErosionMm.toFixed(2)} mm`,
      `horizon radius ${translationErrorMm.toFixed(2)} mm / ${rotationErrorDeg.toFixed(3)}° · primary SDF root-forecast erosion ${primaryErosionMm.toFixed(2)} mm${brakeErosion}${pointErosion} · deterministic bound, not probability`,
      Math.min(1, primaryErosionMm / 5),
      primaryErosionMm >= 2 ? "warning" : "ok",
    );
  } else {
    setLiveAuthorityRow(
      "authority-command-root-prediction-error",
      dynamicAuthoritySample ? "N/A" : "IDLE",
      dynamicAuthoritySample
        ? capabilityReason("command_root_prediction_error", "root prediction error bound unavailable")
        : "root prediction error growth not active · pull the torso to engage",
      0,
      "unavailable",
    );
  }

  if (Number.isFinite(metrics.primary_world_continuous_clearance_m)) {
    const rootRobustClearance = metrics.primary_world_continuous_clearance_m;
    const clearance = Number.isFinite(metrics.primary_world_robust_continuous_clearance_m)
      ? metrics.primary_world_robust_continuous_clearance_m
      : rootRobustClearance;
    const pressure = lowerPressure(clearance, clearanceThresholds);
    const limitingBody = Number.isInteger(metrics.primary_world_continuous_limiting_body)
      ? bodyNames[metrics.primary_world_continuous_limiting_body] || `body ${metrics.primary_world_continuous_limiting_body}`
      : "unknown body";
    const limitingProbe = Number.isInteger(metrics.primary_world_continuous_limiting_probe)
      ? `probe ${metrics.primary_world_continuous_limiting_probe} (${limitingBody})`
      : "limiting probe unavailable";
    const rate = Number.isFinite(metrics.primary_world_distance_rate_bound_m_s)
      ? ` · ḋ ≤ ${metrics.primary_world_distance_rate_bound_m_s.toFixed(3)} m/s`
      : "";
    const work = ` · ${metrics.primary_world_refinement_probe_samples || 0} midpoint probes · ${metrics.primary_world_unresolved_intervals || 0}/${metrics.primary_world_leaf_intervals || 0} unresolved leaves · depth ${metrics.primary_world_maximum_subdivision_depth || 0}`;
    const brake = Number.isFinite(metrics.contingency_world_continuous_clearance_m)
      ? ` · brake certificate ${(1000 * (Number.isFinite(metrics.contingency_world_robust_continuous_clearance_m) ? metrics.contingency_world_robust_continuous_clearance_m : metrics.contingency_world_continuous_clearance_m)).toFixed(1)} mm`
      : "";
    setLiveAuthorityRow(
      "authority-command-world-continuous",
      `${(1000 * clearance).toFixed(1)} mm`,
      `${limitingProbe}${rate}${work}${brake} · root-forecast→full robust ${(1000 * rootRobustClearance).toFixed(3)}→${(1000 * clearance).toFixed(3)} mm · requirement ${(1000 * metrics.command_clearance_requirement_m).toFixed(1)} mm`,
      pressure,
      pressureState(
        pressure,
        clearance < clearanceThresholds.warning,
        clearance < clearanceThresholds.critical,
      ),
    );
  } else {
    setLiveAuthorityRow(
      "authority-command-world-continuous",
      dynamicAuthoritySample ? "N/A" : "IDLE",
      dynamicAuthoritySample
        ? capabilityReason("command_world_continuous_clearance", "continuous world-command certificate unavailable")
        : "bounded world-clearance certificate not active · pull the torso to engage",
      0,
      "unavailable",
    );
  }

  if (metrics.command_selection) {
    const selection = metrics.command_selection;
    const flags = Number.isInteger(metrics.command_admission_flags)
      ? `0x${metrics.command_admission_flags.toString(16).padStart(3, "0")}`
      : "flags unavailable";
    const state = selection === "primary" ? "ok" : selection === "contingency" ? "warning" : "critical";
    const pressure = selection === "primary" ? 0 : selection === "contingency" ? 0.8 : 1;
    const brake = Number.isFinite(metrics.contingency_continuous_clearance_m)
      ? ` · brake certificate ${(1000 * (Number.isFinite(metrics.contingency_robust_continuous_clearance_m) ? metrics.contingency_robust_continuous_clearance_m : metrics.contingency_continuous_clearance_m)).toFixed(1)} mm`
      : "";
    const timing = Number.isFinite(metrics.command_admission_us)
      ? ` · command query ${(metrics.command_admission_us / 1000).toFixed(2)} ms max / ${(metrics.command_admission_batch_us / 1000).toFixed(2)} ms batch`
      : "";
    setLiveAuthorityRow(
      "authority-command-selection",
      selection.toUpperCase(),
      `${flags} · policy-/physics-free command query${brake}${timing}`,
      pressure,
      state,
    );
  } else {
    setLiveAuthorityRow(
      "authority-command-selection",
      "IDLE",
      "no command selection query · browser pose is not an actuator realization",
      0,
      "unavailable",
    );
  }

  const solverThresholds = authorityThresholds.solver_wall_time_us;
  const solverPressure = upperPressure(metrics.solve_us, solverThresholds);
  setLiveAuthorityRow(
    "authority-solver",
    `${(metrics.solve_us / 1000).toFixed(2)} / ${(solverThresholds.critical / 1000).toFixed(0)} ms`,
    `${dynamicAuthoritySample ? "raw WBC" : "preview"} max query ${metrics.solve_us.toFixed(0)} µs · batch ${(metrics.query_batch_us ?? metrics.solve_us).toFixed(0)} µs / ${metrics.query_count ?? 1} queries · ${metrics.task_pseudoinverse_calls || 0} pinv · ${metrics.task_jacobi_sweeps || 0} Jacobi · ${metrics.clipped_steps || 0} clips`,
    solverPressure,
    pressureState(
      solverPressure,
      metrics.solve_us > solverThresholds.warning,
      metrics.solve_us > solverThresholds.critical,
    ),
  );

  for (const priority of priorityNames) {
    const tasks = activeTasks.filter((task) => task.priority === priority);
    const row = liveTaskLevelElements.get(priority);
    if (!dynamicAuthoritySample) {
      row.classList.remove("clipped");
      row.querySelector("span").textContent = "no raw sample";
      row.querySelector("output").textContent = "IDLE";
      continue;
    }
    const clipped = tasks.some((task) => task.clipped);
    const maximum = tasks.reduce((largest, task) => Math.max(largest, task.rms), 0);
    row.classList.toggle("clipped", clipped);
    row.querySelector("span").textContent = tasks.length ? `${tasks.length} active` : "inactive";
    row.querySelector("output").textContent = tasks.length
      ? `${maximum.toExponential(1)}${clipped ? " · CLIPPED" : ""}`
      : "—";
  }
}

function initializeLiveTaskLevels() {
  const container = document.querySelector("#live-task-levels");
  for (const priority of priorityNames) {
    const row = element("div", "live-task-level");
    row.append(
      element("strong", null, priority),
      element("span", null, "inactive"),
      element("output", null, "—"),
    );
    liveTaskLevelElements.set(priority, row);
    container.append(row);
  }
}

function drawSparkline() {
  const ratio = devicePixelRatio || 1;
  const width = sparkline.clientWidth;
  const height = sparkline.clientHeight;
  const pixelWidth = Math.max(1, Math.round(width * ratio));
  const pixelHeight = Math.max(1, Math.round(height * ratio));
  if (sparkline.width !== pixelWidth || sparkline.height !== pixelHeight) {
    sparkline.width = pixelWidth;
    sparkline.height = pixelHeight;
  }
  spark.setTransform(ratio, 0, 0, ratio, 0, 0);
  spark.clearRect(0, 0, width, height);
  if (solveHistory.length < 2) return;
  const max = Math.max(...solveHistory, 1);
  spark.strokeStyle = "#77d4ae";
  spark.lineWidth = 1.5;
  spark.beginPath();
  solveHistory.forEach((value, index) => {
    const x = (index / (solveHistory.length - 1)) * width;
    const y = height - 4 - (value / max) * (height - 10);
    if (index === 0) spark.moveTo(x, y); else spark.lineTo(x, y);
  });
  spark.stroke();
}

function drawAuthorityHistory() {
  const ratio = devicePixelRatio || 1;
  const width = authorityHistoryCanvas.clientWidth;
  const height = authorityHistoryCanvas.clientHeight;
  const pixelWidth = Math.max(1, Math.round(width * ratio));
  const pixelHeight = Math.max(1, Math.round(height * ratio));
  if (
    authorityHistoryCanvas.width !== pixelWidth
    || authorityHistoryCanvas.height !== pixelHeight
  ) {
    authorityHistoryCanvas.width = pixelWidth;
    authorityHistoryCanvas.height = pixelHeight;
  }
  authorityHistoryContext.setTransform(ratio, 0, 0, ratio, 0, 0);
  authorityHistoryContext.clearRect(0, 0, width, height);
  const summary = document.querySelector("#authority-history-summary");
  if (!commandAuthorityHistory.length || width <= 0 || height <= 0) {
    summary.textContent = "awaiting command query";
    return;
  }

  const values = commandAuthorityHistory.flatMap((sample) => [
    sample.sampled,
    sample.continuous,
    sample.required,
  ]).filter(Number.isFinite);
  const lower = Math.min(...values, 0);
  const upper = Math.max(...values, 0.001);
  const padding = Math.max(0.002, 0.12 * (upper - lower));
  const minimum = lower - padding;
  const maximum = upper + padding;
  const xFor = (index) => commandAuthorityHistory.length === 1
    ? width - 1
    : (index / (commandAuthorityHistory.length - 1)) * (width - 1);
  const yFor = (value) => height - 5
    - ((value - minimum) / Math.max(maximum - minimum, Number.EPSILON)) * (height - 10);

  const columnWidth = Math.max(1, width / Math.max(commandAuthorityHistory.length, 1));
  commandAuthorityHistory.forEach((sample, index) => {
    if (sample.selection === "primary") return;
    authorityHistoryContext.fillStyle = sample.selection === "contingency"
      ? "rgba(131, 76, 41, .34)"
      : "rgba(150, 55, 49, .42)";
    authorityHistoryContext.fillRect(xFor(index) - columnWidth / 2, 0, columnWidth + 1, height);
  });

  const drawSeries = (key, color, widthPx, dash = []) => {
    authorityHistoryContext.strokeStyle = color;
    authorityHistoryContext.lineWidth = widthPx;
    authorityHistoryContext.setLineDash(dash);
    authorityHistoryContext.beginPath();
    let drawing = false;
    commandAuthorityHistory.forEach((sample, index) => {
      if (!Number.isFinite(sample[key])) {
        drawing = false;
        return;
      }
      const x = xFor(index);
      const y = yFor(sample[key]);
      if (drawing) authorityHistoryContext.lineTo(x, y);
      else authorityHistoryContext.moveTo(x, y);
      drawing = true;
    });
    authorityHistoryContext.stroke();
  };
  drawSeries("required", "#d7aa64", 1, [3, 3]);
  drawSeries("sampled", "#7aa9d6", 1.2);
  drawSeries("continuous", "#77d4ae", 1.7);
  authorityHistoryContext.setLineDash([]);

  const contingency = commandAuthorityHistory.filter((sample) => sample.selection === "contingency").length;
  const rejected = commandAuthorityHistory.filter((sample) => sample.selection === "rejected").length;
  const latest = commandAuthorityHistory[commandAuthorityHistory.length - 1];
  summary.textContent = `${commandAuthorityHistory.length} ticks · ${contingency} fallback · ${rejected} rejected · ${latest.refinements ?? 0} refine / ${latest.unresolved ?? 0} unresolved`;
}

function reset() {
  selected = null;
  drag = null;
  orbitDrag = null;
  pendingDragCommand = null;
  pushDrag = null;
  pendingPushCommand = null;
  activeForceArrow = null;
  commandAuthorityHistory = [];
  drawAuthorityHistory();
  document.querySelector("#selection-empty").classList.remove("hidden");
  document.querySelector("#selection-detail").classList.add("hidden");
  updateObservationTransport({ robot_observation_transport_mode: "exact" });
  // The measured ghost is live even in TARGET mode, so the reset button
  // always resets both state owners when the plant socket is connected.
  // TARGET keeps its independent Rust preview reset as well; PUSH is owned
  // entirely by the MuJoCo stream.
  if (plantConnected) sendPlant({ type: "plant_reset" });
  if (interactionMode !== "push") send({ type: "reset" });
  scheduleRender();
}

function pauseSimulation() {
  if (!plantConnected) return;
  plantPaused = true;
  sendPlant({ type: "plant_pause" });
  showToast("MuJoCo paused · measured state held");
  updateInteractionUi();
}

function resumeSimulation() {
  if (!plantConnected) return;
  plantPaused = false;
  sendPlant({ type: "plant_resume" });
  showToast("MuJoCo resumed · WBC tracking measured state");
  updateInteractionUi();
}

function showToast(message) {
  toast.textContent = message;
  toast.classList.add("visible");
  setTimeout(() => toast.classList.remove("visible"), 2500);
}

document.querySelector("#reset-button").addEventListener("click", reset);
pauseButton.addEventListener("click", pauseSimulation);
resumeButton.addEventListener("click", resumeSimulation);
[targetTool, pushTool].forEach((button) => {
  button.addEventListener("click", () => setInteractionMode(button.dataset.interactionMode));
});
observationTransportButtons.forEach((button) => {
  button.addEventListener("click", () => {
    const mode = button.dataset.observationMode;
    updateObservationTransport({ robot_observation_transport_mode: mode });
    send({ type: "set_observation_transport", mode });
  });
});
document.querySelector("#geometry-toggle").addEventListener("click", (event) => {
  showGeometry = !showGeometry;
  event.currentTarget.setAttribute("aria-pressed", String(showGeometry));
  scheduleRender();
});
document.querySelector("#rig-toggle").addEventListener("click", (event) => {
  showRig = !showRig;
  event.currentTarget.setAttribute("aria-pressed", String(showRig));
  scheduleRender();
});
architectureButton.addEventListener("click", () => setArchitectureOpen(true));
architectureFooterButton.addEventListener("click", () => setArchitectureOpen(true));
architectureClose.addEventListener("click", (event) => {
  event.preventDefault();
  event.stopPropagation();
  setArchitectureOpen(false);
});
architectureScrim.addEventListener("click", (event) => {
  event.preventDefault();
  event.stopPropagation();
  setArchitectureOpen(false);
});
window.addEventListener("keydown", (event) => {
  if (robotControlsEnabled && event.key.toLowerCase() === "r") reset();
  if (event.key === "Escape" && !architectureReview.hidden) setArchitectureOpen(false);
});
window.addEventListener("resize", resize);
window.setInterval(() => {
  if (
    socket?.readyState === WebSocket.OPEN
    && lastSocketMessageAt > 0
    && performance.now() - lastSocketMessageAt > 1500
  ) {
    connection.classList.remove("online");
    connectionLabel.textContent = "Stream stalled";
    setRobotControlsEnabled(false);
    socket.close();
  }
}, 250);
updateCameraBasis();
initializeLiveTaskLevels();
setRobotControlsEnabled(false);
resize();
connect();
loadArchitecture();
setInterval(loadArchitecture, 30_000);

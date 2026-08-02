use std::{
    alloc::{GlobalAlloc, Layout, System},
    io::{self, Read},
    sync::atomic::{AtomicU64, Ordering},
    time::Instant,
};

use anyhow::{Context, Result, bail};
use bonesaw_core::{
    ConvexSupportPolygon, LipmBoundaryPlan, LipmBoundaryPlannerConfig, LipmSample, LipmState, Vec2,
    Vec3, VectorJet, sample_quintic_vector_jet,
};
use serde::{Deserialize, Serialize};

struct CountingAllocator;

static ALLOCATION_CALLS: AtomicU64 = AtomicU64::new(0);
static ALLOCATED_BYTES: AtomicU64 = AtomicU64::new(0);

unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        ALLOCATION_CALLS.fetch_add(1, Ordering::Relaxed);
        ALLOCATED_BYTES.fetch_add(layout.size() as u64, Ordering::Relaxed);
        unsafe { System.alloc(layout) }
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        ALLOCATION_CALLS.fetch_add(1, Ordering::Relaxed);
        ALLOCATED_BYTES.fetch_add(layout.size() as u64, Ordering::Relaxed);
        unsafe { System.alloc_zeroed(layout) }
    }

    unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
        unsafe { System.dealloc(pointer, layout) }
    }

    unsafe fn realloc(&self, pointer: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        ALLOCATION_CALLS.fetch_add(1, Ordering::Relaxed);
        ALLOCATED_BYTES.fetch_add(new_size as u64, Ordering::Relaxed);
        unsafe { System.realloc(pointer, layout, new_size) }
    }
}

#[global_allocator]
static GLOBAL_ALLOCATOR: CountingAllocator = CountingAllocator;

#[derive(Clone, Debug, Deserialize)]
struct Request {
    ticks: usize,
    dt_seconds: f64,
    liftoff_tick: usize,
    touchdown_tick: usize,
    swing_foot: usize,
    terminal_support_foot: usize,
    gravity_mps2: f64,
    com_height_m: f64,
    initial_com: [f64; 3],
    initial_com_velocity: [f64; 3],
    initial_root: [f64; 3],
    initial_feet: [[f64; 3]; 2],
    swing_touchdown: [f64; 3],
    swing_height_m: f64,
    contact_patch_center_x_m: f64,
    root_horizontal_follow_ratio: f64,
    maximum_root_to_landing_reach_m: f64,
    cop_startup_duration_seconds: f64,
    cop_transition_duration_seconds: f64,
    opening_support_ccw: Vec<[f64; 2]>,
    terminal_support_ccw: Vec<[f64; 2]>,
    minimum_switch_ratio: f64,
    maximum_switch_ratio: f64,
    switch_candidates: usize,
    minimum_cop_margin_m: f64,
}

#[derive(Clone, Debug, Serialize)]
struct RuntimeReport {
    plan_ns: u128,
    sample_ns: u128,
    plan_allocation_calls: u64,
    plan_allocated_bytes: u64,
    sample_allocation_calls: u64,
    sample_allocated_bytes: u64,
}

#[derive(Clone, Debug, Serialize)]
struct PlanReport {
    omega_rad_per_second: f64,
    duration_seconds: f64,
    switch_time_seconds: f64,
    switch_ratio: f64,
    cop_startup_duration_seconds: f64,
    cop_transition_start_seconds: f64,
    cop_transition_end_seconds: f64,
    cop_transition_duration_seconds: f64,
    first_cop_xy: [f64; 2],
    second_cop_xy: [f64; 2],
    terminal_cop_xy: [f64; 2],
    first_cop_margin_m: f64,
    second_cop_margin_m: f64,
    terminal_cop_margin_m: f64,
    authored_touchdown: [f64; 3],
    applied_touchdown: [f64; 3],
    authored_root_to_touchdown_reach_m: f64,
    applied_root_to_touchdown_reach_m: f64,
    root_horizontal_follow_ratio: f64,
    touchdown_was_retargeted: bool,
}

#[derive(Clone, Debug, Serialize)]
struct Response {
    schema: u32,
    implementation: &'static str,
    policy_or_physics_rollout: bool,
    ik_or_wbc_solve: bool,
    exact_repeat: bool,
    runtime: RuntimeReport,
    plan: PlanReport,
    root_targets: Vec<[f64; 3]>,
    root_target_velocities: Vec<[f64; 3]>,
    root_target_accelerations: Vec<[f64; 3]>,
    center_of_mass_targets: Vec<[f64; 3]>,
    center_of_mass_target_velocities: Vec<[f64; 3]>,
    center_of_mass_target_accelerations: Vec<[f64; 3]>,
    target_positions: Vec<[[f64; 3]; 2]>,
    target_velocities: Vec<[[f64; 3]; 2]>,
    target_accelerations: Vec<[[f64; 3]; 2]>,
    reference_stance: Vec<[u8; 2]>,
}

#[derive(Clone, Copy, Debug)]
struct FootSample {
    positions: [[f64; 3]; 2],
    velocities: [[f64; 3]; 2],
    accelerations: [[f64; 3]; 2],
    stance: [u8; 2],
}

fn main() -> Result<()> {
    let mut input = String::new();
    io::stdin()
        .read_to_string(&mut input)
        .context("failed to read reference request from stdin")?;
    let request: Request = serde_json::from_str(&input).context("invalid reference request")?;
    validate_request(&request)?;

    let opening_vertices = request
        .opening_support_ccw
        .iter()
        .map(|point| Vec2::new(point[0], point[1]))
        .collect::<Vec<_>>();
    let terminal_vertices = request
        .terminal_support_ccw
        .iter()
        .map(|point| Vec2::new(point[0], point[1]))
        .collect::<Vec<_>>();
    let opening = ConvexSupportPolygon::<16>::from_ccw_vertices(&opening_vertices)?;
    let terminal = ConvexSupportPolygon::<16>::from_ccw_vertices(&terminal_vertices)?;
    let support_foot = request.initial_feet[request.terminal_support_foot];
    let terminal_position = Vec2::new(
        support_foot[0] + request.contact_patch_center_x_m,
        support_foot[1],
    );
    let initial = LipmState {
        position: Vec2::new(request.initial_com[0], request.initial_com[1]),
        velocity: Vec2::new(
            request.initial_com_velocity[0],
            request.initial_com_velocity[1],
        ),
    };
    let terminal_state = LipmState {
        position: terminal_position,
        velocity: Vec2::zeros(),
    };
    let config = LipmBoundaryPlannerConfig {
        gravity_mps2: request.gravity_mps2,
        com_height_m: request.com_height_m,
        duration_seconds: request.liftoff_tick as f64 * request.dt_seconds,
        cop_startup_duration_seconds: request.cop_startup_duration_seconds,
        cop_transition_duration_seconds: request.cop_transition_duration_seconds,
        minimum_switch_ratio: request.minimum_switch_ratio,
        maximum_switch_ratio: request.maximum_switch_ratio,
        switch_candidates: request.switch_candidates,
        minimum_cop_margin_m: request.minimum_cop_margin_m,
    };

    let plan_allocations_before = ALLOCATION_CALLS.load(Ordering::Relaxed);
    let plan_bytes_before = ALLOCATED_BYTES.load(Ordering::Relaxed);
    let plan_started = Instant::now();
    let plan = LipmBoundaryPlan::plan(initial, terminal_state, &opening, &terminal, config)?;
    let plan_ns = plan_started.elapsed().as_nanos();
    let plan_allocation_calls = ALLOCATION_CALLS.load(Ordering::Relaxed) - plan_allocations_before;
    let plan_allocated_bytes = ALLOCATED_BYTES.load(Ordering::Relaxed) - plan_bytes_before;

    let terminal_com = Vec3::new(
        terminal_state.position.x,
        terminal_state.position.y,
        request.initial_com[2],
    );
    let terminal_root = root_position(&request, terminal_com);
    let (applied_touchdown, authored_reach, applied_reach) =
        reachable_touchdown(&request, terminal_root)?;
    let mut response = Response {
        schema: 1,
        implementation: "Bonesaw smooth-entry two-stage support-constrained LIPM",
        policy_or_physics_rollout: false,
        ik_or_wbc_solve: false,
        exact_repeat: false,
        runtime: RuntimeReport {
            plan_ns,
            sample_ns: 0,
            plan_allocation_calls,
            plan_allocated_bytes,
            sample_allocation_calls: 0,
            sample_allocated_bytes: 0,
        },
        plan: PlanReport {
            omega_rad_per_second: plan.omega_rad_per_second(),
            duration_seconds: plan.config.duration_seconds,
            switch_time_seconds: plan.switch_time_seconds,
            switch_ratio: plan.switch_time_seconds / plan.config.duration_seconds,
            cop_startup_duration_seconds: plan.config.cop_startup_duration_seconds,
            cop_transition_start_seconds: plan.cop_transition_start_seconds,
            cop_transition_end_seconds: plan.cop_transition_end_seconds,
            cop_transition_duration_seconds: plan.config.cop_transition_duration_seconds,
            first_cop_xy: vector2_array(plan.first_cop),
            second_cop_xy: vector2_array(plan.second_cop),
            terminal_cop_xy: vector2_array(plan.terminal_cop),
            first_cop_margin_m: plan.first_cop_margin_m,
            second_cop_margin_m: plan.second_cop_margin_m,
            terminal_cop_margin_m: plan.terminal_cop_margin_m,
            authored_touchdown: request.swing_touchdown,
            applied_touchdown,
            authored_root_to_touchdown_reach_m: authored_reach,
            applied_root_to_touchdown_reach_m: applied_reach,
            root_horizontal_follow_ratio: request.root_horizontal_follow_ratio,
            touchdown_was_retargeted: applied_touchdown != request.swing_touchdown,
        },
        root_targets: Vec::with_capacity(request.ticks),
        root_target_velocities: Vec::with_capacity(request.ticks),
        root_target_accelerations: Vec::with_capacity(request.ticks),
        center_of_mass_targets: Vec::with_capacity(request.ticks),
        center_of_mass_target_velocities: Vec::with_capacity(request.ticks),
        center_of_mass_target_accelerations: Vec::with_capacity(request.ticks),
        target_positions: Vec::with_capacity(request.ticks),
        target_velocities: Vec::with_capacity(request.ticks),
        target_accelerations: Vec::with_capacity(request.ticks),
        reference_stance: Vec::with_capacity(request.ticks),
    };
    let sample_allocations_before = ALLOCATION_CALLS.load(Ordering::Relaxed);
    let sample_bytes_before = ALLOCATED_BYTES.load(Ordering::Relaxed);
    let sample_started = Instant::now();
    for tick in 0..request.ticks {
        append_sample(&request, &plan, applied_touchdown, tick, &mut response)?;
    }
    response.runtime.sample_ns = sample_started.elapsed().as_nanos();
    response.runtime.sample_allocation_calls =
        ALLOCATION_CALLS.load(Ordering::Relaxed) - sample_allocations_before;
    response.runtime.sample_allocated_bytes =
        ALLOCATED_BYTES.load(Ordering::Relaxed) - sample_bytes_before;
    response.exact_repeat = exact_repeat(
        &request,
        &opening,
        &terminal,
        plan.config,
        applied_touchdown,
        &response,
    )?;
    serde_json::to_writer(io::stdout().lock(), &response)
        .context("failed to serialize reference")?;
    Ok(())
}

fn validate_request(request: &Request) -> Result<()> {
    if request.ticks == 0
        || request.dt_seconds <= 0.0
        || !request.dt_seconds.is_finite()
        || request.liftoff_tick == 0
        || request.touchdown_tick <= request.liftoff_tick
        || request.touchdown_tick >= request.ticks
        || request.swing_foot >= 2
        || request.terminal_support_foot >= 2
        || request.swing_foot == request.terminal_support_foot
        || !request.com_height_m.is_finite()
        || request.com_height_m <= 0.0
        || !request.swing_height_m.is_finite()
        || request.swing_height_m < 0.0
        || !request.contact_patch_center_x_m.is_finite()
        || !request.root_horizontal_follow_ratio.is_finite()
        || !(0.0..=1.0).contains(&request.root_horizontal_follow_ratio)
        || !request.maximum_root_to_landing_reach_m.is_finite()
        || request.maximum_root_to_landing_reach_m <= 0.0
        || !request.cop_transition_duration_seconds.is_finite()
        || request.cop_transition_duration_seconds <= 0.0
        || !request.cop_startup_duration_seconds.is_finite()
        || request.cop_startup_duration_seconds <= 0.0
    {
        bail!("invalid reference dimensions or timing");
    }
    if !request
        .initial_com
        .iter()
        .chain(&request.initial_com_velocity)
        .chain(&request.initial_root)
        .chain(request.initial_feet.iter().flatten())
        .chain(&request.swing_touchdown)
        .all(|value| value.is_finite())
    {
        bail!("reference boundary contains non-finite data");
    }
    Ok(())
}

fn append_sample(
    request: &Request,
    plan: &LipmBoundaryPlan,
    applied_touchdown: [f64; 3],
    tick: usize,
    response: &mut Response,
) -> Result<()> {
    let sample = plan.sample(tick as f64 * request.dt_seconds)?;
    let com = Vec3::new(
        sample.state.position.x,
        sample.state.position.y,
        request.initial_com[2],
    );
    let velocity = [sample.state.velocity.x, sample.state.velocity.y, 0.0];
    let acceleration = [sample.acceleration.x, sample.acceleration.y, 0.0];
    let feet = foot_sample(request, applied_touchdown, tick)?;
    response
        .root_targets
        .push(vector3_array(root_position(request, com)));
    response.root_target_velocities.push([
        request.root_horizontal_follow_ratio * sample.state.velocity.x,
        request.root_horizontal_follow_ratio * sample.state.velocity.y,
        0.0,
    ]);
    response.root_target_accelerations.push([
        request.root_horizontal_follow_ratio * sample.acceleration.x,
        request.root_horizontal_follow_ratio * sample.acceleration.y,
        0.0,
    ]);
    response.center_of_mass_targets.push(vector3_array(com));
    response.center_of_mass_target_velocities.push(velocity);
    response
        .center_of_mass_target_accelerations
        .push(acceleration);
    response.target_positions.push(feet.positions);
    response.target_velocities.push(feet.velocities);
    response.target_accelerations.push(feet.accelerations);
    response.reference_stance.push(feet.stance);
    Ok(())
}

fn root_position(request: &Request, com: Vec3) -> Vec3 {
    Vec3::new(
        request.initial_root[0]
            + request.root_horizontal_follow_ratio * (com.x - request.initial_com[0]),
        request.initial_root[1]
            + request.root_horizontal_follow_ratio * (com.y - request.initial_com[1]),
        request.initial_root[2],
    )
}

fn foot_sample(request: &Request, applied_touchdown: [f64; 3], tick: usize) -> Result<FootSample> {
    let mut positions = request.initial_feet;
    let mut velocities = [[0.0; 3]; 2];
    let mut accelerations = [[0.0; 3]; 2];
    let mut stance = [1_u8; 2];
    if tick >= request.liftoff_tick && tick < request.touchdown_tick {
        stance[request.swing_foot] = 0;
        let swing = swing_jet(request, applied_touchdown, tick)?;
        positions[request.swing_foot] = vector3_array(swing.value);
        velocities[request.swing_foot] = vector3_array(swing.velocity);
        accelerations[request.swing_foot] = vector3_array(swing.acceleration);
    } else if tick >= request.touchdown_tick {
        positions[request.swing_foot] = applied_touchdown;
    }
    Ok(FootSample {
        positions,
        velocities,
        accelerations,
        stance,
    })
}

fn swing_jet(request: &Request, applied_touchdown: [f64; 3], tick: usize) -> Result<VectorJet> {
    let swing_ticks = request.touchdown_tick - request.liftoff_tick;
    let phase = (tick - request.liftoff_tick) as f64 / swing_ticks as f64;
    let duration = swing_ticks as f64 * request.dt_seconds;
    let start = VectorJet {
        value: Vec3::from(request.initial_feet[request.swing_foot]),
        velocity: Vec3::zeros(),
        acceleration: Vec3::zeros(),
    };
    let end = VectorJet {
        value: Vec3::from(applied_touchdown),
        velocity: Vec3::zeros(),
        acceleration: Vec3::zeros(),
    };
    let mut sample = sample_quintic_vector_jet(start, end, duration, phase)
        .context("invalid swing interpolation")?;
    let one_minus_phase = 1.0 - phase;
    let bump = 64.0 * phase.powi(3) * one_minus_phase.powi(3);
    let bump_d_phase = 192.0 * phase.powi(2) * one_minus_phase.powi(2) * (1.0 - 2.0 * phase);
    let bump_dd_phase = 384.0 * phase * one_minus_phase * (1.0 - 5.0 * phase + 5.0 * phase.powi(2));
    sample.value.z += request.swing_height_m * bump;
    sample.velocity.z += request.swing_height_m * bump_d_phase / duration;
    sample.acceleration.z += request.swing_height_m * bump_dd_phase / duration.powi(2);
    Ok(sample)
}

fn exact_repeat(
    request: &Request,
    opening: &ConvexSupportPolygon<16>,
    terminal: &ConvexSupportPolygon<16>,
    config: LipmBoundaryPlannerConfig,
    applied_touchdown: [f64; 3],
    response: &Response,
) -> Result<bool> {
    let support_foot = request.initial_feet[request.terminal_support_foot];
    let repeat = LipmBoundaryPlan::plan(
        LipmState {
            position: Vec2::new(request.initial_com[0], request.initial_com[1]),
            velocity: Vec2::new(
                request.initial_com_velocity[0],
                request.initial_com_velocity[1],
            ),
        },
        LipmState {
            position: Vec2::new(
                support_foot[0] + request.contact_patch_center_x_m,
                support_foot[1],
            ),
            velocity: Vec2::zeros(),
        },
        opening,
        terminal,
        config,
    )?;
    for tick in 0..request.ticks {
        let sample = repeat.sample(tick as f64 * request.dt_seconds)?;
        let com = Vec3::new(
            sample.state.position.x,
            sample.state.position.y,
            request.initial_com[2],
        );
        let feet = foot_sample(request, applied_touchdown, tick)?;
        let expected_position = response.center_of_mass_targets[tick];
        let expected_velocity = response.center_of_mass_target_velocities[tick];
        let expected_acceleration = response.center_of_mass_target_accelerations[tick];
        if !same_lipm_bits(
            sample,
            expected_position,
            expected_velocity,
            expected_acceleration,
        ) || !same_array3_bits(
            vector3_array(root_position(request, com)),
            response.root_targets[tick],
        ) || !same_array3_bits(
            [
                request.root_horizontal_follow_ratio * sample.state.velocity.x,
                request.root_horizontal_follow_ratio * sample.state.velocity.y,
                0.0,
            ],
            response.root_target_velocities[tick],
        ) || !same_array3_bits(
            [
                request.root_horizontal_follow_ratio * sample.acceleration.x,
                request.root_horizontal_follow_ratio * sample.acceleration.y,
                0.0,
            ],
            response.root_target_accelerations[tick],
        ) || !same_nested_array3_bits(feet.positions, response.target_positions[tick])
            || !same_nested_array3_bits(feet.velocities, response.target_velocities[tick])
            || !same_nested_array3_bits(feet.accelerations, response.target_accelerations[tick])
            || feet.stance != response.reference_stance[tick]
        {
            return Ok(false);
        }
    }
    Ok(true)
}

fn reachable_touchdown(request: &Request, terminal_root: Vec3) -> Result<([f64; 3], f64, f64)> {
    let authored = Vec3::from(request.swing_touchdown);
    let delta = authored - terminal_root;
    let authored_reach = delta.norm();
    if authored_reach <= request.maximum_root_to_landing_reach_m {
        return Ok((request.swing_touchdown, authored_reach, authored_reach));
    }
    let horizontal = delta.fixed_rows::<2>(0).norm();
    let horizontal_limit_squared =
        request.maximum_root_to_landing_reach_m.powi(2) - delta.z.powi(2);
    if horizontal <= f64::EPSILON || horizontal_limit_squared <= 0.0 {
        bail!("landing reach cannot be repaired without changing authored height");
    }
    let horizontal_limit = horizontal_limit_squared.sqrt();
    let scale = horizontal_limit / horizontal;
    let applied = Vec3::new(
        terminal_root.x + delta.x * scale,
        terminal_root.y + delta.y * scale,
        authored.z,
    );
    Ok((
        vector3_array(applied),
        authored_reach,
        (applied - terminal_root).norm(),
    ))
}

fn same_lipm_bits(
    sample: LipmSample,
    position: [f64; 3],
    velocity: [f64; 3],
    acceleration: [f64; 3],
) -> bool {
    [sample.state.position.x, sample.state.position.y]
        .iter()
        .zip(&position[..2])
        .chain(
            [sample.state.velocity.x, sample.state.velocity.y]
                .iter()
                .zip(&velocity[..2]),
        )
        .chain(
            [sample.acceleration.x, sample.acceleration.y]
                .iter()
                .zip(&acceleration[..2]),
        )
        .all(|(left, right)| left.to_bits() == right.to_bits())
}

fn same_array3_bits(left: [f64; 3], right: [f64; 3]) -> bool {
    left.iter()
        .zip(right.iter())
        .all(|(left, right)| left.to_bits() == right.to_bits())
}

fn same_nested_array3_bits(left: [[f64; 3]; 2], right: [[f64; 3]; 2]) -> bool {
    left.iter()
        .flatten()
        .zip(right.iter().flatten())
        .all(|(left, right)| left.to_bits() == right.to_bits())
}

fn vector2_array(value: Vec2) -> [f64; 2] {
    [value.x, value.y]
}

fn vector3_array(value: Vec3) -> [f64; 3] {
    [value.x, value.y, value.z]
}

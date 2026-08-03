//! Deterministic Rust-side audit for the support-transfer motion-headroom
//! witness. Python owns report rendering; this binary owns model semantics and
//! the hot-loop allocation measurement.

use std::{
    alloc::{GlobalAlloc, Layout, System},
    path::PathBuf,
    sync::atomic::{AtomicU64, Ordering},
    time::Instant,
};

use bonesaw_core::{MotionProgram, RobotState, TimingSpec, minimum_joint_motion_headroom};
use serde::Serialize;

struct CountingAllocator;

static ALLOCATION_CALLS: AtomicU64 = AtomicU64::new(0);
static ALLOCATED_BYTES: AtomicU64 = AtomicU64::new(0);
static DEALLOCATION_CALLS: AtomicU64 = AtomicU64::new(0);

#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator;

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

    unsafe fn realloc(&self, ptr: *mut u8, old: Layout, new_size: usize) -> *mut u8 {
        ALLOCATION_CALLS.fetch_add(1, Ordering::Relaxed);
        ALLOCATED_BYTES.fetch_add(new_size as u64, Ordering::Relaxed);
        unsafe { System.realloc(ptr, old, new_size) }
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        DEALLOCATION_CALLS.fetch_add(1, Ordering::Relaxed);
        unsafe { System.dealloc(ptr, layout) }
    }
}

#[derive(Clone, Copy)]
struct AllocationSnapshot {
    calls: u64,
    bytes: u64,
    deallocations: u64,
}

fn allocation_snapshot() -> AllocationSnapshot {
    AllocationSnapshot {
        calls: ALLOCATION_CALLS.load(Ordering::Relaxed),
        bytes: ALLOCATED_BYTES.load(Ordering::Relaxed),
        deallocations: DEALLOCATION_CALLS.load(Ordering::Relaxed),
    }
}

#[derive(Serialize)]
struct CaseResult {
    name: &'static str,
    coordinate: Option<usize>,
    position_margin_rad: Option<f64>,
    stopping_margin_rad: Option<f64>,
    velocity_fraction: Option<f64>,
    fraction_of_range: Option<f64>,
    nanoseconds_per_call: f64,
    measured_allocation_calls: u64,
    measured_allocated_bytes: u64,
    measured_deallocation_calls: u64,
    repeated_result_bitwise_equal: bool,
}

fn main() -> anyhow::Result<()> {
    let model_path = std::env::args()
        .nth(1)
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("models/upkie/upkie.urdf"));
    let program = MotionProgram::compile_urdf_file(model_path, TimingSpec::default(), 1)?;
    let mut state = RobotState::zeros(&program.model);
    let coordinate = 0;
    let repetitions = 20_000_u64;
    let maximum_acceleration = 200.0;
    let reaction_time_seconds = 0.02;
    let cases = [
        ("centered", 0.0, 0.0),
        ("moving", 0.70, 4.0),
        ("near_limit", 1.20, 12.0),
    ];
    let mut results = Vec::with_capacity(cases.len());
    let mut checksum = 0.0_f64;
    for (name, position, velocity) in cases {
        state.q.fill(0.0);
        state.v.fill(0.0);
        if coordinate < state.q.len() {
            state.q[coordinate] = position;
            state.v[coordinate] = velocity;
        }

        for _ in 0..256 {
            checksum += minimum_joint_motion_headroom(
                &program.model,
                &state,
                maximum_acceleration,
                reaction_time_seconds,
            )
            .ok()
            .flatten()
            .map_or(0.0, |headroom| headroom.fraction_of_range);
        }
        let first = minimum_joint_motion_headroom(
            &program.model,
            &state,
            maximum_acceleration,
            reaction_time_seconds,
        )?;
        let allocations_before = allocation_snapshot();
        let started = Instant::now();
        let mut last = first;
        for _ in 0..repetitions {
            last = minimum_joint_motion_headroom(
                &program.model,
                &state,
                maximum_acceleration,
                reaction_time_seconds,
            )?;
            checksum += last.map_or(0.0, |value| value.fraction_of_range);
        }
        let elapsed_ns = started.elapsed().as_nanos();
        let allocations_after = allocation_snapshot();
        let output = last;
        results.push(CaseResult {
            name,
            coordinate: output.map(|value| value.coordinate),
            position_margin_rad: output.map(|value| value.position_margin_rad),
            stopping_margin_rad: output.map(|value| value.stopping_margin_rad),
            velocity_fraction: output.map(|value| value.velocity_fraction),
            fraction_of_range: output.map(|value| value.fraction_of_range),
            nanoseconds_per_call: elapsed_ns as f64 / repetitions as f64,
            measured_allocation_calls: allocations_after.calls - allocations_before.calls,
            measured_allocated_bytes: allocations_after.bytes - allocations_before.bytes,
            measured_deallocation_calls: allocations_after.deallocations
                - allocations_before.deallocations,
            repeated_result_bitwise_equal: output == first,
        });
    }
    std::hint::black_box(checksum);
    println!(
        "{}",
        serde_json::to_string(&serde_json::json!({
            "schema": "bonesaw.joint-motion-headroom-r288.v1",
            "model": program.model.name,
            "repetitions_per_case": repetitions,
            "maximum_acceleration_mps2": maximum_acceleration,
            "reaction_time_seconds": reaction_time_seconds,
            "cases": results,
            "allocation_claim": "caller-owned model/state; measured hot-loop witness performs no allocations",
        }))?
    );
    Ok(())
}

//! Measure the caller-owned historical frame-query batch surface.
//!
//! This binary is intentionally a thin Rust harness. Python owns the
//! evaluation/reporting schedule; the core owns atlas evaluation and query
//! semantics. It emits one JSON object so the eval can retain the raw result
//! without teaching Python any model math.

use std::{
    alloc::{GlobalAlloc, Layout, System},
    path::PathBuf,
    sync::atomic::{AtomicU64, Ordering},
    time::Instant,
};

use bonesaw_core::{
    CompiledFrameAtlas, ExternalFrameHistories, ExternalFrameSample, HistoricalAtlasEstimate,
    HistoricalFrameQuery, HistoricalFrameQueryWorkspace, HistoryQueryPolicy, ModelCache,
    MotionProgram, RobotHistory, RobotState, TimedRobotState, TimingSpec,
};
use nalgebra::DVector;
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
struct AuditResult {
    schema: &'static str,
    model: String,
    query_count: usize,
    warmup_batches: usize,
    measured_batches: usize,
    elapsed_ns: u128,
    nanoseconds_per_query: f64,
    output_external_capacity_before: usize,
    output_external_capacity_after: usize,
    workspace_external_capacity_before: usize,
    workspace_external_capacity_after: usize,
    workspace_external_sample_capacity_before: usize,
    workspace_external_sample_capacity_after: usize,
    measured_allocation_calls: u64,
    measured_allocated_bytes: u64,
    measured_deallocation_calls: u64,
    repeated_results_bitwise_equal: bool,
    allocation_claim: &'static str,
}

fn main() -> anyhow::Result<()> {
    let model_path = std::env::args()
        .nth(1)
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("models/upkie/upkie.urdf"));
    let program = MotionProgram::compile_urdf_file(model_path, TimingSpec::default(), 1)?;
    let model = &program.model;
    let atlas = CompiledFrameAtlas::standard(model);
    let base = atlas
        .frame_id(model.bodies[model.root.0].name.as_str())
        .expect("root body is present in standard atlas");
    let mut robot_history = RobotHistory::new(8);
    for sample_index in 0..8_i64 {
        let mut state = RobotState::zeros(model);
        state.control_world_from_root.translation.vector.x = sample_index as f64 * 0.001;
        state.q = DVector::from_element(model.dof, 0.0);
        state.v = DVector::from_element(model.dof, 0.0);
        robot_history.push(TimedRobotState {
            time_ns: sample_index * 20_000_000,
            sequence: sample_index as u64,
            state,
        });
    }
    let mut external_histories = ExternalFrameHistories::new([8, 8]);
    for slot in 0..2 {
        let history = external_histories
            .slot_mut(slot)
            .expect("standard atlas has two external slots");
        for sample_index in 0..8_i64 {
            history.push(ExternalFrameSample {
                time_ns: sample_index * 20_000_000,
                anchor_from_frame: nalgebra::Isometry3::identity(),
                twist: None,
                acceleration: None,
                covariance: None,
                sequence: sample_index as u64,
            });
        }
    }
    let policy = HistoryQueryPolicy {
        maximum_interpolation_gap_ns: 40_000_000,
        maximum_extrapolation_ns: 0,
        allow_prediction: false,
        allow_hold: false,
    };
    let query_count = 64;
    let queries = (0..query_count)
        .map(|index| HistoricalFrameQuery {
            from: atlas.map,
            to: base,
            time_ns: 10_000_000 + (index as i64 % 7) * 20_000_000,
            policy,
        })
        .collect::<Vec<_>>();
    let mut outputs = (0..query_count)
        .map(|_| HistoricalAtlasEstimate::with_external_capacity(atlas.external_slot_count))
        .collect::<Vec<_>>();
    let output_external_capacity_before = outputs[0].external_capacity();
    let mut model_cache = ModelCache::new(model);
    let mut external_inputs = bonesaw_core::ExternalFrameInputs::new(&atlas);
    let mut snapshot = bonesaw_core::FrameAtlasSnapshot::new(&atlas);
    let mut workspace = HistoricalFrameQueryWorkspace::new(model, &atlas);
    let workspace_external_capacity_before = workspace.external_capacity();
    let workspace_external_sample_capacity_before = workspace.external_sample_capacity();

    for _ in 0..8 {
        atlas.query_history_batch_with_workspace(
            model,
            &robot_history,
            &external_histories,
            &queries,
            &mut workspace,
            &mut model_cache,
            &mut external_inputs,
            &mut snapshot,
            &mut outputs,
        )?;
    }
    let first_results = outputs
        .iter()
        .map(|output| output.estimate.from_to)
        .collect::<Vec<_>>();
    let measured_batches = 32;
    let allocations_before = allocation_snapshot();
    let started = Instant::now();
    for _ in 0..measured_batches {
        atlas.query_history_batch_with_workspace(
            model,
            &robot_history,
            &external_histories,
            &queries,
            &mut workspace,
            &mut model_cache,
            &mut external_inputs,
            &mut snapshot,
            &mut outputs,
        )?;
    }
    let elapsed_ns = started.elapsed().as_nanos();
    let allocations_after = allocation_snapshot();
    let repeated_results_bitwise_equal = outputs
        .iter()
        .zip(first_results.iter())
        .all(|(output, expected)| output.estimate.from_to == *expected);
    let output_external_capacity_after = outputs[0].external_capacity();
    let workspace_external_capacity_after = workspace.external_capacity();
    let workspace_external_sample_capacity_after = workspace.external_sample_capacity();
    let total_queries = query_count * measured_batches;
    println!(
        "{}",
        serde_json::to_string(&AuditResult {
            schema: "bonesaw.frame-query-allocation-r287.v1",
            model: model.name.clone(),
            query_count,
            warmup_batches: 8,
            measured_batches,
            elapsed_ns,
            nanoseconds_per_query: elapsed_ns as f64 / total_queries as f64,
            output_external_capacity_before,
            output_external_capacity_after,
            workspace_external_capacity_before,
            workspace_external_capacity_after,
            workspace_external_sample_capacity_before: workspace_external_sample_capacity_before,
            workspace_external_sample_capacity_after: workspace_external_sample_capacity_after,
            measured_allocation_calls: allocations_after.calls - allocations_before.calls,
            measured_allocated_bytes: allocations_after.bytes - allocations_before.bytes,
            measured_deallocation_calls: allocations_after.deallocations
                - allocations_before.deallocations,
            repeated_results_bitwise_equal,
            allocation_claim: "strict caller-owned historical reconstruction and atlas query batch records zero allocation/deallocation calls in the measured loop",
        })?
    );
    Ok(())
}

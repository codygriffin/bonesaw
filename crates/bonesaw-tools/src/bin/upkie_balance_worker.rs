use std::{
    alloc::{GlobalAlloc, Layout, System},
    env,
    fs::File,
    io::{BufRead, BufReader, BufWriter, Write},
    path::PathBuf,
    sync::atomic::{AtomicU64, Ordering},
    time::Instant,
};

use anyhow::{Context, Result, bail};
use bonesaw_core::{MotionProgram, RobotState, TimingSpec};
use bonesaw_tools::{UpkieWheelBalancer, UpkieWheelBalancerState};

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

#[derive(Clone, Copy)]
struct Sample {
    ground_position: f64,
    pitch: f64,
}

#[derive(Clone, Copy)]
struct Record {
    ground_velocity: f64,
    left_wheel_velocity: f64,
    right_wheel_velocity: f64,
    integral_velocity: f64,
    elapsed_ns: u64,
}

fn main() -> Result<()> {
    let mut arguments = env::args_os().skip(1);
    let input_path = arguments.next().map(PathBuf::from).context(
        "usage: bonesaw-upkie-balance-worker INPUT.tsv OUTPUT.tsv [MODEL.urdf] [official|live]",
    )?;
    let output_path = arguments.next().map(PathBuf::from).context(
        "usage: bonesaw-upkie-balance-worker INPUT.tsv OUTPUT.tsv [MODEL.urdf] [official|live]",
    )?;
    let model_path = arguments
        .next()
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("models/upkie/upkie.urdf"));
    let profile = arguments
        .next()
        .map(|value| value.to_string_lossy().into_owned())
        .unwrap_or_else(|| "official".to_owned());
    if arguments.next().is_some() {
        bail!("unexpected extra argument");
    }
    if profile != "official" && profile != "live" {
        bail!("profile must be `official` or `live`");
    }

    let input = BufReader::new(
        File::open(&input_path)
            .with_context(|| format!("opening shared corpus {}", input_path.display()))?,
    );
    let mut samples = Vec::new();
    for (line_index, line) in input.lines().enumerate() {
        let line = line?;
        if line_index == 0 || line.trim().is_empty() {
            continue;
        }
        let mut fields = line.split_ascii_whitespace();
        let ground_position = fields
            .next()
            .context("missing ground position")?
            .parse()
            .context("invalid ground position")?;
        let pitch = fields
            .next()
            .context("missing pitch")?
            .parse()
            .context("invalid pitch")?;
        if fields.next().is_some() {
            bail!("unexpected field on corpus line {}", line_index + 1);
        }
        samples.push(Sample {
            ground_position,
            pitch,
        });
    }

    let program = MotionProgram::compile_urdf_file(&model_path, TimingSpec::default(), 1)
        .with_context(|| format!("compiling {}", model_path.display()))?;
    let robot = RobotState::zeros(&program.model);
    let mut balancer = UpkieWheelBalancer::compile(&program.model, &robot)?;
    if profile == "official" {
        // Match the pinned upstream WheelBalancer::Parameters defaults exactly.
        balancer.config.wheel_radius = 0.06;
        balancer.config.position_damping = 0.7;
        balancer.config.position_stiffness = 1.6;
        balancer.config.pitch_damping = 1.8;
        balancer.config.pitch_stiffness = 20.0;
        balancer.config.maximum_ground_velocity = 2.0;
        balancer.config.maximum_integral_velocity = 10.0;
    }

    let mut state = UpkieWheelBalancerState::default();
    let mut desired_accelerations = [0.0; 2];
    let mut records = Vec::with_capacity(samples.len());
    let mut allocation_calls = 0_u64;
    let mut allocated_bytes = 0_u64;
    for sample in samples {
        let calls_before = ALLOCATION_CALLS.load(Ordering::Relaxed);
        let bytes_before = ALLOCATED_BYTES.load(Ordering::Relaxed);
        let started = Instant::now();
        let ground_velocity = balancer.emit_accelerations(
            0.005,
            0.0,
            sample.ground_position,
            sample.pitch,
            robot.v.as_slice(),
            &mut state,
            &mut desired_accelerations,
        );
        let elapsed_ns = started.elapsed().as_nanos() as u64;
        allocation_calls += ALLOCATION_CALLS.load(Ordering::Relaxed) - calls_before;
        allocated_bytes += ALLOCATED_BYTES.load(Ordering::Relaxed) - bytes_before;
        records.push(Record {
            ground_velocity,
            left_wheel_velocity: ground_velocity / balancer.config.wheel_radius,
            right_wheel_velocity: -ground_velocity / balancer.config.wheel_radius,
            integral_velocity: state.integral_velocity,
            elapsed_ns,
        });
    }

    let mut output = BufWriter::new(
        File::create(&output_path)
            .with_context(|| format!("creating output {}", output_path.display()))?,
    );
    writeln!(
        output,
        "ground_velocity\tleft_wheel_velocity\tright_wheel_velocity\tintegral_velocity\telapsed_ns"
    )?;
    for record in records {
        writeln!(
            output,
            "{:.17e}\t{:.17e}\t{:.17e}\t{:.17e}\t{}",
            record.ground_velocity,
            record.left_wheel_velocity,
            record.right_wheel_velocity,
            record.integral_velocity,
            record.elapsed_ns,
        )?;
    }
    eprintln!("{{\"allocation_calls\":{allocation_calls},\"allocated_bytes\":{allocated_bytes}}}");
    Ok(())
}

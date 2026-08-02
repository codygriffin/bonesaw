use std::{env, path::PathBuf};

use anyhow::{Context, Result, bail};
use bonesaw_core::{CollisionAvoidanceConfig, MotionProgram, TimingSpec};

fn main() -> Result<()> {
    let mut model = PathBuf::from("models/toy_humanoid.urdf");
    let mut output = PathBuf::from("bonesaw.motion");
    let mut epoch = 1_u64;
    let mut collision_avoidance = false;
    let mut arguments = env::args().skip(1);
    while let Some(argument) = arguments.next() {
        match argument.as_str() {
            "--model" => {
                model = arguments
                    .next()
                    .map(PathBuf::from)
                    .context("--model requires a path")?;
            }
            "--output" => {
                output = arguments
                    .next()
                    .map(PathBuf::from)
                    .context("--output requires a path")?;
            }
            "--epoch" => {
                epoch = arguments
                    .next()
                    .context("--epoch requires an integer")?
                    .parse()
                    .context("invalid --epoch value")?;
            }
            "--collision-avoidance" => collision_avoidance = true,
            "--help" | "-h" => {
                println!(
                    "Usage: bonesaw-compile [--model PATH] [--output PATH] [--epoch N] \\\n+                     [--collision-avoidance]\n\
                     Compiles a URDF into a checksummed canonical MotionProgram archive."
                );
                return Ok(());
            }
            unknown => bail!("unknown argument {unknown}; try --help"),
        }
    }

    let mut program = MotionProgram::compile_urdf_file(&model, TimingSpec::default(), epoch)
        .with_context(|| format!("compiling {}", model.display()))?;
    if collision_avoidance {
        program = program
            .with_collision_avoidance(CollisionAvoidanceConfig::default())
            .context("compiling default collision avoidance policy")?;
    }
    program
        .write_archive(&output)
        .with_context(|| format!("writing {}", output.display()))?;
    let verified = MotionProgram::read_archive(&output)
        .with_context(|| format!("verifying {}", output.display()))?;
    if verified.header.fingerprint_sha256 != program.header.fingerprint_sha256 {
        bail!("archive verification produced a different program fingerprint");
    }
    let fingerprint: String = program
        .header
        .fingerprint_sha256
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect();
    println!(
        "{}\n{} bodies, {} DOF, {} actuators, schema {}, epoch {}, collision avoidance {}, fingerprint {}",
        output.display(),
        program.model.bodies.len(),
        program.model.dof,
        program.actuation.actuators.len(),
        program.header.schema_version,
        program.header.program_epoch,
        if program.collision_avoidance.is_some() {
            "enabled"
        } else {
            "disabled"
        },
        fingerprint
    );
    Ok(())
}

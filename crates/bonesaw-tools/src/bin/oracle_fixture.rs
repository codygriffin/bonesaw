use std::{env, path::PathBuf};

use anyhow::{Context, Result, bail};
use bonesaw_core::{
    DynamicsCache, Force6, FrameId, ModelCache, Motion6, MotionProgram, RobotState, TimingSpec,
    Vec3,
};
use nalgebra::{DMatrix, DVector};
use serde::Serialize;

#[derive(Serialize)]
struct Fixture {
    model_name: String,
    urdf_path: String,
    coordinate_names: Vec<String>,
    lumped_bodies: Vec<LumpedBodyFixture>,
    gravity_world: [f64; 3],
    samples: Vec<Sample>,
}

#[derive(Serialize)]
struct LumpedBodyFixture {
    name: String,
    mass: f64,
    com_in_body: [f64; 3],
}

#[derive(Serialize)]
struct Sample {
    q: Vec<f64>,
    v: Vec<f64>,
    acceleration: Vec<f64>,
    center_of_mass_world: [f64; 3],
    frames: Vec<FrameFixture>,
    mass_matrix_row_major: Vec<f64>,
    generalized_gravity: Vec<f64>,
    inverse_dynamics: Vec<f64>,
    centroidal_map_moment_force_row_major: Vec<f64>,
    centroidal_momentum_moment_force: [f64; 6],
    root_twist_angular_linear: [f64; 6],
    floating_acceleration_angular_linear_joint: Vec<f64>,
    floating_mass_matrix_row_major: Vec<f64>,
    floating_bias_moment_force_joint: Vec<f64>,
    floating_inverse_dynamics_moment_force_joint: Vec<f64>,
    floating_centroidal_map_moment_force_row_major: Vec<f64>,
}

#[derive(Serialize)]
struct FrameFixture {
    name: String,
    translation: [f64; 3],
    rotation_matrix_row_major: [f64; 9],
    jacobian_angular_linear_row_major: Vec<f64>,
}

fn main() -> Result<()> {
    let mut model_path = PathBuf::from("models/upkie/upkie.urdf");
    let mut samples = 20_usize;
    let mut arguments = env::args().skip(1);
    while let Some(argument) = arguments.next() {
        match argument.as_str() {
            "--model" => {
                model_path = arguments
                    .next()
                    .map(PathBuf::from)
                    .context("--model requires a path")?;
            }
            "--samples" => {
                samples = arguments
                    .next()
                    .context("--samples requires a positive integer")?
                    .parse()
                    .context("invalid --samples value")?;
                if samples == 0 {
                    bail!("--samples must be positive");
                }
            }
            "--help" | "-h" => {
                println!(
                    "Usage: bonesaw-oracle-fixture [--model PATH] [--samples N]\n\
                     Emits deterministic Bonesaw FK/Jacobian/dynamics fixtures as JSON."
                );
                return Ok(());
            }
            unknown => bail!("unknown argument {unknown}; try --help"),
        }
    }

    let program = MotionProgram::compile_urdf_file(&model_path, TimingSpec::default(), 1)
        .with_context(|| format!("compiling {}", model_path.display()))?;
    let model = &program.model;
    let gravity = Vec3::new(0.0, 0.0, -9.81);
    let mut model_cache = ModelCache::new(model);
    let mut dynamics_cache = DynamicsCache::new(model);
    let mut output_samples = Vec::with_capacity(samples);

    for sample_index in 0..samples {
        let mut state = RobotState::zeros(model);
        for joint in &model.joints {
            let Some(index) = joint.coordinate else {
                continue;
            };
            let center = if joint.limit.lower.is_finite() && joint.limit.upper.is_finite() {
                0.5 * (joint.limit.lower + joint.limit.upper)
            } else {
                0.0
            };
            let range = if joint.limit.lower.is_finite() && joint.limit.upper.is_finite() {
                0.2 * (joint.limit.upper - joint.limit.lower)
            } else {
                0.35
            };
            state.q[index] =
                center + range * (sample_index as f64 * 0.37 + index as f64 * 0.61).sin();
            state.v[index] = 0.3 * (sample_index as f64 * 0.19 + index as f64 * 0.43).cos();
        }
        let acceleration = DVector::from_iterator(
            model.dof,
            (0..model.dof)
                .map(|index| 0.4 * (sample_index as f64 * 0.23 + index as f64 * 0.31).sin()),
        );
        let root_twist = Motion6(nalgebra::SVector::<f64, 6>::from_fn(|index, _| {
            0.13 * (sample_index as f64 * 0.17 + index as f64 * 0.29).cos()
        }));
        let floating_acceleration = DVector::from_iterator(
            model.dof + 6,
            (0..model.dof + 6)
                .map(|index| 0.21 * (sample_index as f64 * 0.11 + index as f64 * 0.23).sin()),
        );
        model.forward_kinematics(&state, &mut model_cache)?;
        let mass = model.mass_matrix(&model_cache)?;
        let generalized_gravity = model.gravity_forces(&model_cache, 9.81)?;
        let inverse_dynamics = model.inverse_dynamics(
            &state,
            &acceleration,
            gravity,
            &model_cache,
            &mut dynamics_cache,
        )?;
        let mut centroidal_map = DMatrix::zeros(6, model.dof);
        let mut centroidal_momentum = Force6::default();
        model.centroidal_momentum_into(
            &state,
            &model_cache,
            &mut dynamics_cache,
            &mut centroidal_map,
            &mut centroidal_momentum,
        )?;
        let mut floating_mass = DMatrix::zeros(model.dof + 6, model.dof + 6);
        model.floating_mass_matrix_into(&model_cache, &mut dynamics_cache, &mut floating_mass)?;
        let mut floating_bias = DVector::zeros(model.dof + 6);
        model.floating_bias_forces_into(
            &state,
            root_twist,
            gravity,
            &model_cache,
            &mut dynamics_cache,
            &mut floating_bias,
        )?;
        let mut floating_inverse_dynamics = DVector::zeros(model.dof + 6);
        model.floating_inverse_dynamics_into(
            &state,
            root_twist,
            &floating_acceleration,
            gravity,
            &model_cache,
            &mut dynamics_cache,
            &mut floating_inverse_dynamics,
        )?;
        let mut floating_centroidal_map = DMatrix::zeros(6, model.dof + 6);
        model.floating_centroidal_map_into(
            &model_cache,
            &mut dynamics_cache,
            &mut floating_centroidal_map,
        )?;
        let frames = model
            .bodies
            .iter()
            .map(|body| -> Result<FrameFixture> {
                let pose = model.frame_pose(&model_cache, FrameId(body.id.0))?;
                let rotation = pose.rotation.to_rotation_matrix();
                let jacobian = model.frame_jacobian(&model_cache, FrameId(body.id.0))?;
                Ok(FrameFixture {
                    name: body.name.clone(),
                    translation: [pose.translation.x, pose.translation.y, pose.translation.z],
                    rotation_matrix_row_major: matrix3_row_major(rotation.matrix()),
                    jacobian_angular_linear_row_major: matrix_row_major(&jacobian),
                })
            })
            .collect::<Result<Vec<_>>>()?;
        output_samples.push(Sample {
            q: state.q.as_slice().to_vec(),
            v: state.v.as_slice().to_vec(),
            acceleration: acceleration.as_slice().to_vec(),
            center_of_mass_world: [
                model_cache.center_of_mass_world.x,
                model_cache.center_of_mass_world.y,
                model_cache.center_of_mass_world.z,
            ],
            frames,
            mass_matrix_row_major: matrix_row_major(&mass),
            generalized_gravity: generalized_gravity.as_slice().to_vec(),
            inverse_dynamics: inverse_dynamics.as_slice().to_vec(),
            centroidal_map_moment_force_row_major: matrix_row_major(&centroidal_map),
            centroidal_momentum_moment_force: [
                centroidal_momentum.0[0],
                centroidal_momentum.0[1],
                centroidal_momentum.0[2],
                centroidal_momentum.0[3],
                centroidal_momentum.0[4],
                centroidal_momentum.0[5],
            ],
            root_twist_angular_linear: [
                root_twist.0[0],
                root_twist.0[1],
                root_twist.0[2],
                root_twist.0[3],
                root_twist.0[4],
                root_twist.0[5],
            ],
            floating_acceleration_angular_linear_joint: floating_acceleration.as_slice().to_vec(),
            floating_mass_matrix_row_major: matrix_row_major(&floating_mass),
            floating_bias_moment_force_joint: floating_bias.as_slice().to_vec(),
            floating_inverse_dynamics_moment_force_joint: floating_inverse_dynamics
                .as_slice()
                .to_vec(),
            floating_centroidal_map_moment_force_row_major: matrix_row_major(
                &floating_centroidal_map,
            ),
        });
    }

    let fixture = Fixture {
        model_name: model.name.clone(),
        urdf_path: model_path.display().to_string(),
        coordinate_names: model
            .coordinate_names()
            .into_iter()
            .map(str::to_owned)
            .collect(),
        lumped_bodies: model
            .bodies
            .iter()
            .filter(|body| body.mass > 0.0)
            .map(|body| LumpedBodyFixture {
                name: body.name.clone(),
                mass: body.mass,
                com_in_body: [body.com_in_body.x, body.com_in_body.y, body.com_in_body.z],
            })
            .collect(),
        gravity_world: [gravity.x, gravity.y, gravity.z],
        samples: output_samples,
    };
    println!("{}", serde_json::to_string(&fixture)?);
    Ok(())
}

fn matrix3_row_major(matrix: &nalgebra::Matrix3<f64>) -> [f64; 9] {
    [
        matrix[(0, 0)],
        matrix[(0, 1)],
        matrix[(0, 2)],
        matrix[(1, 0)],
        matrix[(1, 1)],
        matrix[(1, 2)],
        matrix[(2, 0)],
        matrix[(2, 1)],
        matrix[(2, 2)],
    ]
}

fn matrix_row_major(matrix: &DMatrix<f64>) -> Vec<f64> {
    let mut output = Vec::with_capacity(matrix.nrows() * matrix.ncols());
    for row in 0..matrix.nrows() {
        for column in 0..matrix.ncols() {
            output.push(matrix[(row, column)]);
        }
    }
    output
}

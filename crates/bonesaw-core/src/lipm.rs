//! Allocation-free horizontal LIPM reference planning.
//!
//! This module plans references, not feedback. A plan joins an observed
//! horizontal CoM state to an equilibrium state above a future support polygon
//! using two exact constant-CoP arcs. Candidate switch times are searched at
//! construction time and are admitted only when the first CoP is inside the
//! opening support polygon and the second CoP is inside the future support
//! polygon. Sampling is closed form and uses no heap storage.

use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::math::Vec2;

#[derive(Clone, Copy, Debug, Error, Eq, PartialEq)]
pub enum LipmPlanError {
    #[error("support polygon must contain at least three finite, strictly CCW convex vertices")]
    SupportPolygon,
    #[error("LIPM planner configuration is invalid")]
    Configuration,
    #[error("LIPM boundary state contains NaN or infinity")]
    Boundary,
    #[error("no two-stage CoP plan satisfies the support-boundary constraints")]
    Infeasible,
}

/// Fixed-capacity, counter-clockwise convex support polygon.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ConvexSupportPolygon<const MAX_VERTICES: usize> {
    vertices: [Vec2; MAX_VERTICES],
    len: usize,
}

impl<const MAX_VERTICES: usize> ConvexSupportPolygon<MAX_VERTICES> {
    pub fn from_ccw_vertices(vertices: &[Vec2]) -> Result<Self, LipmPlanError> {
        if vertices.len() < 3
            || vertices.len() > MAX_VERTICES
            || !vertices
                .iter()
                .flat_map(Vec2::iter)
                .all(|value| value.is_finite())
        {
            return Err(LipmPlanError::SupportPolygon);
        }
        let mut storage = [Vec2::zeros(); MAX_VERTICES];
        storage[..vertices.len()].copy_from_slice(vertices);
        let polygon = Self {
            vertices: storage,
            len: vertices.len(),
        };
        let mut orientation = 0.0;
        for index in 0..polygon.len {
            let a = polygon.vertices[index];
            let b = polygon.vertices[(index + 1) % polygon.len];
            let c = polygon.vertices[(index + 2) % polygon.len];
            let edge = b - a;
            let next = c - b;
            let cross = edge.x.mul_add(next.y, -edge.y * next.x);
            if edge.norm_squared() <= f64::EPSILON || cross <= 1.0e-14 {
                return Err(LipmPlanError::SupportPolygon);
            }
            orientation += a.x.mul_add(b.y, -a.y * b.x);
        }
        if orientation <= 0.0 {
            return Err(LipmPlanError::SupportPolygon);
        }
        Ok(polygon)
    }

    pub fn vertices(&self) -> &[Vec2] {
        &self.vertices[..self.len]
    }

    /// Euclidean distance to the nearest edge, positive inside.
    pub fn signed_margin(&self, point: Vec2) -> f64 {
        let mut minimum = f64::INFINITY;
        for index in 0..self.len {
            let start = self.vertices[index];
            let edge = self.vertices[(index + 1) % self.len] - start;
            let cross = edge
                .x
                .mul_add(point.y - start.y, -edge.y * (point.x - start.x));
            minimum = minimum.min(cross / edge.norm());
        }
        minimum
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct LipmState {
    pub position: Vec2,
    pub velocity: Vec2,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct LipmSample {
    pub state: LipmState,
    pub acceleration: Vec2,
    pub cop: Vec2,
    pub dcm: Vec2,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct LipmBoundaryPlannerConfig {
    pub gravity_mps2: f64,
    pub com_height_m: f64,
    pub duration_seconds: f64,
    /// Entry ramp from the initial CoM projection to the first planned CoP.
    /// When the boundary velocity is zero this makes initial acceleration
    /// exactly zero instead of applying an impulse-like acceleration step.
    pub cop_startup_duration_seconds: f64,
    /// Linear transfer time between the two constant-CoP arcs.
    pub cop_transition_duration_seconds: f64,
    pub minimum_switch_ratio: f64,
    pub maximum_switch_ratio: f64,
    pub switch_candidates: usize,
    pub minimum_cop_margin_m: f64,
}

impl Default for LipmBoundaryPlannerConfig {
    fn default() -> Self {
        Self {
            gravity_mps2: 9.81,
            com_height_m: 0.7,
            duration_seconds: 1.0,
            cop_startup_duration_seconds: 0.1,
            cop_transition_duration_seconds: 0.1,
            minimum_switch_ratio: 0.05,
            maximum_switch_ratio: 0.95,
            switch_candidates: 91,
            minimum_cop_margin_m: 0.0,
        }
    }
}

impl LipmBoundaryPlannerConfig {
    fn validate(self) -> Result<Self, LipmPlanError> {
        if !self.gravity_mps2.is_finite()
            || self.gravity_mps2 <= 0.0
            || !self.com_height_m.is_finite()
            || self.com_height_m <= 0.0
            || !self.duration_seconds.is_finite()
            || self.duration_seconds <= 0.0
            || !self.cop_startup_duration_seconds.is_finite()
            || self.cop_startup_duration_seconds <= 0.0
            || !self.cop_transition_duration_seconds.is_finite()
            || self.cop_transition_duration_seconds <= 0.0
            || self.cop_transition_duration_seconds >= self.duration_seconds
            || self.cop_startup_duration_seconds + self.cop_transition_duration_seconds
                >= self.duration_seconds
            || !self.minimum_switch_ratio.is_finite()
            || !self.maximum_switch_ratio.is_finite()
            || self.minimum_switch_ratio <= 0.0
            || self.minimum_switch_ratio >= self.maximum_switch_ratio
            || self.maximum_switch_ratio >= 1.0
            || self.switch_candidates < 2
            || !self.minimum_cop_margin_m.is_finite()
            || self.minimum_cop_margin_m < 0.0
        {
            return Err(LipmPlanError::Configuration);
        }
        Ok(self)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct LipmBoundaryPlan {
    pub config: LipmBoundaryPlannerConfig,
    pub initial_state: LipmState,
    pub terminal_state: LipmState,
    pub switch_time_seconds: f64,
    pub cop_startup_end_seconds: f64,
    pub cop_transition_start_seconds: f64,
    pub cop_transition_end_seconds: f64,
    pub first_cop: Vec2,
    pub second_cop: Vec2,
    pub terminal_cop: Vec2,
    pub first_cop_margin_m: f64,
    pub second_cop_margin_m: f64,
    pub terminal_cop_margin_m: f64,
    initial_cop: Vec2,
    startup_end_state: LipmState,
    transition_start_state: LipmState,
    transition_end_state: LipmState,
}

impl LipmBoundaryPlan {
    pub fn plan<const OPENING_VERTICES: usize, const TERMINAL_VERTICES: usize>(
        initial_state: LipmState,
        terminal_state: LipmState,
        opening_support: &ConvexSupportPolygon<OPENING_VERTICES>,
        terminal_support: &ConvexSupportPolygon<TERMINAL_VERTICES>,
        config: LipmBoundaryPlannerConfig,
    ) -> Result<Self, LipmPlanError> {
        let config = config.validate()?;
        if !state_is_finite(initial_state) || !state_is_finite(terminal_state) {
            return Err(LipmPlanError::Boundary);
        }
        // The post-boundary hold is an equilibrium. Nonzero terminal velocity
        // needs a longer support sequence and is deliberately rejected here.
        if terminal_state.velocity.norm() > 1.0e-12 {
            return Err(LipmPlanError::Boundary);
        }
        let terminal_cop = terminal_state.position;
        let terminal_margin = terminal_support.signed_margin(terminal_cop);
        if terminal_margin < config.minimum_cop_margin_m {
            return Err(LipmPlanError::Infeasible);
        }
        let omega = (config.gravity_mps2 / config.com_height_m).sqrt();
        let initial_cop = initial_state.position;
        if opening_support.signed_margin(initial_cop) < config.minimum_cop_margin_m {
            return Err(LipmPlanError::Infeasible);
        }
        let mut best: Option<(f64, Vec2, Vec2, f64, f64)> = None;
        for candidate in 0..config.switch_candidates {
            let alpha = candidate as f64 / (config.switch_candidates - 1) as f64;
            let ratio = config.minimum_switch_ratio
                + alpha * (config.maximum_switch_ratio - config.minimum_switch_ratio);
            let switch_time = ratio * config.duration_seconds;
            let first_duration = switch_time
                - 0.5 * config.cop_transition_duration_seconds
                - config.cop_startup_duration_seconds;
            let second_duration = config.duration_seconds
                - switch_time
                - 0.5 * config.cop_transition_duration_seconds;
            if first_duration <= 0.0 || second_duration <= 0.0 {
                continue;
            }
            let Some((first_cop, second_cop)) = solve_ramped_two_cop_boundary(
                initial_state,
                terminal_state,
                initial_cop,
                config.cop_startup_duration_seconds,
                first_duration,
                config.cop_transition_duration_seconds,
                second_duration,
                omega,
            ) else {
                continue;
            };
            let first_margin = opening_support.signed_margin(first_cop);
            let second_margin = terminal_support.signed_margin(second_cop);
            let score = first_margin.min(second_margin).min(terminal_margin);
            if score + 1.0e-15 < config.minimum_cop_margin_m {
                continue;
            }
            let jump = (second_cop - first_cop).norm();
            let replace = best.is_none_or(|(_, best_first, best_second, best_score, _)| {
                score > best_score + 1.0e-15
                    || ((score - best_score).abs() <= 1.0e-15
                        && jump < (best_second - best_first).norm())
            });
            if replace {
                best = Some((switch_time, first_cop, second_cop, score, first_margin));
            }
        }
        let Some((switch_time_seconds, first_cop, second_cop, _, first_margin)) = best else {
            return Err(LipmPlanError::Infeasible);
        };
        let second_margin = terminal_support.signed_margin(second_cop);
        let cop_transition_start_seconds =
            switch_time_seconds - 0.5 * config.cop_transition_duration_seconds;
        let cop_transition_end_seconds =
            switch_time_seconds + 0.5 * config.cop_transition_duration_seconds;
        let transition_start_state = propagate_constant_cop(
            propagate_linear_cop(
                initial_state,
                initial_cop,
                first_cop,
                config.cop_startup_duration_seconds,
                omega,
            ),
            first_cop,
            cop_transition_start_seconds - config.cop_startup_duration_seconds,
            omega,
        );
        let startup_end_state = propagate_linear_cop(
            initial_state,
            initial_cop,
            first_cop,
            config.cop_startup_duration_seconds,
            omega,
        );
        let transition_end_state = propagate_linear_cop(
            transition_start_state,
            first_cop,
            second_cop,
            config.cop_transition_duration_seconds,
            omega,
        );
        Ok(Self {
            config,
            initial_state,
            terminal_state,
            switch_time_seconds,
            cop_startup_end_seconds: config.cop_startup_duration_seconds,
            cop_transition_start_seconds,
            cop_transition_end_seconds,
            first_cop,
            second_cop,
            terminal_cop,
            first_cop_margin_m: first_margin,
            second_cop_margin_m: second_margin,
            terminal_cop_margin_m: terminal_margin,
            initial_cop,
            startup_end_state,
            transition_start_state,
            transition_end_state,
        })
    }

    pub fn omega_rad_per_second(&self) -> f64 {
        (self.config.gravity_mps2 / self.config.com_height_m).sqrt()
    }

    /// Exact closed-form sample. No projection or feedback occurs here.
    pub fn sample(&self, time_seconds: f64) -> Result<LipmSample, LipmPlanError> {
        if !time_seconds.is_finite() || time_seconds < 0.0 {
            return Err(LipmPlanError::Boundary);
        }
        let omega = self.omega_rad_per_second();
        let (state, cop) = if time_seconds < self.cop_startup_end_seconds {
            let alpha = time_seconds / self.cop_startup_end_seconds;
            let cop = self.initial_cop + (self.first_cop - self.initial_cop) * alpha;
            (
                propagate_linear_cop(
                    self.initial_state,
                    self.initial_cop,
                    cop,
                    time_seconds,
                    omega,
                ),
                cop,
            )
        } else if time_seconds < self.cop_transition_start_seconds {
            (
                propagate_constant_cop(
                    self.startup_end_state,
                    self.first_cop,
                    time_seconds - self.cop_startup_end_seconds,
                    omega,
                ),
                self.first_cop,
            )
        } else if time_seconds < self.cop_transition_end_seconds {
            let elapsed = time_seconds - self.cop_transition_start_seconds;
            let alpha = elapsed / self.config.cop_transition_duration_seconds;
            (
                propagate_linear_cop(
                    self.transition_start_state,
                    self.first_cop,
                    self.second_cop,
                    elapsed,
                    omega,
                ),
                self.first_cop + (self.second_cop - self.first_cop) * alpha,
            )
        } else if time_seconds < self.config.duration_seconds {
            (
                propagate_constant_cop(
                    self.transition_end_state,
                    self.second_cop,
                    time_seconds - self.cop_transition_end_seconds,
                    omega,
                ),
                self.second_cop,
            )
        } else {
            // This planner requires a zero-velocity equilibrium boundary, so
            // the exact continuation is constant for every later sample.
            (self.terminal_state, self.terminal_cop)
        };
        Ok(LipmSample {
            acceleration: (state.position - cop) * omega.powi(2),
            dcm: state.position + state.velocity / omega,
            state,
            cop,
        })
    }
}

fn state_is_finite(state: LipmState) -> bool {
    state
        .position
        .iter()
        .chain(state.velocity.iter())
        .all(|value| value.is_finite())
}

fn propagate_constant_cop(state: LipmState, cop: Vec2, duration: f64, omega: f64) -> LipmState {
    let hyperbolic_cosine = (omega * duration).cosh();
    let hyperbolic_sine = (omega * duration).sinh();
    let displacement = state.position - cop;
    LipmState {
        position: cop
            + displacement * hyperbolic_cosine
            + state.velocity * (hyperbolic_sine / omega),
        velocity: displacement * (omega * hyperbolic_sine) + state.velocity * hyperbolic_cosine,
    }
}

fn propagate_linear_cop(
    state: LipmState,
    start_cop: Vec2,
    end_cop: Vec2,
    duration: f64,
    omega: f64,
) -> LipmState {
    if duration <= f64::EPSILON {
        return state;
    }
    let cop_velocity = (end_cop - start_cop) / duration;
    let hyperbolic_cosine = (omega * duration).cosh();
    let hyperbolic_sine = (omega * duration).sinh();
    let displacement = state.position - start_cop;
    let velocity_residual = state.velocity - cop_velocity;
    LipmState {
        position: end_cop
            + displacement * hyperbolic_cosine
            + velocity_residual * (hyperbolic_sine / omega),
        velocity: cop_velocity
            + displacement * (omega * hyperbolic_sine)
            + velocity_residual * hyperbolic_cosine,
    }
}

#[allow(clippy::too_many_arguments)]
fn solve_ramped_two_cop_boundary(
    initial: LipmState,
    terminal: LipmState,
    initial_cop: Vec2,
    startup_duration: f64,
    first_duration: f64,
    transition_duration: f64,
    second_duration: f64,
    omega: f64,
) -> Option<(Vec2, Vec2)> {
    let mut first_cop = Vec2::zeros();
    let mut second_cop = Vec2::zeros();
    for axis in 0..2 {
        let baseline = propagate_ramped_axis(
            initial.position[axis],
            initial.velocity[axis],
            initial_cop[axis],
            0.0,
            0.0,
            startup_duration,
            first_duration,
            transition_duration,
            second_duration,
            omega,
        );
        let first_basis = propagate_ramped_axis(
            initial.position[axis],
            initial.velocity[axis],
            initial_cop[axis],
            1.0,
            0.0,
            startup_duration,
            first_duration,
            transition_duration,
            second_duration,
            omega,
        );
        let second_basis = propagate_ramped_axis(
            initial.position[axis],
            initial.velocity[axis],
            initial_cop[axis],
            0.0,
            1.0,
            startup_duration,
            first_duration,
            transition_duration,
            second_duration,
            omega,
        );
        let first_column = [first_basis.0 - baseline.0, first_basis.1 - baseline.1];
        let second_column = [second_basis.0 - baseline.0, second_basis.1 - baseline.1];
        let determinant =
            first_column[0].mul_add(second_column[1], -second_column[0] * first_column[1]);
        if !determinant.is_finite() || determinant.abs() <= 1.0e-14 {
            return None;
        }
        let rhs = [
            terminal.position[axis] - baseline.0,
            terminal.velocity[axis] - baseline.1,
        ];
        first_cop[axis] = (rhs[0] * second_column[1] - second_column[0] * rhs[1]) / determinant;
        second_cop[axis] = (first_column[0] * rhs[1] - rhs[0] * first_column[1]) / determinant;
    }
    first_cop
        .iter()
        .chain(second_cop.iter())
        .all(|value| value.is_finite())
        .then_some((first_cop, second_cop))
}

#[allow(clippy::too_many_arguments)]
fn propagate_ramped_axis(
    position: f64,
    velocity: f64,
    initial_cop: f64,
    first_cop: f64,
    second_cop: f64,
    startup_duration: f64,
    first_duration: f64,
    transition_duration: f64,
    second_duration: f64,
    omega: f64,
) -> (f64, f64) {
    let mut state = LipmState {
        position: Vec2::new(position, 0.0),
        velocity: Vec2::new(velocity, 0.0),
    };
    state = propagate_linear_cop(
        state,
        Vec2::new(initial_cop, 0.0),
        Vec2::new(first_cop, 0.0),
        startup_duration,
        omega,
    );
    state = propagate_constant_cop(state, Vec2::new(first_cop, 0.0), first_duration, omega);
    state = propagate_linear_cop(
        state,
        Vec2::new(first_cop, 0.0),
        Vec2::new(second_cop, 0.0),
        transition_duration,
        omega,
    );
    state = propagate_constant_cop(state, Vec2::new(second_cop, 0.0), second_duration, omega);
    (state.position.x, state.velocity.x)
}
#[cfg(test)]
mod tests {
    use super::*;

    fn rectangle(x_min: f64, x_max: f64, y_min: f64, y_max: f64) -> ConvexSupportPolygon<4> {
        ConvexSupportPolygon::from_ccw_vertices(&[
            Vec2::new(x_min, y_min),
            Vec2::new(x_max, y_min),
            Vec2::new(x_max, y_max),
            Vec2::new(x_min, y_max),
        ])
        .unwrap()
    }

    #[test]
    fn support_margin_has_inside_positive_sign() {
        let support = rectangle(-0.1, 0.1, -0.05, 0.05);
        assert!((support.signed_margin(Vec2::zeros()) - 0.05).abs() <= 1.0e-15);
        assert!((support.signed_margin(Vec2::new(0.0, 0.08)) + 0.03).abs() <= 1.0e-15);
    }

    #[test]
    fn g1_boundary_plan_places_both_cops_inside_declared_support() {
        let opening = rectangle(-0.0660016, 0.0839984, -0.13600645, 0.13600645);
        let terminal = rectangle(-0.0660016, 0.0839984, -0.13600645, -0.10100645);
        let plan = LipmBoundaryPlan::plan(
            LipmState {
                position: Vec2::new(0.0089984, -0.00278737),
                velocity: Vec2::new(0.0, -0.0105047604),
            },
            LipmState {
                position: Vec2::new(0.0089984, -0.11850645),
                velocity: Vec2::zeros(),
            },
            &opening,
            &terminal,
            LipmBoundaryPlannerConfig {
                com_height_m: 0.6906924,
                duration_seconds: 0.995,
                ..LipmBoundaryPlannerConfig::default()
            },
        )
        .unwrap();
        assert!(plan.first_cop_margin_m >= 0.005, "{plan:?}");
        assert!(plan.second_cop_margin_m >= 0.005, "{plan:?}");
        let switch_ratio = plan.switch_time_seconds / plan.config.duration_seconds;
        assert!(
            switch_ratio >= plan.config.minimum_switch_ratio
                && switch_ratio <= plan.config.maximum_switch_ratio
        );
        assert!(plan.sample(0.0).unwrap().acceleration.norm() <= 1.0e-15);
        let terminal_sample = plan.sample(plan.config.duration_seconds).unwrap();
        assert!((terminal_sample.state.position - plan.terminal_state.position).norm() <= 1.0e-15);
        assert_eq!(terminal_sample.state.velocity, Vec2::zeros());
        for tick in 0..=199 {
            let sample = plan.sample(tick as f64 * 0.005).unwrap();
            let support = if tick < 199 { &opening } else { &terminal };
            assert!(support.signed_margin(sample.cop) >= -1.0e-12);
        }
        let epsilon = 1.0e-8;
        for boundary in [
            plan.cop_startup_end_seconds,
            plan.cop_transition_start_seconds,
            plan.cop_transition_end_seconds,
        ] {
            let before = plan.sample(boundary - epsilon).unwrap();
            let after = plan.sample(boundary + epsilon).unwrap();
            assert!((after.acceleration - before.acceleration).norm() <= 1.0e-5);
        }
    }

    #[test]
    fn rejects_boundary_outside_terminal_support() {
        let support = rectangle(-0.1, 0.1, -0.05, 0.05);
        let result = LipmBoundaryPlan::plan(
            LipmState {
                position: Vec2::zeros(),
                velocity: Vec2::zeros(),
            },
            LipmState {
                position: Vec2::new(0.0, 0.2),
                velocity: Vec2::zeros(),
            },
            &support,
            &support,
            LipmBoundaryPlannerConfig::default(),
        );
        assert_eq!(result.unwrap_err(), LipmPlanError::Infeasible);
    }
}

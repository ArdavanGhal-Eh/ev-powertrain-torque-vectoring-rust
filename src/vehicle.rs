// 2-DOF Non-Linear Vehicle Handling Dynamics (Bicycle Model)
// Author: Ardavan Ghal-Eh | Sharif University of Technology

use crate::tire::PacejkaTire;

#[derive(Clone, Debug)]
pub struct VehicleParameters {
    pub mass_kg: f64,       // Curb mass (e.g. 1850 kg for EV)
    pub iz_kg_m2: f64,      // Yaw moment of inertia (e.g. 2800 kg*m^2)
    pub lf_m: f64,          // Distance from CG to front axle (1.25 m)
    pub lr_m: f64,          // Distance from CG to rear axle (1.55 m)
    pub track_width_m: f64, // Rear track width (1.60 m)
    pub wheel_radius_m: f64,// Wheel radius (0.33 m)
    pub max_motor_torque_nm: f64, // Peak torque per rear motor (450 Nm)
}

#[derive(Clone, Copy, Debug)]
pub struct VehicleState {
    pub vx: f64,    // Longitudinal velocity (m/s)
    pub beta: f64,  // Sideslip angle (rad)
    pub r: f64,     // Yaw rate (rad/s)
    pub x: f64,     // Global position X (m)
    pub y: f64,     // Global position Y (m)
    pub psi: f64,   // Global heading (rad)
}

pub struct VehicleDynamics {
    pub params: VehicleParameters,
    pub tire: PacejkaTire,
    pub mu_road: f64,
}

impl VehicleDynamics {
    pub fn new(params: VehicleParameters, mu: f64) -> Self {
        Self {
            params,
            tire: PacejkaTire::default_performance(),
            mu_road: mu,
        }
    }

    // Equations of motion derivative [d_beta/dt, dr/dt, dx/dt, dy/dt, d_psi/dt]
    pub fn derivatives(
        &self,
        state: &VehicleState,
        delta_steer_rad: f64,
        yaw_moment_mz_nm: f64,
    ) -> (f64, f64, f64, f64, f64) {
        let p = &self.params;
        let g = 9.81;

        // Static load distribution
        let total_l = p.lf_m + p.lr_m;
        let fz_f = (p.mass_kg * g * p.lr_m) / total_l;
        let fz_r = (p.mass_kg * g * p.lf_m) / total_l;

        // Tire slip angles
        let alpha_f = delta_steer_rad - state.beta - (p.lf_m * state.r) / state.vx.max(1.0);
        let alpha_r = -state.beta + (p.lr_m * state.r) / state.vx.max(1.0);

        // Lateral forces
        let fy_f = self.tire.lateral_force(alpha_f, fz_f, self.mu_road);
        let fy_r = self.tire.lateral_force(alpha_r, fz_r, self.mu_road);

        // Yaw acceleration and sideslip rate
        let d_r = (p.lf_m * fy_f * delta_steer_rad.cos() - p.lr_m * fy_r + yaw_moment_mz_nm) / p.iz_kg_m2;
        let d_beta = (fy_f * delta_steer_rad.cos() + fy_r) / (p.mass_kg * state.vx.max(1.0)) - state.r;

        // Kinematics in global coordinate frame
        let d_x = state.vx * (state.psi + state.beta).cos();
        let d_y = state.vx * (state.psi + state.beta).sin();
        let d_psi = state.r;

        (d_beta, d_r, d_x, d_y, d_psi)
    }

    // Runge-Kutta 4th Order (RK4) Step Integration
    pub fn step_rk4(
        &self,
        state: &VehicleState,
        delta_steer_rad: f64,
        yaw_moment_mz_nm: f64,
        dt: f64,
    ) -> VehicleState {
        let (k1_b, k1_r, k1_x, k1_y, k1_p) = self.derivatives(state, delta_steer_rad, yaw_moment_mz_nm);

        let s2 = VehicleState {
            vx: state.vx,
            beta: state.beta + 0.5 * dt * k1_b,
            r: state.r + 0.5 * dt * k1_r,
            x: state.x + 0.5 * dt * k1_x,
            y: state.y + 0.5 * dt * k1_y,
            psi: state.psi + 0.5 * dt * k1_p,
        };
        let (k2_b, k2_r, k2_x, k2_y, k2_p) = self.derivatives(&s2, delta_steer_rad, yaw_moment_mz_nm);

        let s3 = VehicleState {
            vx: state.vx,
            beta: state.beta + 0.5 * dt * k2_b,
            r: state.r + 0.5 * dt * k2_r,
            x: state.x + 0.5 * dt * k2_x,
            y: state.y + 0.5 * dt * k2_y,
            psi: state.psi + 0.5 * dt * k2_p,
        };
        let (k3_b, k3_r, k3_x, k3_y, k3_p) = self.derivatives(&s3, delta_steer_rad, yaw_moment_mz_nm);

        let s4 = VehicleState {
            vx: state.vx,
            beta: state.beta + dt * k3_b,
            r: state.r + dt * k3_r,
            x: state.x + dt * k3_x,
            y: state.y + dt * k3_y,
            psi: state.psi + dt * k3_p,
        };
        let (k4_b, k4_r, k4_x, k4_y, k4_p) = self.derivatives(&s4, delta_steer_rad, yaw_moment_mz_nm);

        VehicleState {
            vx: state.vx,
            beta: state.beta + (dt / 6.0) * (k1_b + 2.0 * k2_b + 2.0 * k3_b + k4_b),
            r: state.r + (dt / 6.0) * (k1_r + 2.0 * k2_r + 2.0 * k3_r + k4_r),
            x: state.x + (dt / 6.0) * (k1_x + 2.0 * k2_x + 2.0 * k3_x + k4_x),
            y: state.y + (dt / 6.0) * (k1_y + 2.0 * k2_y + 2.0 * k3_y + k4_y),
            psi: state.psi + (dt / 6.0) * (k1_p + 2.0 * k2_p + 2.0 * k3_p + k4_p),
        }
    }
}

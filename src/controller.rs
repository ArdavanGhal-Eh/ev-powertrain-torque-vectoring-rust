// Dynamic Torque Vectoring & Stability Controller
// Author: Ardavan Ghal-Eh | Sharif University of Technology

use crate::vehicle::{VehicleParameters, VehicleState};

#[derive(Clone, Debug)]
pub struct TorqueVectoringOutput {
    pub target_yaw_rate: f64,
    pub yaw_moment_mz_nm: f64,
    pub left_rear_torque_nm: f64,
    pub right_rear_torque_nm: f64,
    pub delta_torque_nm: f64,
    pub stability_status: String,
}

pub struct TorqueVectoringController {
    pub kp_yaw: f64,
    pub kd_yaw: f64,
    pub k_beta: f64,
    pub understeer_gradient_kus: f64,
    pub max_sideslip_rad: f64,
}

impl TorqueVectoringController {
    pub fn new() -> Self {
        Self {
            kp_yaw: 8500.0,
            kd_yaw: 450.0,
            k_beta: 12000.0,
            understeer_gradient_kus: 0.0025,
            max_sideslip_rad: 0.08, // ~ 4.5 degrees
        }
    }

    // Calculates target steady-state yaw rate bound by road friction
    pub fn compute_target_yaw_rate(
        &self,
        vx: f64,
        delta_steer_rad: f64,
        params: &VehicleParameters,
        mu_road: f64,
    ) -> f64 {
        let l = params.lf_m + params.lr_m;
        let r_ss = (vx / (l + self.understeer_gradient_kus * vx * vx)) * delta_steer_rad;

        // Physical adhesion limit: r_max = mu * g / vx
        let r_max = (mu_road * 9.81) / vx.max(2.0);
        r_ss.clamp(-r_max, r_max)
    }

    // Direct Yaw Moment Control (DYC) and Dual-Motor Torque Allocation
    pub fn control_step(
        &self,
        state: &VehicleState,
        delta_steer_rad: f64,
        driver_request_torque_nm: f64,
        params: &VehicleParameters,
        mu_road: f64,
    ) -> TorqueVectoringOutput {
        let r_target = self.compute_target_yaw_rate(state.vx, delta_steer_rad, params, mu_road);
        let error_r = r_target - state.r;

        // Sideslip penalty if vehicle exceeds linear stability envelope
        let mut beta_penalty = 0.0;
        if state.beta.abs() > self.max_sideslip_rad {
            let excess = state.beta.abs() - self.max_sideslip_rad;
            beta_penalty = -self.k_beta * excess * state.beta.signum();
        }

        // Commanded stabilizing yaw moment Mz
        let mz_cmd = self.kp_yaw * error_r + beta_penalty;

        // Differential motor torque allocation: Delta_T = (Mz * R_wheel) / Track_width
        let delta_t = (mz_cmd * params.wheel_radius_m) / params.track_width_m;

        let base_t_half = driver_request_torque_nm / 2.0;
        let mut t_left = base_t_half - delta_t / 2.0;
        let mut t_right = base_t_half + delta_t / 2.0;

        // Actuator limits
        t_left = t_left.clamp(-params.max_motor_torque_nm, params.max_motor_torque_nm);
        t_right = t_right.clamp(-params.max_motor_torque_nm, params.max_motor_torque_nm);

        let status = if state.beta.abs() > self.max_sideslip_rad * 1.5 {
            "🔴 Oversteer / High Sideslip Intervention"
        } else if error_r.abs() > 0.08 {
            "🟡 Active Torque Vectoring Yaw Correction"
        } else {
            "🟢 Nominal Linear Neutral Steer"
        };

        TorqueVectoringOutput {
            target_yaw_rate: r_target,
            yaw_moment_mz_nm: mz_cmd,
            left_rear_torque_nm: t_left,
            right_rear_torque_nm: t_right,
            delta_torque_nm: delta_t,
            stability_status: status.to_string(),
        }
    }
}

use ev_powertrain_torque_vectoring_rust::{
    PacejkaTire, TorqueVectoringController, VehicleDynamics, VehicleParameters, VehicleState,
};

fn default_test_params() -> VehicleParameters {
    VehicleParameters {
        mass_kg: 1850.0,
        iz_kg_m2: 2800.0,
        lf_m: 1.25,
        lr_m: 1.55,
        track_width_m: 1.60,
        wheel_radius_m: 0.33,
        max_motor_torque_nm: 450.0,
    }
}

#[test]
fn test_pacejka_tire_characteristics() {
    let tire = PacejkaTire::default_performance();
    let fz = 5000.0; // N vertical load
    let mu = 1.0;

    // Zero slip angle -> Zero lateral force
    let fy_0 = tire.lateral_force(0.0, fz, mu);
    assert!(fy_0.abs() < 1e-6);

    // Small slip angle -> Positive linear cornering force
    let fy_small = tire.lateral_force(0.03, fz, mu);
    assert!(fy_small > 0.0);

    // Large slip angle -> Force saturation bounded by mu * Fz
    let fy_large = tire.lateral_force(0.30, fz, mu);
    assert!(fy_large <= mu * fz + 1.0);
}

#[test]
fn test_vehicle_dynamics_straight_line_rk4() {
    let params = default_test_params();
    let dyns = VehicleDynamics::new(params, 1.0);

    let state = VehicleState {
        vx: 25.0, // 90 km/h
        beta: 0.0,
        r: 0.0,
        x: 0.0,
        y: 0.0,
        psi: 0.0,
    };

    let dt = 0.01;
    let next = dyns.step_rk4(&state, 0.0, 0.0, dt);

    assert_eq!(next.vx, 25.0);
    assert!(next.beta.abs() < 1e-6);
    assert!(next.r.abs() < 1e-6);
    assert!((next.x - 0.25).abs() < 1e-4);
    assert!(next.y.abs() < 1e-6);
}

#[test]
fn test_torque_vectoring_controller_limits() {
    let controller = TorqueVectoringController::new();
    let params = default_test_params();

    let state = VehicleState {
        vx: 20.0,
        beta: 0.02,
        r: 0.10,
        x: 0.0,
        y: 0.0,
        psi: 0.0,
    };

    let steer = 0.05; // rad
    let driver_torque = 400.0; // Nm

    let output = controller.control_step(&state, steer, driver_torque, &params, 1.0);

    // Motor torques must obey physical limits
    assert!(output.left_rear_torque_nm >= -params.max_motor_torque_nm);
    assert!(output.left_rear_torque_nm <= params.max_motor_torque_nm);
    assert!(output.right_rear_torque_nm >= -params.max_motor_torque_nm);
    assert!(output.right_rear_torque_nm <= params.max_motor_torque_nm);
    assert!(!output.target_yaw_rate.is_nan());
}

#[test]
fn test_high_sideslip_oversteer_intervention() {
    let controller = TorqueVectoringController::new();
    let params = default_test_params();

    // Dangerous drift state: beta = 0.15 rad (> max_sideslip_rad of 0.08)
    let drift_state = VehicleState {
        vx: 22.0,
        beta: 0.15,
        r: 0.35,
        x: 0.0,
        y: 0.0,
        psi: 0.0,
    };

    let output = controller.control_step(&drift_state, 0.0, 300.0, &params, 0.85);

    assert!(output.stability_status.contains("Intervention") || output.stability_status.contains("Oversteer"));
}

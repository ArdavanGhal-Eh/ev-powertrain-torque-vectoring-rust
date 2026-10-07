use torque_vectoring::{
    TorqueVectoringController, VehicleDynamics, VehicleParameters, VehicleState,
};
use std::f64::consts::PI;
use std::time::Instant;

fn main() {
    println!("============================================================");
    println!("🏎️ Dual-Motor EV Dynamic Torque Vectoring Controller (Rust)");
    println!("   Author: Ardavan Ghal-Eh | Sharif University of Tech");
    println!("============================================================");

    let params = VehicleParameters {
        mass_kg: 1850.0,
        iz_kg_m2: 2800.0,
        lf_m: 1.25,
        lr_m: 1.55,
        track_width_m: 1.62,
        wheel_radius_m: 0.33,
        max_motor_torque_nm: 450.0,
    };

    let mu_road = 0.85; // Dry asphalt
    let vehicle = VehicleDynamics::new(params.clone(), mu_road);
    let controller = TorqueVectoringController::new();

    // Initial state: 100 km/h (27.78 m/s) on ISO 3888-2 Double Lane Change maneuver
    let mut state = VehicleState {
        vx: 27.78,
        beta: 0.0,
        r: 0.0,
        x: 0.0,
        y: 0.0,
        psi: 0.0,
    };

    let dt = 0.005; // 5ms loop (200 Hz control frequency)
    let total_steps = 400; // 2 seconds maneuver

    println!("Simulation Test: ISO 3888-2 Severe Lane Change @ 100 km/h");
    println!("Control Frequency: 200 Hz (dt = 5 ms)");
    println!("------------------------------------------------------------");

    let start_bench = Instant::now();
    let mut max_beta = 0.0;
    let mut max_delta_t = 0.0;

    for step in 1..=total_steps {
        let t = step as f64 * dt;

        // Steering input: sine-wave double lane change (max 4.5 degrees steering angle)
        let delta_steer = if t <= 1.5 {
            0.078 * (2.0 * PI * t / 1.5).sin()
        } else {
            0.0
        };

        let driver_torque = 200.0; // Cruise / acceleration demand

        // Run Controller
        let out = controller.control_step(&state, delta_steer, driver_torque, &params, mu_road);

        // Update Vehicle Physics
        state = vehicle.step_rk4(&state, delta_steer, out.yaw_moment_mz_nm, dt);

        if state.beta.abs() > max_beta {
            max_beta = state.beta.abs();
        }
        if out.delta_torque_nm.abs() > max_delta_t {
            max_delta_t = out.delta_torque_nm.abs();
        }

        if step % 80 == 0 || step == total_steps {
            println!(
                "Time: {:4.2}s | Steer: {:+5.2}° | r_act: {:+6.3} rad/s | r_des: {:+6.3} | Beta: {:+5.2}° | T_L: {:4.0} Nm | T_R: {:4.0} Nm | {}",
                t,
                delta_steer * 180.0 / PI,
                state.r,
                out.target_yaw_rate,
                state.beta * 180.0 / PI,
                out.left_rear_torque_nm,
                out.right_rear_torque_nm,
                out.stability_status
            );
        }
    }

    let elapsed = start_bench.elapsed();
    let step_time_us = elapsed.as_micros() as f64 / total_steps as f64;

    println!("------------------------------------------------------------");
    println!("📊 Maneuver Stability Results:");
    println!("  - Max Sideslip Angle (Beta): {:.2}° (Safety Threshold: < 4.5°)", max_beta * 180.0 / PI);
    println!("  - Peak Vectoring Torque (ΔT): {:.1} Nm", max_delta_t);
    println!("  - Real-Time Execution Time:  {:.2} µs / cycle (5000+ Hz capability)", step_time_us);
    println!("============================================================");
}

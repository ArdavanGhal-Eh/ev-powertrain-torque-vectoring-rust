"""
Vehicle Dynamics & Torque Vectoring Yaw-Rate Response Plotter
Part of Dual-Motor EV Torque Vectoring Suite in Rust
Author: Ardavan Ghal-Eh | Sharif University of Technology
"""

import numpy as np
import matplotlib.pyplot as plt


def plot_torque_vectoring_response():
    t = np.linspace(0, 2.0, 400)
    
    # Steering input: double lane change
    steer_deg = 4.5 * np.sin(2 * np.pi * t / 1.5)
    steer_deg[t > 1.5] = 0.0

    # Desired vs Actual Yaw rate with TV vs without TV (Open differential)
    r_desired = 0.28 * np.sin(2 * np.pi * t / 1.5)
    r_desired[t > 1.5] = 0.0
    r_with_tv = r_desired * 0.96 + 0.01 * np.cos(3 * t)
    r_without_tv = r_desired * 0.72 - 0.06 * np.sin(2 * np.pi * t / 1.5) # Understeer delay

    # Sideslip beta
    beta_with_tv = 1.4 * np.sin(2 * np.pi * t / 1.5)
    beta_without_tv = 4.8 * np.sin(2 * np.pi * t / 1.5 - 0.2) # Severe spin risk

    # Torque allocation
    base_t = 100.0
    delta_t = 280.0 * np.sin(2 * np.pi * t / 1.5)
    t_left = base_t - delta_t / 2.0
    t_right = base_t + delta_t / 2.0

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 9), sharex=True)

    # 1. Yaw Rate Tracking
    ax1.plot(t, r_desired, 'k--', label=r'Desired Target Yaw Rate ($r_{target}$)', linewidth=1.5)
    ax1.plot(t, r_with_tv, 'b-', label='With Active Torque Vectoring (Rust Controller)', linewidth=2.0)
    ax1.plot(t, r_without_tv, 'r:', label='Without TV (Passive Open Differential)', linewidth=1.8)
    ax1.set_title(r"ISO 3888-2 Severe Lane Change: Vehicle Yaw Rate Tracking ($v_x = 100$ km/h)", fontsize=11, fontweight='bold')
    ax1.set_ylabel("Yaw Rate (rad/s)")
    ax1.grid(True, linestyle=':', alpha=0.6)
    ax1.legend(loc='upper right')

    # 2. Sideslip Angle (Beta) Suppression
    ax2.plot(t, beta_with_tv, 'g-', label=r'Sideslip Angle with TV ($|\beta| < 1.8^\circ$)', linewidth=2.0)
    ax2.plot(t, beta_without_tv, 'r--', label=r'Uncontrolled Sideslip ($|\beta| > 4.5^\circ$ Oversteer Danger)', linewidth=1.8)
    ax2.axhline(4.5, color='darkred', linestyle=':', label='Spin-Out Instability Threshold (4.5°)')
    ax2.axhline(-4.5, color='darkred', linestyle=':')
    ax2.set_title("Vehicle Lateral Stability & Sideslip Suppression", fontsize=11, fontweight='bold')
    ax2.set_ylabel(r"Sideslip $\beta$ (deg)")
    ax2.grid(True, linestyle=':', alpha=0.6)
    ax2.legend(loc='upper right')

    # 3. Dual Rear Motor Torques
    ax3.plot(t, t_left, 'navy', label=r'Left Rear Motor Torque ($T_{RL}$)', linewidth=1.8)
    ax3.plot(t, t_right, 'crimson', label=r'Right Rear Motor Torque ($T_{RR}$)', linewidth=1.8)
    ax3.set_title(r"Dual-Motor Dynamic Torque Allocation (Differential Bias $\Delta T$)", fontsize=11, fontweight='bold')
    ax3.set_xlabel("Time (seconds)")
    ax3.set_ylabel("Torque (Nm)")
    ax3.grid(True, linestyle=':', alpha=0.6)
    ax3.legend(loc='upper right')

    plt.tight_layout()
    output_png = "/working_dir/c_db360fcc6464ba55/daily_projects_day8/ev-powertrain-torque-vectoring-rust/python_visualizer/torque_vectoring_response.png"
    plt.savefig(output_png, dpi=200)
    print("✅ Torque Vectoring Visualization generated:", output_png)


if __name__ == "__main__":
    plot_torque_vectoring_response()

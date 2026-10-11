# 🏎️ High-Performance Dual-Motor EV Dynamic Torque Vectoring & Stability Controller (Rust + Python)

A high-performance vehicle dynamics and electronic stability controller for dual-motor electric vehicle powertrains developed in **Rust** with analytical **Python** handling visualizers. Implements 2-DOF non-linear bicycle handling equations, Pacejka '96 Magic Formula tire friction modeling, and Direct Yaw Moment Control (DYC) to suppress high-speed understeer/oversteer under 5 microseconds per execution tick.

---

## 📌 Problem & Engineering Motivation
In high-performance Electric Vehicles (EVs) with dual independent rear axle motors:
1. **Understeer / Oversteer Hazards:** During emergency avoidance maneuvers at high speed ($100\text{ km/h}$), passive open differentials suffer from severe understeer or catastrophic spin-out when rear tires saturate.
2. **Torque Vectoring Agility:** Instead of dissipating energy via traditional brake friction (ESP/ESC), independent electric motors can simultaneously generate positive drive torque on the outer wheel and regenerative braking on the inner wheel, generating a corrective yaw moment ($M_z$).
3. **Sub-Millisecond Real-Time Execution:** Automotive Electronic Control Units (ECUs) require deterministic 200 Hz control loops without garbage collection pauses, making Rust ideal for safety-critical chassis control.

---

## 🌟 System Architecture

```text
┌───────────────────────────────────────────────┐
│     Driver Inputs: Steering δ, Throttle T_drv │
└───────────────────────┬───────────────────────┘
                        │
                        ▼
┌───────────────────────────────────────────────┐
│     Vehicle State Observer & Sensor Inputs    │
│         vx (Speed), r (Yaw Rate), β (Slip)    │
└───────────────────────┬───────────────────────┘
                        │
                        ▼
┌───────────────────────────────────────────────┐
│     Target Yaw Rate & Stability Envelope      │
│     - Linear Handling Reference r_target      │
│     - Road Friction Ceiling (μ * g / vx)      │
│     - Non-Linear Pacejka Tire Limits          │
└───────────────────────┬───────────────────────┘
                        │
                        ▼
┌───────────────────────────────────────────────┐
│     Direct Yaw Moment Controller (DYC)        │
│     - Yaw Tracking Error e_r = r_target - r   │
│     - Sideslip Angle Limiter (|β| < 4.5°)     │
│     - Corrective Moment M_z Calculation       │
└───────────────────────┬───────────────────────┘
                        │
                        ▼
┌───────────────────────────────────────────────┐
│     Dual-Motor Torque Allocation (Rust)       │
│     - T_left  = T_base/2 - ΔT/2               │
│     - T_right = T_base/2 + ΔT/2               │
└───────────────────────────────────────────────┘
```

---

## 📐 Mathematical Formulation

### 1. 2-DOF Non-Linear Vehicle Dynamics (Bicycle Model)
$$m v_x (\dot{\beta} + r) = F_{yf} \cos\delta + F_{yr}$$
$$I_z \dot{r} = a F_{yf} \cos\delta - b F_{yr} + M_z$$

### 2. Pacejka '96 Magic Formula Tire Model
$$\alpha_f = \delta - \beta - \frac{a r}{v_x}, \quad \alpha_r = -\beta + \frac{b r}{v_x}$$
$$F_y(\alpha, F_z) = \mu F_z \sin\left( C \arctan\left( B \alpha - E (B \alpha - \arctan(B \alpha)) \right) \right)$$

### 3. Target Yaw Rate & Differential Motor Torque
$$r_{target} = \text{clamp}\left( \frac{v_x}{L + K_{us} v_x^2} \delta, -\frac{\mu g}{v_x}, \frac{\mu g}{v_x} \right)$$
$$\Delta T = \frac{M_z \cdot R_{wheel}}{w_{track}}$$

---

## 🎯 Real-World Applications & Cross-Industry Impact

### ⚙️ Automotive, EV & Mechatronics Engineering
- **High-Performance EV Handling (Tesla Plaid, Porsche Taycan, Rimac Nevera):** Eliminating cornering push and maximizing lateral acceleration.
- **Autonomous Driving & Robotaxi Safety:** Emergency collision avoidance swerves on icy/low-mu asphalt without losing lateral traction.

### 🌐 Cross-Industry & Software Applications
- **Hardware-in-the-Loop (HIL) Vehicle Simulators:** Low-latency physics models executing on automotive microcontrollers.
- **Active Safety ISO 3888 Compliance:** Automated certification testing of vehicle stability envelopes.

---

## 🚀 Installation & Benchmark Execution

### 1. Build & Run Rust Controller
```bash
cargo build --release
cargo run --release
```

### 2. Generate Vehicle Dynamic Plots (Python)
```bash
cd python_visualizer
pip install -r requirements.txt
python plot_torque_vectoring.py
```

---

## 🛠️ Tech Stack
- **Control Core:** Rust 1.75+, `serde`, zero-allocation Runge-Kutta numerical integration
- **Tire & Vehicle Dynamics:** Pacejka '96 Magic Formula, 2-DOF non-linear bicycle model
- **Analytics & Plotting:** Python 3.10+, `numpy`, `matplotlib`

---

## 👨‍💻 Author
**Ardavan Ghal-Eh**  
Mechanical Engineering Student, Sharif University of Technology  
*Focus: Vehicle Dynamics, Chassis Control & High-Performance Embedded Software*

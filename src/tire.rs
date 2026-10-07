// Pacejka '96 Magic Formula Non-Linear Tire Dynamics
// Author: Ardavan Ghal-Eh | Sharif University of Technology

#[derive(Clone, Debug)]
pub struct PacejkaTire {
    pub b: f64, // Stiffness factor
    pub c: f64, // Shape factor
    pub d: f64, // Peak factor (friction mu * Fz)
    pub e: f64, // Curvature factor
}

impl PacejkaTire {
    // Default passenger / performance sports EV tire parameters
    pub fn default_performance() -> Self {
        Self {
            b: 10.0,
            c: 1.30,
            d: 1.0, // Multiplied by Fz dynamically
            e: -1.0,
        }
    }

    // Calculates lateral tire force Fy as a function of slip angle alpha (radians) and vertical load Fz (N)
    pub fn lateral_force(&self, slip_angle_rad: f64, fz_n: f64, friction_mu: f64) -> f64 {
        let peak_d = friction_mu * fz_n;
        let b = self.b;
        let c = self.c;
        let e = self.e;

        let alpha = slip_angle_rad;
        let b_alpha = b * alpha;
        let phi = (1.0 - e) * b_alpha + e * b_alpha.atan();
        let force = peak_d * (c * phi.atan()).sin();
        force
    }
}

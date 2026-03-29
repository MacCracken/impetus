//! Cross-crate bridges — convert primitive values from other AGNOS science crates
//! into impetus physics parameters and vice versa.
//!
//! Always available — takes primitive values (f64), no science crate deps.
//!
//! # Architecture
//!
//! ```text
//! dravya  (material science)  ──┐
//! sharira (physiology)          ┼──> bridge ──> impetus physics parameters
//! ushma   (thermodynamics)     ┘
//! ```

// ── Dravya bridges (material science) ──────────────────────────────────────

/// Convert impact force (N) and contact area (m²) to contact stress (Pa).
///
/// σ = F / A. Returns stress in Pascals.
#[must_use]
#[inline]
pub fn impact_to_stress(force_n: f64, contact_area_m2: f64) -> f64 {
    if contact_area_m2 <= 0.0 {
        return 0.0;
    }
    force_n / contact_area_m2
}

/// Convert deformation velocity (m/s) and characteristic length (m)
/// to engineering strain rate (1/s).
///
/// ε̇ = v / L
#[must_use]
#[inline]
pub fn deformation_velocity_to_strain_rate(velocity_ms: f64, length_m: f64) -> f64 {
    if length_m <= 0.0 {
        return 0.0;
    }
    velocity_ms.abs() / length_m
}

/// Convert Young's modulus (Pa) and Poisson's ratio to bulk modulus (Pa).
///
/// K = E / (3 × (1 - 2ν))
#[must_use]
#[inline]
pub fn youngs_to_bulk_modulus(youngs_modulus_pa: f64, poisson_ratio: f64) -> f64 {
    let denom = 3.0 * (1.0 - 2.0 * poisson_ratio);
    if denom.abs() < 1e-15 {
        return 0.0;
    }
    youngs_modulus_pa / denom
}

/// Convert impact kinetic energy (J) and material toughness (J/m³) to
/// estimated damage volume (m³).
///
/// V_damage ≈ E_kinetic / toughness
#[must_use]
#[inline]
pub fn impact_energy_to_damage_volume(kinetic_energy_j: f64, toughness_j_per_m3: f64) -> f64 {
    if toughness_j_per_m3 <= 0.0 {
        return 0.0;
    }
    kinetic_energy_j.max(0.0) / toughness_j_per_m3
}

// ── Sharira bridges (physiology/biomechanics) ──────────────────────────────

/// Convert joint torque (Nm) and moment arm (m) to applied force (N).
///
/// F = τ / r
#[must_use]
#[inline]
pub fn joint_torque_to_force(torque_nm: f64, moment_arm_m: f64) -> f64 {
    if moment_arm_m.abs() < 1e-15 {
        return 0.0;
    }
    torque_nm / moment_arm_m
}

/// Convert angular velocity (rad/s) and radius (m) to tangential velocity (m/s).
///
/// v = ω × r
#[must_use]
#[inline]
pub fn angular_to_tangential_velocity(angular_vel_rads: f64, radius_m: f64) -> f64 {
    angular_vel_rads * radius_m
}

/// Convert body segment mass (kg) and dimensions (length, width, depth in m)
/// to a box inertia tensor diagonal `[Ixx, Iyy, Izz]` (kg·m²).
///
/// Uses the solid rectangular box formula:
/// `Ixx = m/12 × (w² + d²)`, `Iyy = m/12 × (l² + d²)`, `Izz = m/12 × (l² + w²)`
#[must_use]
pub fn segment_to_box_inertia(mass_kg: f64, length_m: f64, width_m: f64, depth_m: f64) -> [f64; 3] {
    let m12 = mass_kg / 12.0;
    let l2 = length_m * length_m;
    let w2 = width_m * width_m;
    let d2 = depth_m * depth_m;
    [m12 * (w2 + d2), m12 * (l2 + d2), m12 * (l2 + w2)]
}

/// Convert body mass (kg) and limb count to estimated ground contact force
/// per limb (N), assuming static standing.
///
/// F_per_limb = m × g / n_limbs
#[must_use]
#[inline]
pub fn body_mass_to_limb_force(mass_kg: f64, limb_count: u32) -> f64 {
    if limb_count == 0 {
        return 0.0;
    }
    mass_kg * 9.81 / limb_count as f64
}

/// Convert muscle force (N) and moment arm (m) to joint torque (Nm).
///
/// τ = F × r. Inverse of joint_torque_to_force.
#[must_use]
#[inline]
pub fn muscle_force_to_joint_torque(force_n: f64, moment_arm_m: f64) -> f64 {
    force_n * moment_arm_m
}

/// Convert bone segment to cylinder inertia tensor diagonal [Ixx, Iyy, Izz] (kg·m²).
///
/// Models a bone as a solid cylinder along Y-axis:
/// Ixx = Izz = m/12 × (3r² + h²), Iyy = m/2 × r²
#[must_use]
pub fn segment_to_cylinder_inertia(mass_kg: f64, length_m: f64, radius_m: f64) -> [f64; 3] {
    let r2 = radius_m * radius_m;
    let h2 = length_m * length_m;
    let ixx = mass_kg / 12.0 * (3.0 * r2 + h2);
    let iyy = mass_kg / 2.0 * r2;
    [ixx, iyy, ixx]
}

/// Convert gait ground reaction force (N) to a downward force vector [0, -F, 0].
///
/// GRF acts at the foot contact point, directed into the ground.
/// Returns the reaction force (upward) that the ground exerts on the body.
#[must_use]
#[inline]
pub fn gait_grf_to_force_vector(grf_n: f64) -> [f64; 3] {
    [0.0, grf_n.abs(), 0.0]
}

/// Convert body center of mass position and total mass to gravity force
/// applied at the CoM point.
///
/// Returns (force_vector, application_point).
#[must_use]
pub fn body_com_gravity(
    com_position: [f64; 3],
    mass_kg: f64,
    gravity: f64,
) -> ([f64; 3], [f64; 3]) {
    ([0.0, -mass_kg * gravity, 0.0], com_position)
}

/// Convert joint stiffness (0-1 normalized) and angular displacement (rad)
/// to restoring torque (Nm).
///
/// Uses a linear spring model: τ = -k_max × stiffness × θ
/// where k_max is a reference stiffness (100 Nm/rad).
#[must_use]
#[inline]
pub fn joint_stiffness_to_torque(stiffness_normalized: f64, angle_rad: f64) -> f64 {
    let k_max = 100.0; // Nm/rad reference stiffness
    -k_max * stiffness_normalized.clamp(0.0, 1.0) * angle_rad
}

/// Convert joint damping (0-1 normalized) and angular velocity (rad/s)
/// to damping torque (Nm).
///
/// τ = -c_max × damping × ω
/// where c_max is a reference damping coefficient (10 Nm·s/rad).
#[must_use]
#[inline]
pub fn joint_damping_to_torque(damping_normalized: f64, angular_vel_rads: f64) -> f64 {
    let c_max = 10.0; // Nm·s/rad reference damping
    -c_max * damping_normalized.clamp(0.0, 1.0) * angular_vel_rads
}

// ── Ushma bridges (thermodynamics) ─────────────────────────────────────────

/// Convert friction coefficient, normal force (N), and sliding velocity (m/s)
/// to heat generation rate (W).
///
/// P = μ × F_n × v
#[must_use]
#[inline]
pub fn friction_heat_rate(friction_coeff: f64, normal_force_n: f64, velocity_ms: f64) -> f64 {
    (friction_coeff * normal_force_n * velocity_ms).abs()
}

/// Convert collision kinetic energy (J) and material specific heat (J/(kg·K))
/// and contact mass (kg) to temperature rise (K).
///
/// ΔT = E / (m × c_p), assuming all kinetic energy converts to heat.
/// In practice, only a fraction converts — multiply result by an efficiency
/// factor (typically 0.5–0.9).
#[must_use]
#[inline]
pub fn collision_energy_to_temperature_rise(
    kinetic_energy_j: f64,
    contact_mass_kg: f64,
    specific_heat_j_per_kg_k: f64,
) -> f64 {
    let denom = contact_mass_kg * specific_heat_j_per_kg_k;
    if denom <= 0.0 {
        return 0.0;
    }
    kinetic_energy_j.max(0.0) / denom
}

/// Convert temperature difference (K) and coefficient of thermal expansion (1/K)
/// to thermal strain (dimensionless).
///
/// ε_thermal = α × ΔT
#[must_use]
#[inline]
pub fn thermal_strain(expansion_coeff_per_k: f64, temperature_delta_k: f64) -> f64 {
    expansion_coeff_per_k * temperature_delta_k
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── Dravya bridges ─────────────────────────────────────────────────

    #[test]
    fn impact_stress_basic() {
        let s = impact_to_stress(1000.0, 0.01);
        assert!((s - 100_000.0).abs() < 0.1);
    }

    #[test]
    fn impact_stress_zero_area() {
        assert_eq!(impact_to_stress(1000.0, 0.0), 0.0);
    }

    #[test]
    fn strain_rate_basic() {
        let sr = deformation_velocity_to_strain_rate(10.0, 1.0);
        assert!((sr - 10.0).abs() < 0.001);
    }

    #[test]
    fn strain_rate_zero_length() {
        assert_eq!(deformation_velocity_to_strain_rate(10.0, 0.0), 0.0);
    }

    #[test]
    fn bulk_modulus_steel() {
        // Steel: E ≈ 200 GPa, ν ≈ 0.3 → K ≈ 166.7 GPa
        let k = youngs_to_bulk_modulus(200e9, 0.3);
        assert!((k - 166.67e9).abs() < 1e9);
    }

    #[test]
    fn damage_volume_basic() {
        let v = impact_energy_to_damage_volume(100.0, 1e6);
        assert!((v - 1e-4).abs() < 1e-10);
    }

    #[test]
    fn damage_volume_zero_toughness() {
        assert_eq!(impact_energy_to_damage_volume(100.0, 0.0), 0.0);
    }

    // ── Sharira bridges ────────────────────────────────────────────────

    #[test]
    fn torque_to_force_basic() {
        let f = joint_torque_to_force(10.0, 0.5);
        assert!((f - 20.0).abs() < 0.001);
    }

    #[test]
    fn torque_to_force_zero_arm() {
        assert_eq!(joint_torque_to_force(10.0, 0.0), 0.0);
    }

    #[test]
    fn angular_to_tangential() {
        let v = angular_to_tangential_velocity(2.0, 0.5);
        assert!((v - 1.0).abs() < 0.001);
    }

    #[test]
    fn box_inertia_cube() {
        // 1kg cube with 1m sides: Ixx = Iyy = Izz = 1/12 * 1 * (1+1) = 1/6
        let i = segment_to_box_inertia(1.0, 1.0, 1.0, 1.0);
        let expected = 1.0 / 6.0;
        assert!((i[0] - expected).abs() < 0.001);
        assert!((i[1] - expected).abs() < 0.001);
        assert!((i[2] - expected).abs() < 0.001);
    }

    #[test]
    fn limb_force_biped() {
        let f = body_mass_to_limb_force(70.0, 2);
        assert!((f - 70.0 * 9.81 / 2.0).abs() < 0.01);
    }

    #[test]
    fn limb_force_zero_limbs() {
        assert_eq!(body_mass_to_limb_force(70.0, 0), 0.0);
    }

    // ── Ushma bridges ──────────────────────────────────────────────────

    #[test]
    fn friction_heat_basic() {
        let p = friction_heat_rate(0.5, 100.0, 2.0);
        assert!((p - 100.0).abs() < 0.01);
    }

    #[test]
    fn collision_temp_rise() {
        // 100J into 1kg of steel (c_p ≈ 500 J/(kg·K)) → ΔT = 0.2K
        let dt = collision_energy_to_temperature_rise(100.0, 1.0, 500.0);
        assert!((dt - 0.2).abs() < 0.001);
    }

    #[test]
    fn collision_temp_zero_mass() {
        assert_eq!(collision_energy_to_temperature_rise(100.0, 0.0, 500.0), 0.0);
    }

    #[test]
    fn thermal_strain_steel() {
        // Steel α ≈ 12e-6 /K, ΔT = 100K → ε = 0.0012
        let e = thermal_strain(12e-6, 100.0);
        assert!((e - 0.0012).abs() < 1e-6);
    }

    // ── New sharira bridges ────────────────────────────────────────────

    #[test]
    fn muscle_force_to_torque_basic() {
        // 100N force at 0.05m moment arm → 5 Nm
        let t = muscle_force_to_joint_torque(100.0, 0.05);
        assert!((t - 5.0).abs() < 0.001);
    }

    #[test]
    fn muscle_force_to_torque_inverse() {
        // Round-trip: force → torque → force
        let force = 200.0;
        let arm = 0.04;
        let torque = muscle_force_to_joint_torque(force, arm);
        let recovered = joint_torque_to_force(torque, arm);
        assert!((recovered - force).abs() < 0.001);
    }

    #[test]
    fn cylinder_inertia_basic() {
        // 2kg cylinder, length 0.3m, radius 0.02m
        let i = segment_to_cylinder_inertia(2.0, 0.3, 0.02);
        let r2 = 0.02_f64 * 0.02;
        let h2 = 0.3_f64 * 0.3;
        let expected_ixx = 2.0 / 12.0 * (3.0 * r2 + h2);
        let expected_iyy = 2.0 / 2.0 * r2;
        assert!((i[0] - expected_ixx).abs() < 1e-10);
        assert!((i[1] - expected_iyy).abs() < 1e-10);
        assert!((i[2] - expected_ixx).abs() < 1e-10); // Izz == Ixx
    }

    #[test]
    fn gait_grf_positive() {
        let v = gait_grf_to_force_vector(700.0);
        assert_eq!(v, [0.0, 700.0, 0.0]);
    }

    #[test]
    fn gait_grf_negative_input() {
        // Negative input should produce positive upward force
        let v = gait_grf_to_force_vector(-700.0);
        assert_eq!(v, [0.0, 700.0, 0.0]);
    }

    #[test]
    fn body_com_gravity_basic() {
        let (force, point) = body_com_gravity([1.0, 2.0, 3.0], 70.0, 9.81);
        assert!((force[1] - (-70.0 * 9.81)).abs() < 0.001);
        assert_eq!(force[0], 0.0);
        assert_eq!(force[2], 0.0);
        assert_eq!(point, [1.0, 2.0, 3.0]);
    }

    #[test]
    fn joint_stiffness_basic() {
        // Half stiffness, 0.1 rad displacement → τ = -100 * 0.5 * 0.1 = -5 Nm
        let t = joint_stiffness_to_torque(0.5, 0.1);
        assert!((t - (-5.0)).abs() < 0.001);
    }

    #[test]
    fn joint_stiffness_clamps() {
        // Stiffness > 1 should clamp to 1
        let t = joint_stiffness_to_torque(2.0, 0.1);
        assert!((t - (-10.0)).abs() < 0.001);
    }

    #[test]
    fn joint_damping_basic() {
        // Full damping, 1 rad/s → τ = -10 * 1.0 * 1.0 = -10 Nm
        let t = joint_damping_to_torque(1.0, 1.0);
        assert!((t - (-10.0)).abs() < 0.001);
    }

    #[test]
    fn joint_damping_clamps() {
        // Damping < 0 should clamp to 0
        let t = joint_damping_to_torque(-0.5, 1.0);
        assert!((t - 0.0).abs() < 0.001);
    }
}

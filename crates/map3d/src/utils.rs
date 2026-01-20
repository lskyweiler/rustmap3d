use glam;
use rand_distr::{Distribution, Normal};

/// Linear interpolation between two values.
///
/// # Arguments
///
/// * `x` - Start value.
/// * `y` - End value.
/// * `a` - Amount / factor between `x` and `y`, usually between 0 and 1.
///
/// # Returns
///
/// - `value` - Value between `x` and `y` based on a factor `a`.
pub fn lerp(x: f64, y: f64, a: f64) -> f64 {
    return (y - x) * a + x;
}

pub fn quat_forward(q: &glam::DQuat) -> glam::DVec3 {
    return glam::DMat3::from_quat(*q).x_axis;
}
pub fn quat_left(q: &glam::DQuat) -> glam::DVec3 {
    return glam::DMat3::from_quat(*q).y_axis;
}
pub fn quat_up(q: &glam::DQuat) -> glam::DVec3 {
    return glam::DMat3::from_quat(*q).z_axis;
}

/// Generates a random normalized quaternion.
///
/// # Returns
///
/// * `quat` - Random normalized quaternion.
pub fn rand_orientation() -> glam::DQuat {
    return glam::DQuat::from_xyzw(
        rand::random(),
        rand::random(),
        rand::random(),
        rand::random(),
    )
    .normalize();
}

/// Wraps an angle to the domains [[-pi, pi]].
///
/// # Arguments
///
/// * `angle` - Angle [[radians]].
///
/// # Returns
///
/// - `angle` - Angle guaranteed to be in domain [[-pi, pi]] [[radians]].
pub fn wrap_to_pi(angle: f64) -> f64 {
    let mut x = angle % std::f64::consts::TAU;

    if x <= -std::f64::consts::PI {
        x += std::f64::consts::TAU;
    } else if x > std::f64::consts::PI {
        x -= std::f64::consts::TAU;
    }

    x
}

/// Wraps an angle in degrees to be between [0, 360]
///
/// # Arguments
///
/// - `angle_d` (`f64`) - angle in degrees
///
/// # Returns
///
/// - `f64` - 0. <= angle <= 360.
pub fn wrap_to_0_360(angle_d: f64) -> f64 {
    let mut wrapped = angle_d % 360.0;
    if wrapped < 0.0 {
        wrapped += 360.0;
    }
    wrapped
}

/// Generates a uniform random point on the surface of a sphere.
///
/// # Arguments
///
/// * `radius` - Radius of sphere.
///
/// # Returns
///
/// * `vector` - Random point in R3.
pub fn rand_point_on_sphere(radius: f64) -> glam::DVec3 {
    let mut rng = rand::rng();
    let normal = Normal::new(0.0, 1.0).unwrap();

    return radius
        * (glam::DVec3::new(
            normal.sample(&mut rng),
            normal.sample(&mut rng),
            normal.sample(&mut rng),
        )
        .normalize());
}

/// Test utility to assert that two vector's components are close to equal
/// Panics if not close to equal.
///
/// # Arguments
///
/// * `a` - Vector A
/// * `b` - Vector B
/// * `tol` - Absolute tolerance
#[deprecated(
    since = "0.2.0",
    note = "assert!(glam::DVec3::abs_diff_eq(...)) instead"
)]
pub fn assert_vecs_close(a: &glam::DVec3, b: &glam::DVec3, tol: f64) {
    assert!(almost::equal_with(a.x, b.x, tol));
    assert!(almost::equal_with(a.y, b.y, tol));
    assert!(almost::equal_with(a.z, b.z, tol));
}

#[cfg(test)]
mod util_tests {
    use super::*;
    use approx::relative_eq;
    use rstest::*;
    use std::f64::consts::PI;

    #[rstest]
    #[case(-4.0*PI, 0.0)]
    #[case(-1.2*PI, 0.8*PI)]
    #[case(-0.5*PI, -0.5*PI)]
    #[case(0.0, 0.0)]
    #[case(0.6*PI, 0.6*PI)]
    #[case(1.6*PI, -0.4*PI)]
    fn test_wrap_to_pi(#[case] angle: f64, #[case] truth: f64) {
        let test = wrap_to_pi(angle);
        assert!(relative_eq!(
            test,
            truth,
            max_relative = 1e-13,
            epsilon = 1e-14
        ));
    }

    #[rstest]
    #[case(90., 90.)]
    #[case(720., 0.)]
    #[case(450., 90.)]
    #[case(-90., 270.)]
    fn test_wrap_to_0_360(#[case] angle: f64, #[case] truth: f64) {
        let test = wrap_to_0_360(angle);
        assert!(relative_eq!(
            test,
            truth,
            max_relative = 1e-13,
            epsilon = 1e-14
        ));
    }
}

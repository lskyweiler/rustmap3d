use glam;

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

pub fn angle_between_vec2(a: &glam::DVec2, b: &glam::DVec2) -> f64 {
    let mut dot: f64 = a.dot(*b);
    dot = f64::clamp(dot, -1., 1.);
    return f64::acos(dot);
}
pub fn angle_between_vec3(a: &glam::DVec3, b: &glam::DVec3) -> f64 {
    let mut dot: f64 = a.dot(*b);
    dot = f64::clamp(dot, -1., 1.);
    return f64::acos(dot);
}

pub fn rand_orienation() -> glam::DQuat {
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
}

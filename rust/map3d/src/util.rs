use glam;

pub type Vec3Tup = (f64, f64, f64);
pub type QuatTup = (f64, f64, f64, f64);
pub type Mat3Tup = (f64, f64, f64, f64, f64, f64, f64, f64, f64);

pub fn vec3_to_tuple(vec3: &glam::DVec3) -> Vec3Tup {
    return (vec3.x, vec3.y, vec3.z);
}
pub fn tuple_to_vec3(tup: &Vec3Tup) -> glam::DVec3 {
    return glam::DVec3::new(tup.0, tup.1, tup.2);
}
pub fn quat_to_tuple(q: &glam::DQuat) -> QuatTup {
    return (q.w, q.x, q.y, q.z);
}
pub fn tuple_to_quat(q: &QuatTup) -> glam::DQuat {
    // pyglm serializes as w, x, y, z
    return glam::DQuat::from_xyzw(q.1, q.2, q.3, q.0);
}
pub fn mat3_to_tuple(mat3: &glam::DMat3) -> Mat3Tup {
    return (
        mat3.x_axis.x,
        mat3.x_axis.y,
        mat3.x_axis.z,
        mat3.y_axis.x,
        mat3.y_axis.y,
        mat3.y_axis.z,
        mat3.z_axis.x,
        mat3.z_axis.y,
        mat3.z_axis.z,
    );
}

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

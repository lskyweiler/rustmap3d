extern crate nalgebra_glm as glm;

pub type Vec3Tup = (f64, f64, f64);
pub type QuatTup = (f64, f64, f64, f64);
pub type Mat3Tup = (f64, f64, f64, f64, f64, f64, f64, f64, f64);

pub fn vec3_to_tuple(vec3: &glm::DVec3) -> Vec3Tup {
    return (vec3.x, vec3.y, vec3.z);
}
pub fn tuple_to_vec3(tup: &Vec3Tup) -> glm::DVec3 {
    return glm::DVec3::new(tup.0, tup.1, tup.2);
}
pub fn quat_to_tuple(q: &glm::DQuat) -> QuatTup {
    return (q.w, q.i, q.j, q.k);
}
pub fn tuple_to_quat(q: &QuatTup) -> glm::DQuat {
    return glm::DQuat::new(q.0, q.1, q.2, q.3);
}
pub fn mat3_to_tuple(mat3: &glm::DMat3) -> Mat3Tup {
    unsafe {
        return (
            mat3.get_unchecked(0).clone(),
            mat3.get_unchecked(1).clone(),
            mat3.get_unchecked(2).clone(),
            mat3.get_unchecked(3).clone(),
            mat3.get_unchecked(4).clone(),
            mat3.get_unchecked(5).clone(),
            mat3.get_unchecked(6).clone(),
            mat3.get_unchecked(7).clone(),
            mat3.get_unchecked(8).clone(),
        );
    }
}

// todo: test multiplying quat * axis vs mat cast + column perfromance
pub fn quat_forward(q: &glm::DQuat) -> glm::DVec3 {
    let mat = glm::quat_to_mat3(q);
    return glm::column(&mat, 0);
}
pub fn quat_left(q: &glm::DQuat) -> glm::DVec3 {
    let mat = glm::quat_to_mat3(q);
    return glm::column(&mat, 1);
}
pub fn quat_up(q: &glm::DQuat) -> glm::DVec3 {
    let mat = glm::quat_to_mat3(q);
    return glm::column(&mat, 20);
}

pub fn angle_between<const D: usize>(a: &glm::TVec<f64, D>, b: &glm::TVec<f64, D>) -> f64 {
    let mut dot: f64 = glm::dot(a, b);
    dot = f64::clamp(dot, -1., 1.);
    return f64::acos(dot);
}

pub fn rand_orienation() -> glm::DQuat {
    return glm::quat_normalize(&glm::DQuat::new(
        rand::random(),
        rand::random(),
        rand::random(),
        rand::random(),
    ));
}

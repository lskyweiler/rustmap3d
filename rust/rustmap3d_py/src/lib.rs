use map3d::geo;
use pyo3::prelude::*;
extern crate nalgebra_glm as glm;

type Vec3Tup = (f64, f64, f64);
type QuatTup = (f64, f64, f64, f64);

fn vec3_to_tuple(vec3: &glm::DVec3) -> Vec3Tup {
    return (vec3.x, vec3.y, vec3.z);
}
fn tuple_to_vec3(tup: &Vec3Tup) -> glm::DVec3 {
    return glm::DVec3::new(tup.0, tup.1, tup.2);
}
fn quat_to_tuple(q: &glm::DQuat) -> QuatTup {
    return (q.w, q.coords.x, q.coords.y, q.coords.z);
}
fn tuple_to_quat(q: &QuatTup) -> glm::DQuat {
    return glm::DQuat::new(q.0, q.1, q.2, q.3);
}

#[pyfunction]
pub fn ecef2lla(x: f64, y: f64, z: f64) -> Vec3Tup {
    let lla = geo::ecef2lla_map3d(&glm::DVec3::new(x, y, z));
    return vec3_to_tuple(&lla);
}
#[pyfunction]
pub fn lla2ecef(lat: f64, lon: f64, alt: f64) -> Vec3Tup {
    let ecef = geo::lla2ecef(&glm::DVec3::new(lat, lon, alt));
    return vec3_to_tuple(&ecef);
}
#[pyfunction]
pub fn rand_lla() -> Vec3Tup {
    let lla = geo::rand_lla();
    return vec3_to_tuple(&lla);
}
#[pyfunction]
pub fn rand_ecef() -> Vec3Tup {
    let ecef = geo::rand_ecef();
    return vec3_to_tuple(&ecef);
}
#[pyfunction]
pub fn orient_ecef_quat_towards_lla(
    obs_ecef: Vec3Tup,
    obs_ecef_quat: QuatTup,
    target_ecef: Vec3Tup,
) -> QuatTup {
    let ecef_quat = geo::orient_ecef_quat_towards_lla(
        &tuple_to_vec3(&obs_ecef),
        &tuple_to_quat(&obs_ecef_quat),
        &tuple_to_vec3(&target_ecef),
    );
    return quat_to_tuple(&ecef_quat);
}

#[pymodule]
fn rustmap3d(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(ecef2lla, m)?)?;
    m.add_function(wrap_pyfunction!(lla2ecef, m)?)?;
    m.add_function(wrap_pyfunction!(rand_lla, m)?)?;
    m.add_function(wrap_pyfunction!(rand_ecef, m)?)?;
    m.add_function(wrap_pyfunction!(orient_ecef_quat_towards_lla, m)?)?;
    Ok(())
}

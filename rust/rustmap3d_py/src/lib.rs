use map3d::geo;
use pyo3::prelude::*;
extern crate nalgebra_glm as glm;

type Vec3Tup = (f64, f64, f64);

fn vec3_to_tuple(vec3: &glm::DVec3) -> Vec3Tup {
    return (vec3.x, vec3.y, vec3.z);
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
    let lla = geo::rand_lla(None, None, None, None, None, None);
    return vec3_to_tuple(&lla);
}
#[pyfunction]
pub fn rand_ecef() -> Vec3Tup {
    let ecef = geo::rand_ecef(None, None, None, None, None, None);
    return vec3_to_tuple(&ecef);
}

#[pymodule]
fn rustmap3d(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(ecef2lla, m)?)?;
    m.add_function(wrap_pyfunction!(lla2ecef, m)?)?;
    m.add_function(wrap_pyfunction!(rand_lla, m)?)?;
    m.add_function(wrap_pyfunction!(rand_ecef, m)?)?;
    Ok(())
}

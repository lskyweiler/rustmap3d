use map3d::{
    geo,
    util::{self, Vec3Tup},
};
use pyo3::prelude::*;
extern crate nalgebra_glm as glm;

#[pyfunction]
pub fn ecef2lla(x: f64, y: f64, z: f64) -> util::Vec3Tup {
    let lla = geo::ecef2lla_map3d(&glm::DVec3::new(x, y, z));
    return util::vec3_to_tuple(&lla);
}
#[pyfunction]
pub fn lla2ecef(lat: f64, lon: f64, alt: f64) -> util::Vec3Tup {
    let ecef = geo::lla2ecef(&glm::DVec3::new(lat, lon, alt));
    return util::vec3_to_tuple(&ecef);
}
#[pyfunction]
pub fn ecef2enu(x: f64, y: f64, z: f64, ref_lat: f64, ref_lon: f64) -> Vec3Tup {
    let enu = geo::ecef2enu(
        &util::tuple_to_vec3(&(x, y, z)),
        &glm::DVec2::new(ref_lat, ref_lon),
    );
    return util::vec3_to_tuple(&enu);
}
#[pyfunction]
pub fn enu2ecef(e: f64, n: f64, u: f64, ref_lat: f64, ref_lon: f64) -> Vec3Tup {
    let ecef = geo::enu2ecef(
        &util::tuple_to_vec3(&(e, n, u)),
        &glm::DVec2::new(ref_lat, ref_lon),
    );
    return util::vec3_to_tuple(&ecef);
}
#[pyfunction]
pub fn ecef2ned(x: f64, y: f64, z: f64, ref_lat: f64, ref_lon: f64) -> Vec3Tup {
    let ned = geo::ecef2ned(
        &util::tuple_to_vec3(&(x, y, z)),
        &glm::DVec2::new(ref_lat, ref_lon),
    );
    return util::vec3_to_tuple(&ned);
}
#[pyfunction]
pub fn ned2ecef(n: f64, e: f64, d: f64, ref_lat: f64, ref_lon: f64) -> Vec3Tup {
    let ecef = geo::ned2ecef(
        &util::tuple_to_vec3(&(n, e, d)),
        &glm::DVec2::new(ref_lat, ref_lon),
    );
    return util::vec3_to_tuple(&ecef);
}
#[pyfunction]
pub fn ecef2aer(x: f64, y: f64, z: f64, ref_lat: f64, ref_lon: f64) -> Vec3Tup {
    let aer = geo::ecef2aer(
        &util::tuple_to_vec3(&(x, y, z)),
        &glm::DVec2::new(ref_lat, ref_lon),
    );
    return util::vec3_to_tuple(&aer);
}
#[pyfunction]
pub fn aer2ecef(a: f64, e: f64, r: f64, ref_lat: f64, ref_lon: f64, alt: f64) -> Vec3Tup {
    let ecef = geo::aer2ecef(
        &util::tuple_to_vec3(&(a, e, r)),
        &glm::DVec3::new(ref_lat, ref_lon, alt),
    );
    return util::vec3_to_tuple(&ecef);
}
#[pyfunction]
pub fn enu2aer(e: f64, n: f64, u: f64) -> Vec3Tup {
    let aer = geo::enu2aer(&util::tuple_to_vec3(&(e, n, u)));
    return util::vec3_to_tuple(&aer);
}
#[pyfunction]
pub fn aer2enu(a: f64, e: f64, r: f64) -> Vec3Tup {
    let enu = geo::aer2enu(&util::tuple_to_vec3(&(a, e, r)));
    return util::vec3_to_tuple(&enu);
}
#[pyfunction]
pub fn ned2aer(n: f64, e: f64, d: f64) -> Vec3Tup {
    let aer = geo::ned2aer(&util::tuple_to_vec3(&(n, e, d)));
    return util::vec3_to_tuple(&aer);
}
#[pyfunction]
pub fn aer2ned(a: f64, e: f64, r: f64) -> Vec3Tup {
    let ned = geo::aer2enu(&util::tuple_to_vec3(&(a, e, r)));
    return util::vec3_to_tuple(&ned);
}

#[pyfunction]
pub fn rand_lla() -> util::Vec3Tup {
    let lla = geo::rand_lla();
    return util::vec3_to_tuple(&lla);
}
#[pyfunction]
pub fn rand_ecef() -> util::Vec3Tup {
    let ecef = geo::rand_ecef();
    return util::vec3_to_tuple(&ecef);
}
#[pyfunction]
pub fn rand_orientation() -> util::QuatTup {
    let quat = util::rand_orienation();
    return util::quat_to_tuple(&quat);
}

#[pyfunction]
pub fn orient_ecef_quat_towards_lla(
    obs_ecef: util::Vec3Tup,
    obs_ecef_quat: util::QuatTup,
    target_ecef: util::Vec3Tup,
) -> util::QuatTup {
    let ecef_quat = geo::orient_ecef_quat_towards_lla(
        &util::tuple_to_vec3(&obs_ecef),
        &util::tuple_to_quat(&obs_ecef_quat),
        &util::tuple_to_vec3(&target_ecef),
    );
    return util::quat_to_tuple(&ecef_quat);
}

#[pyfunction]
pub fn ecef2enu_quat(lat: f64, lon: f64) -> util::QuatTup {
    let quat = geo::ecef2enu_quat(lat, lon);
    return util::quat_to_tuple(&quat);
}
#[pyfunction]
pub fn enu2ecef_quat(lat: f64, lon: f64) -> util::QuatTup {
    let quat = geo::enu2ecef_quat(lat, lon);
    return util::quat_to_tuple(&quat);
}
#[pyfunction]
pub fn ecef2ned_quat(lat: f64, lon: f64) -> util::QuatTup {
    let quat = geo::ecef2ned_quat(lat, lon);
    return util::quat_to_tuple(&quat);
}
#[pyfunction]
pub fn ned2ecef_quat(lat: f64, lon: f64) -> util::QuatTup {
    let quat = geo::ned2ecef_quat(lat, lon);
    return util::quat_to_tuple(&quat);
}
#[pyfunction]
pub fn ecef2enu_dcm(lat: f64, lon: f64) -> util::Mat3Tup {
    let dcm = geo::ecef2enu_dcm(lat, lon);
    return util::mat3_to_tuple(&dcm);
}
#[pyfunction]
pub fn enu2ecef_dcm(lat: f64, lon: f64) -> util::Mat3Tup {
    let dcm = geo::enu2ecef_dcm(lat, lon);
    return util::mat3_to_tuple(&dcm);
}
#[pyfunction]
pub fn ecef2ned_dcm(lat: f64, lon: f64) -> util::Mat3Tup {
    let dcm = geo::ecef2ned_dcm(lat, lon);
    return util::mat3_to_tuple(&dcm);
}
#[pyfunction]
pub fn ned2ecef_dcm(lat: f64, lon: f64) -> util::Mat3Tup {
    let dcm = geo::ned2ecef_dcm(lat, lon);
    return util::mat3_to_tuple(&dcm);
}

#[pyfunction]
pub fn enu2heading(east: f64, north: f64, up: f64) -> f64 {
    return geo::enu2heading(&util::tuple_to_vec3(&(east, north, up)));
}
#[pyfunction]
pub fn ecef_quat2heading(ecef_quat: util::QuatTup, ref_lat: f64, ref_lon: f64) -> f64 {
    let heading = geo::ecef_quat2heading(
        &util::tuple_to_quat(&ecef_quat),
        &glm::DVec2::new(ref_lat, ref_lon),
    );
    return heading;
}
#[pyfunction]
pub fn ecef2heading(ecef_rel: util::Vec3Tup, ref_lat: f64, ref_lon: f64) -> f64 {
    let heading = geo::ecef2heading(
        &util::tuple_to_vec3(&ecef_rel),
        &glm::DVec2::new(ref_lat, ref_lon),
    );
    return heading;
}
#[pyfunction]
pub fn ecef2bearing(obs_ecef: util::Vec3Tup, targ_ecef: util::Vec3Tup) -> f64 {
    let bearing = geo::ecef2bearing(
        &util::tuple_to_vec3(&obs_ecef),
        &util::tuple_to_vec3(&targ_ecef),
    );
    return bearing;
}

#[pyfunction]
pub fn angle_between(a: util::Vec3Tup, b: util::Vec3Tup) -> f64 {
    return util::angle_between(&util::tuple_to_vec3(&a), &util::tuple_to_vec3(&b));
}

#[pymodule]
fn rustmap3d(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(ecef2lla, m)?)?;
    m.add_function(wrap_pyfunction!(lla2ecef, m)?)?;

    m.add_function(wrap_pyfunction!(ecef2enu, m)?)?;
    m.add_function(wrap_pyfunction!(enu2ecef, m)?)?;
    m.add_function(wrap_pyfunction!(ecef2ned, m)?)?;
    m.add_function(wrap_pyfunction!(ned2ecef, m)?)?;
    m.add_function(wrap_pyfunction!(ecef2aer, m)?)?;
    m.add_function(wrap_pyfunction!(aer2ecef, m)?)?;
    m.add_function(wrap_pyfunction!(enu2aer, m)?)?;
    m.add_function(wrap_pyfunction!(aer2enu, m)?)?;
    m.add_function(wrap_pyfunction!(ned2aer, m)?)?;
    m.add_function(wrap_pyfunction!(aer2ned, m)?)?;

    m.add_function(wrap_pyfunction!(rand_lla, m)?)?;
    m.add_function(wrap_pyfunction!(rand_ecef, m)?)?;
    m.add_function(wrap_pyfunction!(rand_orientation, m)?)?;

    m.add_function(wrap_pyfunction!(orient_ecef_quat_towards_lla, m)?)?;

    m.add_function(wrap_pyfunction!(ecef2enu_quat, m)?)?;
    m.add_function(wrap_pyfunction!(enu2ecef_quat, m)?)?;
    m.add_function(wrap_pyfunction!(ecef2ned_quat, m)?)?;
    m.add_function(wrap_pyfunction!(ned2ecef_quat, m)?)?;

    m.add_function(wrap_pyfunction!(ecef2enu_dcm, m)?)?;
    m.add_function(wrap_pyfunction!(enu2ecef_dcm, m)?)?;
    m.add_function(wrap_pyfunction!(ecef2ned_dcm, m)?)?;
    m.add_function(wrap_pyfunction!(ned2ecef_dcm, m)?)?;

    m.add_function(wrap_pyfunction!(ecef2bearing, m)?)?;
    m.add_function(wrap_pyfunction!(angle_between, m)?)?;
    Ok(())
}

use pyo3::prelude::*;
use map3d::geo;


#[pyfunction]
pub fn ecef2lla(x: f64, y: f64, z: f64) -> (f64, f64, f64) {
    return geo::ecef2lla_map3d(x, y, z);
}
#[pyfunction]
pub fn lla2ecef(lat: f64, lon: f64, alt: f64) -> (f64, f64, f64) {
    return geo::lla2ecef(lat, lon, alt);
}


#[pymodule]
fn rustmap3d(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(ecef2lla, m)?)?;
    m.add_function(wrap_pyfunction!(lla2ecef, m)?)?;
    Ok(())
}
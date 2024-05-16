use pyo3::prelude::*;
use map3d::geo;


#[pyfunction]
pub fn ecef2lla(x: f64, y: f64, z: f64) -> (f64, f64, f64) {
    return geo::ecef2lla_ferarri(x, y, z);
}


#[pymodule]
fn rustmap3d(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(ecef2lla, m)?)?;
    Ok(())
}
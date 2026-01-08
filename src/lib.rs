use pymap3d::*;
use pyo3::prelude::*;
use pyo3_stub_gen::define_stub_info_gatherer;

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
    m.add_function(wrap_pyfunction!(ecef_uvw2enu, m)?)?;
    m.add_function(wrap_pyfunction!(enu2ecef_uvw, m)?)?;
    m.add_function(wrap_pyfunction!(ecef_uvw2ned, m)?)?;
    m.add_function(wrap_pyfunction!(ned2ecef_uvw, m)?)?;
    m.add_function(wrap_pyfunction!(ecef_uvw2aer, m)?)?;
    m.add_function(wrap_pyfunction!(aer2ecef_uvw, m)?)?;

    m.add_function(wrap_pyfunction!(enu2aer, m)?)?;
    m.add_function(wrap_pyfunction!(aer2enu, m)?)?;
    m.add_function(wrap_pyfunction!(ned2aer, m)?)?;
    m.add_function(wrap_pyfunction!(aer2ned, m)?)?;

    m.add_function(wrap_pyfunction!(rand_lla, m)?)?;
    m.add_function(wrap_pyfunction!(rand_ecef, m)?)?;
    m.add_function(wrap_pyfunction!(rand_orientation, m)?)?;

    m.add_function(wrap_pyfunction!(ecef2enu_quat, m)?)?;
    m.add_function(wrap_pyfunction!(enu2ecef_quat, m)?)?;
    m.add_function(wrap_pyfunction!(ecef2ned_quat, m)?)?;
    m.add_function(wrap_pyfunction!(ned2ecef_quat, m)?)?;

    m.add_function(wrap_pyfunction!(ecef2enu_dcm, m)?)?;
    m.add_function(wrap_pyfunction!(enu2ecef_dcm, m)?)?;
    m.add_function(wrap_pyfunction!(ecef2ned_dcm, m)?)?;
    m.add_function(wrap_pyfunction!(ned2ecef_dcm, m)?)?;

    m.add_function(wrap_pyfunction!(dms2dd, m)?)?;
    m.add_function(wrap_pyfunction!(dd2dms, m)?)?;
    m.add_function(wrap_pyfunction!(ll2dms, m)?)?;

    m.add_function(wrap_pyfunction!(angle_between, m)?)?;

    m.add_function(wrap_pyfunction!(vincenty_direct, m)?)?;
    m.add_function(wrap_pyfunction!(vincenty_inverse, m)?)?;
    Ok(())
}

define_stub_info_gatherer!(stub_info);

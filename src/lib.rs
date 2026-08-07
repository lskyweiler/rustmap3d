use map3d;
use pymap3d::*;
use pyo3::{prelude::*, PyTypeInfo};
use pyo3_stub_gen::define_stub_info_gatherer;

fn override_pyclass_module<'py, P: PyTypeInfo>(py: Python<'py>, module: &str) -> PyResult<()> {
    let py_cls = P::type_object(py);
    py_cls.setattr("__module__", module)?;
    Ok(())
}

#[pymodule]
fn rustmap3d(m: &Bound<'_, PyModule>) -> PyResult<()> {
    {
        m.gil_used(false)?; // 3.14t support

        m.add_function(wrap_pyfunction!(ecef2lla, m)?)?;
        m.add_function(wrap_pyfunction!(lla2ecef, m)?)?;

        m.add_function(wrap_pyfunction!(ecef2enu, m)?)?;
        m.add_function(wrap_pyfunction!(enu2ecef, m)?)?;
        m.add_function(wrap_pyfunction!(ecef2ned, m)?)?;
        m.add_function(wrap_pyfunction!(ned2ecef, m)?)?;
        m.add_function(wrap_pyfunction!(ecef2aer, m)?)?;
        m.add_function(wrap_pyfunction!(aer2ecef, m)?)?;
        m.add_function(wrap_pyfunction!(lla2ned, m)?)?;
        m.add_function(wrap_pyfunction!(ned2lla, m)?)?;
        m.add_function(wrap_pyfunction!(lla2enu, m)?)?;
        m.add_function(wrap_pyfunction!(enu2lla, m)?)?;
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

        m.add_function(wrap_pyfunction!(mach, m)?)?;

        m.add_function(wrap_pyfunction!(vincenty_direct, m)?)?;
        m.add_function(wrap_pyfunction!(vincenty_inverse, m)?)?;
    }
    {
        m.add_class::<map3d::geo_objects::geo_position::GeoPosition>()?;
        m.add_class::<map3d::geo_objects::geo_vector::GeoVector>()?;
        m.add_class::<map3d::geo_objects::geo_orientation::GeoOrientation>()?;
        m.add_class::<map3d::geo_objects::geo_velocity::GeoVelocity>()?;
        m.add_class::<map3d::geo_objects::geo_orientation::EulerRot>()?;

        // We need to tell python that these classes are compiled into this library
        override_pyclass_module::<map3d::DVec3>(m.py(), "rustmap3d")?;
        override_pyclass_module::<map3d::DQuat>(m.py(), "rustmap3d")?;
        m.add_class::<map3d::DVec3>()?;
        m.add_class::<map3d::DQuat>()?;
    }

    Ok(())
}

define_stub_info_gatherer!(stub_info);

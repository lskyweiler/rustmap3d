use map3d;
use pyo3::{prelude::*, PyTypeInfo};
use pyo3_stub_gen::define_stub_info_gatherer;

/// Overrides a class's module path. This is necessary so the classes don't show as builtins. Hopefully this is fixed in future versions of pyo3
fn override_pyclass_module<'py, P: PyTypeInfo>(py: Python<'py>, module: &str) -> PyResult<()> {
    let py_cls = P::type_object(py);
    py_cls.setattr("__module__", module)?;
    Ok(())
}

#[pymodule]
fn rustmap3d(m: &Bound<'_, PyModule>) -> PyResult<()> {
    pymap3d::add_to_pymodule(m)?;

    {
        m.add_class::<map3d::geo_objects::geo_position::GeoPosition>()?;
        m.add_class::<map3d::geo_objects::geo_vector::GeoVector>()?;
        m.add_class::<map3d::geo_objects::geo_orientation::GeoOrientation>()?;
        m.add_class::<map3d::geo_objects::geo_velocity::GeoVelocity>()?;
        m.add_class::<map3d::geo_objects::geo_orientation::EulerRot>()?;
        m.add_class::<map3d::geo_objects::geo_polygon_zone::GeoPolygonZone>()?;

        // We need to tell python that these classes are compiled into this library
        override_pyclass_module::<map3d::DVec3>(m.py(), "rustmap3d")?;
        override_pyclass_module::<map3d::DQuat>(m.py(), "rustmap3d")?;
        m.add_class::<map3d::DVec3>()?;
        m.add_class::<map3d::DQuat>()?;
    }

    Ok(())
}

define_stub_info_gatherer!(stub_info);

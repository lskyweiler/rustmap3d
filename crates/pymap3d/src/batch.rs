//! NumPy bindings for the batched transforms.
//!
//! Every function takes anything NumPy can convert to a float64 array of shape (N, 3), one point
//! per row, and returns a new (N, 3) float64 array. Results match calling the single-point
//! function on each row. The conversion runs without holding the Python GIL.
use glam;
use numpy::{ndarray::Array2, AllowTypeChange, PyArray2, PyArrayLikeDyn, PyUntypedArrayMethods};
use pyo3::{exceptions::*, prelude::*};
use pyo3_stub_gen::derive::*;
use std::sync::{LazyLock, Mutex};

/// Create a thread pool for this module.
/// This is done on the python side since we want a rust user to have total control over rayon behavior
static THREAD_POOL: LazyLock<Mutex<rayon::ThreadPool>> = LazyLock::new(|| {
    // Initialize a a thread pool for this module
    Mutex::new(
        rayon::ThreadPoolBuilder::new()
            .num_threads(std::thread::available_parallelism().unwrap().get())
            .build()
            .expect("Failed to initialize the local threadpool"),
    )
});

/// Configures the internal thread pool
///
/// # Arguments
///
/// - `num_threads` (`u32`) - Set the number of threads to use in the batched functions. None means use all threads
///
/// # Errors
///
/// - RuntimeError - If the threadpool could not be created using this configuration
///
#[gen_stub_pyfunction]
#[pyfunction]
#[pyo3(signature = (num_threads = None))]
pub fn configure_threads(num_threads: Option<usize>) -> PyResult<()> {
    match THREAD_POOL.lock() {
        Ok(mut locked) => {
            let n_threads =
                num_threads.unwrap_or(std::thread::available_parallelism().unwrap().get());
            if n_threads != locked.current_num_threads() {
                *locked = rayon::ThreadPoolBuilder::new()
                    .num_threads(n_threads)
                    .build()
                    .expect("Failed to initialize the local threadpool");
            }
            Ok(())
        }
        Err(what) => Err(PyRuntimeError::new_err(format!("{:?}", what))),
    }
}

/// Helper function to execute a batch function using the module's thread pool
fn execute_in_local_thread_pool<'py>(
    py: Python<'py>,
    kernel: impl FnOnce() -> Vec<glam::DVec3> + Sync + Send,
) -> PyResult<Bound<'py, PyArray2<f64>>> {
    let out = py.detach(|| THREAD_POOL.lock().expect("msg").install(kernel));
    return Ok(dvec3s_to_rows(py, out));
}

/// Copies an (N, 3) array into one vector per row.
fn rows_to_dvec3s(
    arr: &PyArrayLikeDyn<f64, AllowTypeChange>,
    name: &str,
) -> PyResult<Vec<glam::DVec3>> {
    let shape = arr.shape();
    if shape.len() != 2 || shape[1] != 3 {
        return Err(PyValueError::new_err(format!(
            "{name} must have shape (N, 3), got {shape:?}"
        )));
    }
    let view = arr.as_array();
    return Ok(view
        .rows()
        .into_iter()
        .map(|row| glam::dvec3(row[[0]], row[[1]], row[[2]]))
        .collect());
}

/// Builds an (N, 3) array from one vector per row.
fn dvec3s_to_rows<'py>(py: Python<'py>, vecs: Vec<glam::DVec3>) -> Bound<'py, PyArray2<f64>> {
    let n = vecs.len();
    let flat: Vec<f64> = vecs.into_iter().flat_map(|v| v.to_array()).collect();
    let arr = Array2::from_shape_vec((n, 3), flat).expect("n * 3 values for an (n, 3) array");
    return PyArray2::from_owned_array(py, arr);
}

/// Batched `ecef2lla`: converts each row of `ecef` from ECEF positions in meters to
/// WGS84 LLA [degrees-degrees-meters].
///
/// # Arguments
///
/// * `ecef` - Array-like of shape (N, 3) of ECEF positions in meters [numpy.typing.ArrayLike]
///
/// # Returns
///
/// * Array of shape (N, 3) of WGS84 LLA [degrees-degrees-meters] [numpy.ndarray[float64]]
///
/// # Raises
///
/// * `ValueError` if `ecef` is not of shape (N, 3)
#[gen_stub_pyfunction]
#[pyfunction]
pub fn ecef2lla_batch<'py>(
    py: Python<'py>,
    #[gen_stub(override_type(type_repr = "numpy.typing.ArrayLike", imports = ("numpy.typing")))]
    ecef: PyArrayLikeDyn<'py, f64, AllowTypeChange>,
) -> PyResult<Bound<'py, PyArray2<f64>>> {
    let v = rows_to_dvec3s(&ecef, "ecef")?;
    execute_in_local_thread_pool(py, || map3d::ecef2lla_batch(&v))
}

/// Batched `lla2ecef`: converts each row of `lla` from WGS84 LLA [degrees-degrees-meters] to
/// ECEF positions in meters.
///
/// # Arguments
///
/// * `lla` - Array-like of shape (N, 3) of WGS84 LLA [degrees-degrees-meters] [numpy.typing.ArrayLike]
///
/// # Returns
///
/// * Array of shape (N, 3) of ECEF positions in meters [numpy.ndarray[float64]]
///
/// # Raises
///
/// * `ValueError` if `lla` is not of shape (N, 3)
#[gen_stub_pyfunction]
#[pyfunction]
pub fn lla2ecef_batch<'py>(
    py: Python<'py>,
    #[gen_stub(override_type(type_repr = "numpy.typing.ArrayLike", imports = ("numpy.typing")))]
    lla: PyArrayLikeDyn<'py, f64, AllowTypeChange>,
) -> PyResult<Bound<'py, PyArray2<f64>>> {
    let v = rows_to_dvec3s(&lla, "lla")?;
    execute_in_local_thread_pool(py, || map3d::lla2ecef_batch(&v))
}

/// Batched `ecef_uvw2enu`: converts each row of `ecef_uvw` from ECEF uvw vectors in meters to
/// ENU vectors in meters.
///
/// # Arguments
///
/// * `ecef_uvw` - Array-like of shape (N, 3) of ECEF uvw vectors in meters [numpy.typing.ArrayLike]
/// * `lat_ref_d` - Reference WGS84 Lat in degrees [float]
/// * `lon_ref_d` - Reference WGS84 Lon in degrees [float]
///
/// # Returns
///
/// * Array of shape (N, 3) of ENU vectors in meters [numpy.ndarray[float64]]
///
/// # Raises
///
/// * `ValueError` if `ecef_uvw` is not of shape (N, 3)
#[gen_stub_pyfunction]
#[pyfunction]
pub fn ecef_uvw2enu_batch<'py>(
    py: Python<'py>,
    #[gen_stub(override_type(type_repr = "numpy.typing.ArrayLike", imports = ("numpy.typing")))]
    ecef_uvw: PyArrayLikeDyn<'py, f64, AllowTypeChange>,
    lat_ref_d: f64,
    lon_ref_d: f64,
) -> PyResult<Bound<'py, PyArray2<f64>>> {
    let v = rows_to_dvec3s(&ecef_uvw, "ecef_uvw")?;
    let r = glam::dvec3(lat_ref_d, lon_ref_d, 0.);
    execute_in_local_thread_pool(py, || map3d::ecef_uvw2enu_batch(&v, r))
}

/// Batched `ecef2enu`: converts each row of `ecef` from ECEF positions in meters to
/// ENU vectors in meters.
///
/// # Arguments
///
/// * `ecef` - Array-like of shape (N, 3) of ECEF positions in meters [numpy.typing.ArrayLike]
/// * `lat_ref_d` - Reference WGS84 Lat in degrees [float]
/// * `lon_ref_d` - Reference WGS84 Lon in degrees [float]
/// * `alt_ref_m` - Reference WGS84 altitude in meters, defaults to 0 [float]
///
/// # Returns
///
/// * Array of shape (N, 3) of ENU vectors in meters [numpy.ndarray[float64]]
///
/// # Raises
///
/// * `ValueError` if `ecef` is not of shape (N, 3)
#[gen_stub_pyfunction]
#[pyfunction]
#[pyo3(signature = (ecef, lat_ref_d, lon_ref_d, alt_ref_m=0.0))]
pub fn ecef2enu_batch<'py>(
    py: Python<'py>,
    #[gen_stub(override_type(type_repr = "numpy.typing.ArrayLike", imports = ("numpy.typing")))]
    ecef: PyArrayLikeDyn<'py, f64, AllowTypeChange>,
    lat_ref_d: f64,
    lon_ref_d: f64,
    alt_ref_m: f64,
) -> PyResult<Bound<'py, PyArray2<f64>>> {
    let v = rows_to_dvec3s(&ecef, "ecef")?;
    let r = glam::dvec3(lat_ref_d, lon_ref_d, alt_ref_m);
    execute_in_local_thread_pool(py, || map3d::ecef2enu_batch(&v, r))
}

/// Batched `lla2enu`: converts each row of `lla` from WGS84 LLA [degrees-degrees-meters] to
/// ENU vectors in meters.
///
/// # Arguments
///
/// * `lla` - Array-like of shape (N, 3) of WGS84 LLA [degrees-degrees-meters] [numpy.typing.ArrayLike]
/// * `lat_ref_d` - Reference WGS84 Lat in degrees [float]
/// * `lon_ref_d` - Reference WGS84 Lon in degrees [float]
/// * `alt_ref_m` - Reference WGS84 altitude in meters, defaults to 0 [float]
///
/// # Returns
///
/// * Array of shape (N, 3) of ENU vectors in meters [numpy.ndarray[float64]]
///
/// # Raises
///
/// * `ValueError` if `lla` is not of shape (N, 3)
#[gen_stub_pyfunction]
#[pyfunction]
#[pyo3(signature = (lla, lat_ref_d, lon_ref_d, alt_ref_m=0.0))]
pub fn lla2enu_batch<'py>(
    py: Python<'py>,
    #[gen_stub(override_type(type_repr = "numpy.typing.ArrayLike", imports = ("numpy.typing")))]
    lla: PyArrayLikeDyn<'py, f64, AllowTypeChange>,
    lat_ref_d: f64,
    lon_ref_d: f64,
    alt_ref_m: f64,
) -> PyResult<Bound<'py, PyArray2<f64>>> {
    let v = rows_to_dvec3s(&lla, "lla")?;
    let r = glam::dvec3(lat_ref_d, lon_ref_d, alt_ref_m);
    execute_in_local_thread_pool(py, || map3d::ecef2enu_batch(&map3d::lla2ecef_batch(&v), r))
}

/// Batched `enu2ecef_uvw`: converts each row of `enu` from ENU vectors in meters to
/// ECEF uvw vectors in meters.
///
/// # Arguments
///
/// * `enu` - Array-like of shape (N, 3) of ENU vectors in meters [numpy.typing.ArrayLike]
/// * `lat_ref_d` - Reference WGS84 Lat in degrees [float]
/// * `lon_ref_d` - Reference WGS84 Lon in degrees [float]
///
/// # Returns
///
/// * Array of shape (N, 3) of ECEF uvw vectors in meters [numpy.ndarray[float64]]
///
/// # Raises
///
/// * `ValueError` if `enu` is not of shape (N, 3)
#[gen_stub_pyfunction]
#[pyfunction]
pub fn enu2ecef_uvw_batch<'py>(
    py: Python<'py>,
    #[gen_stub(override_type(type_repr = "numpy.typing.ArrayLike", imports = ("numpy.typing")))]
    enu: PyArrayLikeDyn<'py, f64, AllowTypeChange>,
    lat_ref_d: f64,
    lon_ref_d: f64,
) -> PyResult<Bound<'py, PyArray2<f64>>> {
    let v = rows_to_dvec3s(&enu, "enu")?;
    let r = glam::dvec3(lat_ref_d, lon_ref_d, 0.);
    execute_in_local_thread_pool(py, || map3d::enu2ecef_uvw_batch(&v, r))
}

/// Batched `enu2ecef`: converts each row of `enu` from ENU vectors in meters to
/// ECEF positions in meters.
///
/// # Arguments
///
/// * `enu` - Array-like of shape (N, 3) of ENU vectors in meters [numpy.typing.ArrayLike]
/// * `lat_ref_d` - Reference WGS84 Lat in degrees [float]
/// * `lon_ref_d` - Reference WGS84 Lon in degrees [float]
/// * `alt_ref_m` - Reference WGS84 altitude in meters, defaults to 0 [float]
///
/// # Returns
///
/// * Array of shape (N, 3) of ECEF positions in meters [numpy.ndarray[float64]]
///
/// # Raises
///
/// * `ValueError` if `enu` is not of shape (N, 3)
#[gen_stub_pyfunction]
#[pyfunction]
#[pyo3(signature = (enu, lat_ref_d, lon_ref_d, alt_ref_m=0.0))]
pub fn enu2ecef_batch<'py>(
    py: Python<'py>,
    #[gen_stub(override_type(type_repr = "numpy.typing.ArrayLike", imports = ("numpy.typing")))]
    enu: PyArrayLikeDyn<'py, f64, AllowTypeChange>,
    lat_ref_d: f64,
    lon_ref_d: f64,
    alt_ref_m: f64,
) -> PyResult<Bound<'py, PyArray2<f64>>> {
    let v = rows_to_dvec3s(&enu, "enu")?;
    let r = glam::dvec3(lat_ref_d, lon_ref_d, alt_ref_m);
    execute_in_local_thread_pool(py, || map3d::enu2ecef_batch(&v, r))
}

/// Batched `enu2lla`: converts each row of `enu` from ENU vectors in meters to
/// WGS84 LLA [degrees-degrees-meters].
///
/// # Arguments
///
/// * `enu` - Array-like of shape (N, 3) of ENU vectors in meters [numpy.typing.ArrayLike]
/// * `lat_ref_d` - Reference WGS84 Lat in degrees [float]
/// * `lon_ref_d` - Reference WGS84 Lon in degrees [float]
/// * `alt_ref_m` - Reference WGS84 altitude in meters, defaults to 0 [float]
///
/// # Returns
///
/// * Array of shape (N, 3) of WGS84 LLA [degrees-degrees-meters] [numpy.ndarray[float64]]
///
/// # Raises
///
/// * `ValueError` if `enu` is not of shape (N, 3)
#[gen_stub_pyfunction]
#[pyfunction]
#[pyo3(signature = (enu, lat_ref_d, lon_ref_d, alt_ref_m=0.0))]
pub fn enu2lla_batch<'py>(
    py: Python<'py>,
    #[gen_stub(override_type(type_repr = "numpy.typing.ArrayLike", imports = ("numpy.typing")))]
    enu: PyArrayLikeDyn<'py, f64, AllowTypeChange>,
    lat_ref_d: f64,
    lon_ref_d: f64,
    alt_ref_m: f64,
) -> PyResult<Bound<'py, PyArray2<f64>>> {
    let v = rows_to_dvec3s(&enu, "enu")?;
    let r = glam::dvec3(lat_ref_d, lon_ref_d, alt_ref_m);
    execute_in_local_thread_pool(py, || map3d::ecef2lla_batch(&map3d::enu2ecef_batch(&v, r)))
}

/// Batched `ecef_uvw2ned`: converts each row of `ecef_uvw` from ECEF uvw vectors in meters to
/// NED vectors in meters.
///
/// # Arguments
///
/// * `ecef_uvw` - Array-like of shape (N, 3) of ECEF uvw vectors in meters [numpy.typing.ArrayLike]
/// * `lat_ref_d` - Reference WGS84 Lat in degrees [float]
/// * `lon_ref_d` - Reference WGS84 Lon in degrees [float]
///
/// # Returns
///
/// * Array of shape (N, 3) of NED vectors in meters [numpy.ndarray[float64]]
///
/// # Raises
///
/// * `ValueError` if `ecef_uvw` is not of shape (N, 3)
#[gen_stub_pyfunction]
#[pyfunction]
pub fn ecef_uvw2ned_batch<'py>(
    py: Python<'py>,
    #[gen_stub(override_type(type_repr = "numpy.typing.ArrayLike", imports = ("numpy.typing")))]
    ecef_uvw: PyArrayLikeDyn<'py, f64, AllowTypeChange>,
    lat_ref_d: f64,
    lon_ref_d: f64,
) -> PyResult<Bound<'py, PyArray2<f64>>> {
    let v = rows_to_dvec3s(&ecef_uvw, "ecef_uvw")?;
    let r = glam::dvec3(lat_ref_d, lon_ref_d, 0.);
    execute_in_local_thread_pool(py, || map3d::ecef_uvw2ned_batch(&v, r))
}

/// Batched `ecef2ned`: converts each row of `ecef` from ECEF positions in meters to
/// NED vectors in meters.
///
/// # Arguments
///
/// * `ecef` - Array-like of shape (N, 3) of ECEF positions in meters [numpy.typing.ArrayLike]
/// * `lat_ref_d` - Reference WGS84 Lat in degrees [float]
/// * `lon_ref_d` - Reference WGS84 Lon in degrees [float]
/// * `alt_ref_m` - Reference WGS84 altitude in meters, defaults to 0 [float]
///
/// # Returns
///
/// * Array of shape (N, 3) of NED vectors in meters [numpy.ndarray[float64]]
///
/// # Raises
///
/// * `ValueError` if `ecef` is not of shape (N, 3)
#[gen_stub_pyfunction]
#[pyfunction]
#[pyo3(signature = (ecef, lat_ref_d, lon_ref_d, alt_ref_m=0.0))]
pub fn ecef2ned_batch<'py>(
    py: Python<'py>,
    #[gen_stub(override_type(type_repr = "numpy.typing.ArrayLike", imports = ("numpy.typing")))]
    ecef: PyArrayLikeDyn<'py, f64, AllowTypeChange>,
    lat_ref_d: f64,
    lon_ref_d: f64,
    alt_ref_m: f64,
) -> PyResult<Bound<'py, PyArray2<f64>>> {
    let v = rows_to_dvec3s(&ecef, "ecef")?;
    let r = glam::dvec3(lat_ref_d, lon_ref_d, alt_ref_m);
    execute_in_local_thread_pool(py, || map3d::ecef2ned_batch(&v, r))
}

/// Batched `lla2ned`: converts each row of `lla` from WGS84 LLA [degrees-degrees-meters] to
/// NED vectors in meters.
///
/// # Arguments
///
/// * `lla` - Array-like of shape (N, 3) of WGS84 LLA [degrees-degrees-meters] [numpy.typing.ArrayLike]
/// * `lat_ref_d` - Reference WGS84 Lat in degrees [float]
/// * `lon_ref_d` - Reference WGS84 Lon in degrees [float]
/// * `alt_ref_m` - Reference WGS84 altitude in meters, defaults to 0 [float]
///
/// # Returns
///
/// * Array of shape (N, 3) of NED vectors in meters [numpy.ndarray[float64]]
///
/// # Raises
///
/// * `ValueError` if `lla` is not of shape (N, 3)
#[gen_stub_pyfunction]
#[pyfunction]
#[pyo3(signature = (lla, lat_ref_d, lon_ref_d, alt_ref_m=0.0))]
pub fn lla2ned_batch<'py>(
    py: Python<'py>,
    #[gen_stub(override_type(type_repr = "numpy.typing.ArrayLike", imports = ("numpy.typing")))]
    lla: PyArrayLikeDyn<'py, f64, AllowTypeChange>,
    lat_ref_d: f64,
    lon_ref_d: f64,
    alt_ref_m: f64,
) -> PyResult<Bound<'py, PyArray2<f64>>> {
    let v = rows_to_dvec3s(&lla, "lla")?;
    let r = glam::dvec3(lat_ref_d, lon_ref_d, alt_ref_m);
    execute_in_local_thread_pool(py, || map3d::ecef2ned_batch(&map3d::lla2ecef_batch(&v), r))
}

/// Batched `ned2ecef_uvw`: converts each row of `ned` from NED vectors in meters to
/// ECEF uvw vectors in meters.
///
/// # Arguments
///
/// * `ned` - Array-like of shape (N, 3) of NED vectors in meters [numpy.typing.ArrayLike]
/// * `lat_ref_d` - Reference WGS84 Lat in degrees [float]
/// * `lon_ref_d` - Reference WGS84 Lon in degrees [float]
///
/// # Returns
///
/// * Array of shape (N, 3) of ECEF uvw vectors in meters [numpy.ndarray[float64]]
///
/// # Raises
///
/// * `ValueError` if `ned` is not of shape (N, 3)
#[gen_stub_pyfunction]
#[pyfunction]
pub fn ned2ecef_uvw_batch<'py>(
    py: Python<'py>,
    #[gen_stub(override_type(type_repr = "numpy.typing.ArrayLike", imports = ("numpy.typing")))]
    ned: PyArrayLikeDyn<'py, f64, AllowTypeChange>,
    lat_ref_d: f64,
    lon_ref_d: f64,
) -> PyResult<Bound<'py, PyArray2<f64>>> {
    let v = rows_to_dvec3s(&ned, "ned")?;
    let r = glam::dvec3(lat_ref_d, lon_ref_d, 0.);
    execute_in_local_thread_pool(py, || map3d::ned2ecef_uvw_batch(&v, r))
}

/// Batched `ned2ecef`: converts each row of `ned` from NED vectors in meters to
/// ECEF positions in meters.
///
/// # Arguments
///
/// * `ned` - Array-like of shape (N, 3) of NED vectors in meters [numpy.typing.ArrayLike]
/// * `lat_ref_d` - Reference WGS84 Lat in degrees [float]
/// * `lon_ref_d` - Reference WGS84 Lon in degrees [float]
/// * `alt_ref_m` - Reference WGS84 altitude in meters, defaults to 0 [float]
///
/// # Returns
///
/// * Array of shape (N, 3) of ECEF positions in meters [numpy.ndarray[float64]]
///
/// # Raises
///
/// * `ValueError` if `ned` is not of shape (N, 3)
#[gen_stub_pyfunction]
#[pyfunction]
#[pyo3(signature = (ned, lat_ref_d, lon_ref_d, alt_ref_m=0.0))]
pub fn ned2ecef_batch<'py>(
    py: Python<'py>,
    #[gen_stub(override_type(type_repr = "numpy.typing.ArrayLike", imports = ("numpy.typing")))]
    ned: PyArrayLikeDyn<'py, f64, AllowTypeChange>,
    lat_ref_d: f64,
    lon_ref_d: f64,
    alt_ref_m: f64,
) -> PyResult<Bound<'py, PyArray2<f64>>> {
    let v = rows_to_dvec3s(&ned, "ned")?;
    let r = glam::dvec3(lat_ref_d, lon_ref_d, alt_ref_m);
    execute_in_local_thread_pool(py, || map3d::ned2ecef_batch(&v, r))
}

/// Batched `ned2lla`: converts each row of `ned` from NED vectors in meters to
/// WGS84 LLA [degrees-degrees-meters].
///
/// # Arguments
///
/// * `ned` - Array-like of shape (N, 3) of NED vectors in meters [numpy.typing.ArrayLike]
/// * `lat_ref_d` - Reference WGS84 Lat in degrees [float]
/// * `lon_ref_d` - Reference WGS84 Lon in degrees [float]
/// * `alt_ref_m` - Reference WGS84 altitude in meters, defaults to 0 [float]
///
/// # Returns
///
/// * Array of shape (N, 3) of WGS84 LLA [degrees-degrees-meters] [numpy.ndarray[float64]]
///
/// # Raises
///
/// * `ValueError` if `ned` is not of shape (N, 3)
#[gen_stub_pyfunction]
#[pyfunction]
#[pyo3(signature = (ned, lat_ref_d, lon_ref_d, alt_ref_m=0.0))]
pub fn ned2lla_batch<'py>(
    py: Python<'py>,
    #[gen_stub(override_type(type_repr = "numpy.typing.ArrayLike", imports = ("numpy.typing")))]
    ned: PyArrayLikeDyn<'py, f64, AllowTypeChange>,
    lat_ref_d: f64,
    lon_ref_d: f64,
    alt_ref_m: f64,
) -> PyResult<Bound<'py, PyArray2<f64>>> {
    let v = rows_to_dvec3s(&ned, "ned")?;
    let r = glam::dvec3(lat_ref_d, lon_ref_d, alt_ref_m);
    execute_in_local_thread_pool(py, || map3d::ecef2lla_batch(&map3d::ned2ecef_batch(&v, r)))
}

/// Batched `ecef_uvw2aer`: converts each row of `ecef_uvw` from ECEF uvw vectors in meters to
/// AER [degrees-degrees-meters].
///
/// # Arguments
///
/// * `ecef_uvw` - Array-like of shape (N, 3) of ECEF uvw vectors in meters [numpy.typing.ArrayLike]
/// * `lat_ref_d` - Reference WGS84 Lat in degrees [float]
/// * `lon_ref_d` - Reference WGS84 Lon in degrees [float]
///
/// # Returns
///
/// * Array of shape (N, 3) of AER [degrees-degrees-meters] [numpy.ndarray[float64]]
///
/// # Raises
///
/// * `ValueError` if `ecef_uvw` is not of shape (N, 3)
#[gen_stub_pyfunction]
#[pyfunction]
pub fn ecef_uvw2aer_batch<'py>(
    py: Python<'py>,
    #[gen_stub(override_type(type_repr = "numpy.typing.ArrayLike", imports = ("numpy.typing")))]
    ecef_uvw: PyArrayLikeDyn<'py, f64, AllowTypeChange>,
    lat_ref_d: f64,
    lon_ref_d: f64,
) -> PyResult<Bound<'py, PyArray2<f64>>> {
    let v = rows_to_dvec3s(&ecef_uvw, "ecef_uvw")?;
    let r = glam::dvec3(lat_ref_d, lon_ref_d, 0.);
    execute_in_local_thread_pool(py, || map3d::ecef_uvw2aer_batch(&v, r))
}

/// Batched `ecef2aer`: converts each row of `ecef` from ECEF positions in meters to
/// AER [degrees-degrees-meters].
///
/// # Arguments
///
/// * `ecef` - Array-like of shape (N, 3) of ECEF positions in meters [numpy.typing.ArrayLike]
/// * `lat_ref_d` - Reference WGS84 Lat in degrees [float]
/// * `lon_ref_d` - Reference WGS84 Lon in degrees [float]
/// * `alt_ref_m` - Reference WGS84 altitude in meters, defaults to 0 [float]
///
/// # Returns
///
/// * Array of shape (N, 3) of AER [degrees-degrees-meters] [numpy.ndarray[float64]]
///
/// # Raises
///
/// * `ValueError` if `ecef` is not of shape (N, 3)
#[gen_stub_pyfunction]
#[pyfunction]
#[pyo3(signature = (ecef, lat_ref_d, lon_ref_d, alt_ref_m=0.0))]
pub fn ecef2aer_batch<'py>(
    py: Python<'py>,
    #[gen_stub(override_type(type_repr = "numpy.typing.ArrayLike", imports = ("numpy.typing")))]
    ecef: PyArrayLikeDyn<'py, f64, AllowTypeChange>,
    lat_ref_d: f64,
    lon_ref_d: f64,
    alt_ref_m: f64,
) -> PyResult<Bound<'py, PyArray2<f64>>> {
    let v = rows_to_dvec3s(&ecef, "ecef")?;
    let r = glam::dvec3(lat_ref_d, lon_ref_d, alt_ref_m);
    execute_in_local_thread_pool(py, || map3d::ecef2aer_batch(&v, r))
}

/// Batched `aer2ecef_uvw`: converts each row of `aer` from AER [degrees-degrees-meters] to
/// ECEF uvw vectors in meters.
///
/// # Arguments
///
/// * `aer` - Array-like of shape (N, 3) of AER [degrees-degrees-meters] [numpy.typing.ArrayLike]
/// * `lat_ref_d` - Reference WGS84 Lat in degrees [float]
/// * `lon_ref_d` - Reference WGS84 Lon in degrees [float]
///
/// # Returns
///
/// * Array of shape (N, 3) of ECEF uvw vectors in meters [numpy.ndarray[float64]]
///
/// # Raises
///
/// * `ValueError` if `aer` is not of shape (N, 3)
#[gen_stub_pyfunction]
#[pyfunction]
pub fn aer2ecef_uvw_batch<'py>(
    py: Python<'py>,
    #[gen_stub(override_type(type_repr = "numpy.typing.ArrayLike", imports = ("numpy.typing")))]
    aer: PyArrayLikeDyn<'py, f64, AllowTypeChange>,
    lat_ref_d: f64,
    lon_ref_d: f64,
) -> PyResult<Bound<'py, PyArray2<f64>>> {
    let v = rows_to_dvec3s(&aer, "aer")?;
    let r = glam::dvec3(lat_ref_d, lon_ref_d, 0.);
    execute_in_local_thread_pool(py, || map3d::aer2ecef_uvw_batch(&v, r))
}

/// Batched `aer2ecef`: converts each row of `aer` from AER [degrees-degrees-meters] to
/// ECEF positions in meters.
///
/// # Arguments
///
/// * `aer` - Array-like of shape (N, 3) of AER [degrees-degrees-meters] [numpy.typing.ArrayLike]
/// * `lat_ref_d` - Reference WGS84 Lat in degrees [float]
/// * `lon_ref_d` - Reference WGS84 Lon in degrees [float]
/// * `alt_ref_m` - Reference WGS84 altitude in meters, defaults to 0 [float]
///
/// # Returns
///
/// * Array of shape (N, 3) of ECEF positions in meters [numpy.ndarray[float64]]
///
/// # Raises
///
/// * `ValueError` if `aer` is not of shape (N, 3)
#[gen_stub_pyfunction]
#[pyfunction]
#[pyo3(signature = (aer, lat_ref_d, lon_ref_d, alt_ref_m=0.0))]
pub fn aer2ecef_batch<'py>(
    py: Python<'py>,
    #[gen_stub(override_type(type_repr = "numpy.typing.ArrayLike", imports = ("numpy.typing")))]
    aer: PyArrayLikeDyn<'py, f64, AllowTypeChange>,
    lat_ref_d: f64,
    lon_ref_d: f64,
    alt_ref_m: f64,
) -> PyResult<Bound<'py, PyArray2<f64>>> {
    let v = rows_to_dvec3s(&aer, "aer")?;
    let r = glam::dvec3(lat_ref_d, lon_ref_d, alt_ref_m);
    execute_in_local_thread_pool(py, || map3d::aer2ecef_batch(&v, r))
}

/// Batched `enu2aer`: converts each row of `enu` from ENU vectors in meters to
/// AER [degrees-degrees-meters].
///
/// # Arguments
///
/// * `enu` - Array-like of shape (N, 3) of ENU vectors in meters [numpy.typing.ArrayLike]
///
/// # Returns
///
/// * Array of shape (N, 3) of AER [degrees-degrees-meters] [numpy.ndarray[float64]]
///
/// # Raises
///
/// * `ValueError` if `enu` is not of shape (N, 3)
#[gen_stub_pyfunction]
#[pyfunction]
pub fn enu2aer_batch<'py>(
    py: Python<'py>,
    #[gen_stub(override_type(type_repr = "numpy.typing.ArrayLike", imports = ("numpy.typing")))]
    enu: PyArrayLikeDyn<'py, f64, AllowTypeChange>,
) -> PyResult<Bound<'py, PyArray2<f64>>> {
    let v = rows_to_dvec3s(&enu, "enu")?;
    execute_in_local_thread_pool(py, || map3d::enu2aer_batch(&v))
}

/// Batched `aer2enu`: converts each row of `aer` from AER [degrees-degrees-meters] to
/// ENU vectors in meters.
///
/// # Arguments
///
/// * `aer` - Array-like of shape (N, 3) of AER [degrees-degrees-meters] [numpy.typing.ArrayLike]
///
/// # Returns
///
/// * Array of shape (N, 3) of ENU vectors in meters [numpy.ndarray[float64]]
///
/// # Raises
///
/// * `ValueError` if `aer` is not of shape (N, 3)
#[gen_stub_pyfunction]
#[pyfunction]
pub fn aer2enu_batch<'py>(
    py: Python<'py>,
    #[gen_stub(override_type(type_repr = "numpy.typing.ArrayLike", imports = ("numpy.typing")))]
    aer: PyArrayLikeDyn<'py, f64, AllowTypeChange>,
) -> PyResult<Bound<'py, PyArray2<f64>>> {
    let v = rows_to_dvec3s(&aer, "aer")?;
    execute_in_local_thread_pool(py, || map3d::aer2enu_batch(&v))
}

/// Batched `ned2aer`: converts each row of `ned` from NED vectors in meters to
/// AER [degrees-degrees-meters].
///
/// # Arguments
///
/// * `ned` - Array-like of shape (N, 3) of NED vectors in meters [numpy.typing.ArrayLike]
///
/// # Returns
///
/// * Array of shape (N, 3) of AER [degrees-degrees-meters] [numpy.ndarray[float64]]
///
/// # Raises
///
/// * `ValueError` if `ned` is not of shape (N, 3)
#[gen_stub_pyfunction]
#[pyfunction]
pub fn ned2aer_batch<'py>(
    py: Python<'py>,
    #[gen_stub(override_type(type_repr = "numpy.typing.ArrayLike", imports = ("numpy.typing")))]
    ned: PyArrayLikeDyn<'py, f64, AllowTypeChange>,
) -> PyResult<Bound<'py, PyArray2<f64>>> {
    let v = rows_to_dvec3s(&ned, "ned")?;
    execute_in_local_thread_pool(py, || map3d::ned2aer_batch(&v))
}

/// Batched `aer2ned`: converts each row of `aer` from AER [degrees-degrees-meters] to
/// NED vectors in meters.
///
/// # Arguments
///
/// * `aer` - Array-like of shape (N, 3) of AER [degrees-degrees-meters] [numpy.typing.ArrayLike]
///
/// # Returns
///
/// * Array of shape (N, 3) of NED vectors in meters [numpy.ndarray[float64]]
///
/// # Raises
///
/// * `ValueError` if `aer` is not of shape (N, 3)
#[gen_stub_pyfunction]
#[pyfunction]
pub fn aer2ned_batch<'py>(
    py: Python<'py>,
    #[gen_stub(override_type(type_repr = "numpy.typing.ArrayLike", imports = ("numpy.typing")))]
    aer: PyArrayLikeDyn<'py, f64, AllowTypeChange>,
) -> PyResult<Bound<'py, PyArray2<f64>>> {
    let v = rows_to_dvec3s(&aer, "aer")?;
    execute_in_local_thread_pool(py, || map3d::aer2ned_batch(&v))
}

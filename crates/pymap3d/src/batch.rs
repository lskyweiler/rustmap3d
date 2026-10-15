//! NumPy bindings for the batched transforms.
//!
//! Every function takes anything NumPy can convert to a float64 array of shape (N, 3), one point
//! per row, and returns a new (N, 3) float64 array. Results match calling the single-point
//! function on each row. The conversion runs without holding the Python GIL.
use glam;
use numpy::{ndarray::Array2, AllowTypeChange, PyArray2, PyArrayLikeDyn, PyUntypedArrayMethods};
use pyo3::{exceptions::PyValueError, prelude::*};
use pyo3_stub_gen::derive::*;

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
    let out = py.detach(|| map3d::ecef2lla_batch(&v));
    return Ok(dvec3s_to_rows(py, out));
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
    let out = py.detach(|| map3d::lla2ecef_batch(&v));
    return Ok(dvec3s_to_rows(py, out));
}

#[cfg(feature = "pyo3")]
use crate::dvec3;
use crate::{traits::*, transforms::*, DVec3};
#[cfg(feature = "pydantic-serde")]
use crate::{utils, validator_wrapper_fn};
#[allow(unused_imports)]
#[cfg(feature = "bevy")]
use bevy::prelude::*;
#[cfg(feature = "pyo3")]
use either::Either;
#[cfg(not(feature = "pyo3"))]
use map3d_derive::*;
#[allow(unused_imports)]
#[cfg(feature = "pyo3")]
use pyo3::{exceptions::PyValueError, prelude::*, types::*};
#[cfg(feature = "pyo3")]
use pyo3_stub_gen::derive::*;
#[cfg(any(feature = "py-bevy", feature = "gen-to-owned-stubs"))]
use simple_py_bevy::*;
use std::ops::{Div, Mul};

/// Represents a vector relative to a reference point
#[derive(Clone, Copy, Default, PartialEq)]
#[cfg_attr(feature = "pyo3", gen_stub_pyclass)]
#[cfg_attr(
    all(feature = "pyo3", feature = "set-pyclass-module"),
    pyclass(module = "rustmap3d")
)]
#[cfg_attr(all(feature = "pyo3", not(feature = "set-pyclass-module")), pyclass)]
#[cfg_attr(not(feature = "pyo3"), derive(DummyPyO3))]
#[cfg_attr(
    all(feature = "py-bevy", feature = "pyo3"),
    derive(PyBevyCompRef, PyStructRef)
)]
#[cfg_attr(feature = "gen-to-owned-stubs", derive(PyToOwnedStub))]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
#[cfg_attr(feature = "bevy", derive(Component, Reflect), reflect(Component))]
pub struct GeoVector {
    #[
        cfg_attr(
            all(feature = "py-bevy", feature = "pyo3"), 
            py_bevy(
                get_ref = pyglam::DVec3Ref,
                other_set_type = pyglam::DVec3Ref
            )
        )
    ]
    #[pyo3(get, set)]
    ecef_uvw: DVec3,
    lla_ref: (f64, f64, f64),
}

impl GeoVector {
    pub fn ecef_uvw(&self) -> &DVec3 {
        &self.ecef_uvw
    }
}

#[cfg(all(feature = "pydantic-serde", feature = "pyo3"))]
validator_wrapper_fn!(GeoVector, "GeoVector"); // need to create a non-classmethod validator for pydantic to hook into
#[cfg(all(feature = "pydantic-serde", feature = "pyo3"))]
fn get_pydantic_json_schema_input_schema<'py>(py: Python<'py>) -> PyResult<Py<PyAny>> {
    /*
    json_schema_input_schema=core_schema.model_schema(
            GeoPosition,
            core_schema.model_fields_schema(
                fields={
                    "ecef": core_schema.model_field(
                        schema=dvec_3_schema
                    )
                },
            ),
        ),
    */

    use pyo3::PyTypeInfo;
    use std::collections::HashMap;

    let pydantic = utils::pydantic::Pydantic::new(py)?;
    let lla_schema = utils::pydantic::create_lat_lon_tuple_schema(py)?;
    let dvec3_schema = utils::pydantic::create_dvec3_schema(py)?;

    let desc =
        utils::pydantic::create_pydantic_schema_description(py, "Relative ECEF vector in meters")?;
    let desc_kwargs = HashMap::from([desc]).into_py_dict(py)?;
    let ecef_schema = pydantic
        .model_field_fn
        .call((dvec3_schema,), Some(&desc_kwargs))?
        .unbind();
    let lla_schema = pydantic.model_field_fn.call1((lla_schema,))?.unbind();

    let model_field_schema_kwargs = HashMap::from([("ecef", ecef_schema), ("lla_ref", lla_schema)]);
    let fields = pydantic
        .model_fields_schema_fn
        .call1((model_field_schema_kwargs,))?;
    let geo_vec_class = GeoVector::type_object(py).unbind();

    let out = pydantic.model_schema_fn.call1((geo_vec_class, fields))?;
    Ok(out.unbind())
}

#[cfg_attr(
    all(feature = "py-bevy", feature = "pyo3"),
    py_bevy_methods,
    py_ref_methods
)]
#[cfg_attr(feature = "pyo3", gen_stub_pymethods, pymethods)]
impl GeoVector {
    /// Load from a json string
    /// ```
    /// {
    ///     "ecef_uvw": [
    ///         119962.85915496295,
    ///         -5189589.602611365,
    ///         3693569.6778840856
    ///     ],
    ///     "lla_ref": [0., 0., 0.]
    /// }
    /// ```
    #[cfg(all(feature = "serde", feature = "pyo3"))]
    #[staticmethod]
    fn model_validate_json(json_str: &str) -> PyResult<Self> {
        match serde_json::from_str(json_str) {
            Ok(loaded) => Ok(loaded),
            Err(what) => Err(PyValueError::new_err(format!("{}", what))),
        }
    }
    /// Load from a json python dict
    /// ```
    /// {
    ///     "ecef_uvw": [
    ///         119962.85915496295,
    ///         -5189589.602611365,
    ///         3693569.6778840856
    ///     ],
    ///     "lla_ref": [0., 0., 0.]
    /// }
    /// ```
    #[cfg(all(feature = "serde", feature = "pyo3"))]
    #[staticmethod]
    fn model_validate<'py>(py: Python<'py>, json_dict: Py<PyDict>) -> PyResult<Self> {
        let s = crate::utils::pydict_to_dump(py, json_dict)?;
        Self::model_validate_json(&s)
    }

    /// Dump to a json string
    /// # Examples
    ///
    /// ```
    /// {
    ///     "ecef_uvw": [
    ///         119962.85915496295,
    ///         -5189589.602611365,
    ///         3693569.6778840856
    ///     ],
    ///     "lla_ref": [0., 0., 0.]
    /// }
    /// ```
    #[cfg(all(feature = "serde", feature = "pyo3"))]
    fn model_dump_json(&self) -> PyResult<String> {
        match serde_json::to_string(&self) {
            Ok(s) => Ok(s),
            Err(what) => Err(PyValueError::new_err(format!("{}", what))),
        }
    }
    /// Dump to a python dict
    /// # Examples
    ///
    /// ```
    /// {
    ///     "ecef_uvw": [
    ///         119962.85915496295,
    ///         -5189589.602611365,
    ///         3693569.6778840856
    ///     ],
    ///     "lla_ref": [0., 0., 0.]
    /// }
    /// ```
    #[cfg(all(feature = "serde", feature = "pyo3"))]
    fn model_dump<'py>(&self, py: Python<'py>) -> PyResult<Py<PyAny>> {
        let s = self.model_dump_json()?;
        crate::utils::dump_to_pydict(py, &s)
    }
    /// Pydantic hook
    /// Allows this to be used as-is in pydantic basemodels
    ///
    /// * Example
    /// ```python,no_run
    /// class MyModel(pydantic.BaseModel):
    ///     vec: rustmap3d.GeoVector
    ///
    /// dumped = MyModel(...).model_dump_json()
    /// loaded = MyModel.model_validate_json(dumped)
    /// ```
    // * Note: it would be nice to macro_rules! this away, but pyo3 does not allow macros inside impl blocks
    // *        and we dont want to use multiple-pymethods since that will break the pyo3-stub-generation
    #[cfg(all(feature = "pydantic-serde", feature = "pyo3"))]
    #[classmethod]
    fn __get_pydantic_core_schema__<'py>(
        cls: &Bound<'_, PyType>,
        py: Python<'py>,
        _source: Py<PyAny>,
        _handler: Py<PyAny>,
    ) -> PyResult<Py<PyAny>> {
        let validator = PyCFunction::new_closure(
            py,
            None,
            None,
            |args: &Bound<'_, PyTuple>,
             _kwargs: Option<&Bound<'_, PyDict>>|
             -> PyResult<Py<PyAny>> {
                let py = args.py();
                let first = args.get_item(0)?;
                validate_obj(py, first.unbind())
            },
        )
        .unwrap();
        let validator = validator.as_any().clone().unbind();
        let serializer = cls.getattr("model_dump")?.unbind();

        let json_schema_input_schema = get_pydantic_json_schema_input_schema(py)?;

        utils::pydantic::create_pydantic_core_schema(
            py,
            validator,
            serializer,
            json_schema_input_schema,
        )
    }
    #[cfg(all(feature = "pydantic-serde", feature = "pyo3"))]
    #[classattr]
    fn model_config<'py>(py: Python<'py>) -> PyResult<Py<PyAny>> {
        if py.import("pydantic").is_err() {
            // if pydantic is not installed dont require it
            let none = PyNone::get(py);
            let none_any = none.to_owned().into_any();
            return Ok(none_any.unbind());
        }
        utils::pydantic::create_pydantic_model_config(py)
    }

    /// Create a Vector from an ecef vector relative to a reference point
    ///
    /// # Arguments
    ///
    /// - `ecef_uvw` (`&DVec3`) - Vector in ecef frame in meters
    /// - `reference` (`EitherGeoPosOrLLATup`) - Reference geo location
    ///
    #[staticmethod]
    pub fn from_ecef(ecef_uvw: &DVec3, reference: EitherGeoPosOrLLATup) -> Self {
        Self {
            ecef_uvw: ecef_uvw.clone(),
            lla_ref: reference.into_lat_lon_triple(),
        }
    }
    /// Create a Vector from an enu vector relative to a reference point
    ///
    /// # Arguments
    ///
    /// - `enu` (`&DVec3`) - enu vector in meters
    /// - `reference` (`EitherGeoPosOrLLATup`) - Reference geo location
    ///
    #[staticmethod]
    pub fn from_enu(enu: &DVec3, reference: EitherGeoPosOrLLATup) -> Self {
        let lla_ref = reference.into_lat_lon_triple();
        Self {
            ecef_uvw: enu2ecef_uvw(enu, &lla_ref).into(),
            lla_ref: lla_ref,
        }
    }
    /// Create a Vector from an ned vector relative to a reference point
    ///
    /// # Arguments
    ///
    /// - `ned` (`&DVec3`) - ned vector in meters
    /// - `reference` (`EitherGeoPosOrLLATup`) - Reference geo location
    ///
    #[staticmethod]
    pub fn from_ned(ned: &DVec3, reference: EitherGeoPosOrLLATup) -> Self {
        let lla_ref = reference.into_lat_lon_triple();
        Self {
            ecef_uvw: ned2ecef_uvw(ned, lla_ref).into(),
            lla_ref: lla_ref,
        }
    }
    /// Create a Vector from an ned vector relative to a reference point
    ///
    /// # Arguments
    ///
    /// - `ned` (`&DVec3`) - ned vector in meters
    /// - `reference` (`EitherGeoPosOrLLATup`) - Reference geo location
    ///
    #[staticmethod]
    pub fn from_aer(aer: &DVec3, reference: EitherGeoPosOrLLATup) -> Self {
        let lla_ref = reference.into_lat_lon_triple();
        Self {
            ecef_uvw: aer2ecef_uvw(aer, lla_ref).into(),
            lla_ref: lla_ref,
        }
    }

    /// Compute the length of this vector in meters
    ///
    /// # Returns
    ///
    /// * `length` - Length of this vector in [[meters]]
    pub fn length(&self) -> f64 {
        self.ecef_uvw.length()
    }

    /// Get the absolute ecef position of this vector
    ///
    /// # Returns
    ///
    /// - `DVec3` - Absolute ecef position in meters
    ///
    pub fn ecef(&self) -> DVec3 {
        self.ecef_uvw + lla2ecef(self.lla_ref)
    }

    /// Get this vector as east north up relative to its reference
    pub fn enu(&self) -> DVec3 {
        ecef_uvw2enu(&self.ecef_uvw, &self.lla_ref).into()
    }
    /// Get this vector as north east down relative to its reference
    pub fn ned(&self) -> DVec3 {
        ecef_uvw2ned(&self.ecef_uvw, &self.lla_ref).into()
    }
    /// Get this vector as az el range relative to its reference
    pub fn aer(&self) -> DVec3 {
        ecef_uvw2aer(&self.ecef_uvw, &self.lla_ref).into()
    }

    /// Get the distance in meters this vector goes north from its reference point
    pub fn north(&self) -> f64 {
        self.enu().y
    }
    /// Get the distance in meters this vector goes south from its reference point
    pub fn south(&self) -> f64 {
        -self.enu().y
    }
    /// Get the distance in meters this vector goes east from its reference point
    pub fn east(&self) -> f64 {
        self.enu().x
    }
    /// Get the distance in meters this vector goes west from its reference point
    pub fn west(&self) -> f64 {
        -self.enu().x
    }
    /// Get the distance in meters this vector goes away from earth from its reference point
    pub fn up(&self) -> f64 {
        self.enu().z
    }
    /// Get the distance in meters this vector towards the earth from its reference point
    pub fn down(&self) -> f64 {
        self.ned().z
    }

    /// Compute the clockwise angle off true north for this vector relative to its reference
    /// Angle is always between [0., 360.]
    ///
    /// # Returns
    ///
    /// * `azimuth` - Clockwise angle off true north in [[degrees]]
    pub fn azimuth(&self) -> f64 {
        self.aer().x
    }

    /// Scale this vector with a float
    /// # Arguments
    ///
    /// - `rhs` (`f64`) - Scale to multiply
    ///
    /// # Returns
    ///
    /// - `PyResult<GeoVector>` - Scaled GeoVector
    ///
    #[cfg(feature = "pyo3")]
    fn __mul__(&self, rhs: f64) -> PyResult<GeoVector> {
        Ok(self * rhs)
    }
    #[cfg(feature = "pyo3")]
    fn __rmul__(&self, rhs: f64) -> PyResult<GeoVector> {
        self.__mul__(rhs)
    }
    /// Shrink this vector with a float
    #[cfg(feature = "pyo3")]
    fn __div__(&self, rhs: f64) -> PyResult<GeoVector> {
        Ok(self / rhs)
    }

    /// Constructor for pickle/deepcopy support
    #[cfg(feature = "pyo3")]
    #[new]
    fn py_new(ecef_uvw: Either<DVec3, (f64, f64, f64)>, lla_ref: (f64, f64, f64)) -> Self {
        match ecef_uvw {
            Either::Left(vec) => GeoVector {
                ecef_uvw: vec,
                lla_ref,
            },
            Either::Right(tup) => GeoVector {
                ecef_uvw: dvec3(tup.0, tup.1, tup.2),
                lla_ref,
            },
        }
    }

    /// Support for pickle/deepcopy
    #[cfg(feature = "pyo3")]
    fn __getnewargs__(&self) -> PyResult<((f64, f64, f64), (f64, f64, f64))> {
        Ok((
            (self.ecef_uvw.x, self.ecef_uvw.y, self.ecef_uvw.z),
            self.lla_ref,
        ))
    }
}

macro_rules! geo_vec_scale {
    ($a:ty, $b:ty) => {
        impl Mul<$a> for $b {
            type Output = GeoVector;
            fn mul(self, scale: $a) -> Self::Output {
                let scaled = self.ecef_uvw * scale;
                GeoVector::from_ecef(&scaled, self.lla_ref.into_either())
            }
        }
        impl Div<$a> for $b {
            type Output = GeoVector;
            fn div(self, scale: $a) -> Self::Output {
                let scaled = self.ecef_uvw / scale;
                GeoVector::from_ecef(&scaled, self.lla_ref.into_either())
            }
        }
    };
}
geo_vec_scale!(f64, GeoVector);
geo_vec_scale!(&f64, GeoVector);
geo_vec_scale!(f64, &GeoVector);
geo_vec_scale!(&f64, &GeoVector);

#[cfg(test)]
mod test_geo_vector {
    use super::*;

    mod test_constructors {
        use super::*;

        #[test]
        fn test_from_ecef() {
            let actual =
                GeoVector::from_ecef(&pyglam::dvec3(100., 0., 0.), (0., 0., 0.).into_either());
            assert!(actual
                .ecef_uvw()
                .abs_diff_eq(glam::dvec3(100., 0., 0.), 1e-10));
        }
        #[test]
        fn test_from_enu() {
            let actual =
                GeoVector::from_enu(&pyglam::dvec3(100., 0., 0.), (0., 0., 0.).into_either());
            assert!(actual.enu().abs_diff_eq(glam::dvec3(100., 0., 0.), 1e-10));
        }
        #[test]
        fn test_from_ned() {
            let actual =
                GeoVector::from_ned(&pyglam::dvec3(100., 0., 0.), (0., 0., 0.).into_either());
            assert!(actual.ned().abs_diff_eq(glam::dvec3(100., 0., 0.), 1e-10));
        }
        #[test]
        fn test_from_aer() {
            let actual =
                GeoVector::from_aer(&pyglam::dvec3(100., 0., 100.), (0., 0., 0.).into_either());
            assert!(actual.aer().abs_diff_eq(glam::dvec3(100., 0., 100.), 1e-10));
        }
    }

    mod test_utils {
        use super::*;

        #[test]
        fn test_cardinals() {
            let actual =
                GeoVector::from_ecef(&pyglam::dvec3(100., 50., -100.), (0., 0., 0.).into_either());

            almost::equal_with(actual.north(), -100., 1e-10);
            almost::equal_with(actual.south(), 100., 1e-10);
            almost::equal_with(actual.east(), 50., 1e-10);
            almost::equal_with(actual.west(), -50., 1e-10);
            almost::equal_with(actual.up(), 100., 1e-10);
            almost::equal_with(actual.down(), -100., 1e-10);
        }

        #[test]
        fn test_angles() {
            // vector due east
            let actual =
                GeoVector::from_ecef(&pyglam::dvec3(0., 100., 0.), (0., 0., 0.).into_either());
            almost::equal_with(actual.azimuth(), 90., 1e-10);
        }

        #[test]
        fn test_length() {
            let actual =
                GeoVector::from_ecef(&pyglam::dvec3(-100., 0., 0.), (0., 0., 0.).into_either());
            almost::equal_with(actual.length(), 100., 1e-10);
        }
    }
}

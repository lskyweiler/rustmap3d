#[cfg(feature = "pydantic-serde")]
use crate::validator_wrapper_fn;
use crate::{
    geo_objects::geo_vector::GeoVector, traits::*, transforms::*, utils, vincenty::*, DVec3,
};
#[allow(unused_imports)]
#[cfg(feature = "bevy")]
use bevy::prelude::*;
use either::Either;
#[allow(unused_imports)]
use glam::{self, swizzles::*};
#[cfg(not(feature = "pyo3"))]
use map3d_derive::*;
#[allow(unused_imports)]
#[cfg(feature = "pyo3")]
use pyo3::{exceptions::PyValueError, prelude::*, types::*};
#[cfg(feature = "pyo3")]
use pyo3_stub_gen::derive::*;
#[cfg(feature = "py-bevy")]
use simple_py_bevy::*;
use std::{
    fmt::Debug,
    ops::{Add, Sub},
};

pub type EitherGeoPosOrLLATup = Either<(f64, f64, f64), GeoPosition>;
impl Into<EitherGeoPosOrLLATup> for GeoPosition {
    fn into(self) -> EitherGeoPosOrLLATup {
        Either::Right(self)
    }
}
impl Into<EitherGeoPosOrLLATup> for &GeoPosition {
    fn into(self) -> EitherGeoPosOrLLATup {
        Either::Right(self.clone())
    }
}

/// Represents a position on the earth
#[derive(Clone, Copy, Default, PartialEq)]
#[cfg_attr(feature = "pyo3", gen_stub_pyclass, pyclass)]
#[cfg_attr(not(feature = "pyo3"), derive(DummyPyO3))]
#[cfg_attr(
    all(feature = "py-bevy", feature = "pyo3"),
    derive(PyBevyCompRef, PyStructRef)
)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
#[cfg_attr(
    feature = "bevy",
    derive(Component, Reflect),
    reflect(Component, Clone)
)]
#[repr(transparent)]
pub struct GeoPosition {
    // Store the position in an [ecef](https://en.wikipedia.org/wiki/Earth-centered,_Earth-fixed_coordinate_system) vector since this is the most exact representation
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
    ecef: DVec3,
}

impl GeoPosition {
    pub fn ecef(&self) -> &DVec3 {
        &self.ecef
    }
    pub fn ecef_mut(&mut self) -> &mut DVec3 {
        return &mut self.ecef;
    }
    pub fn lla(&self) -> (f64, f64, f64) {
        ecef2lla(&self.ecef).into()
    }
}

#[cfg(all(feature = "pydantic-serde", feature = "pyo3"))]
validator_wrapper_fn!(GeoPosition, "GeoPosition"); // need to create a non-classmethod validator for pydantic to hook into
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
    let dvec3_schema = utils::pydantic::create_dvec3_schema(py)?;

    let desc = utils::pydantic::create_pydantic_schema_description(py, "ECEF location in meters")?;
    let desc_kwargs = HashMap::from([desc]).into_py_dict(py)?;
    let ecef_schema = pydantic
        .model_field_fn
        .call((dvec3_schema,), Some(&desc_kwargs))?;

    let model_field_schema_kwargs = HashMap::from([("ecef", ecef_schema)]);
    let fields = pydantic
        .model_fields_schema_fn
        .call1((model_field_schema_kwargs,))?;
    let geo_pos_class = GeoPosition::type_object(py).unbind();

    let out = pydantic.model_schema_fn.call1((geo_pos_class, fields))?;
    Ok(out.unbind())
}

#[cfg_attr(
    all(feature = "py-bevy", feature = "pyo3"),
    py_bevy_methods,
    py_ref_methods
)]
#[cfg_attr(feature = "pyo3", gen_stub_pymethods, pymethods)]
impl GeoPosition {
    /// Load from a json string
    /// ```
    /// "{'ecef':[0.,0.,0.]}"
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
    ///     "ecef": [
    ///         119962.85915496295,
    ///         -5189589.602611365,
    ///         3693569.6778840856
    ///     ]
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
    /// "{'ecef':[0.,0.,0.]}"
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
    ///     "ecef": [
    ///         119962.85915496295,
    ///         -5189589.602611365,
    ///         3693569.6778840856
    ///     ]
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
    ///     pos: rustmap3d.GeoPosition
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
        utils::pydantic::create_pydantic_model_config(py)
    }

    /// Construct a GeoPosition from an ECEF (Earth Centered, Earth Fixed) vec3 in meters
    ///
    /// # Arguments
    ///
    /// - `ecef` (`DVec3`) - ECEF location in meters
    #[new]
    fn py_new(ecef_m: Either<DVec3, (f64, f64, f64)>) -> Self {
        match ecef_m {
            Either::Left(vec) => Self::from_ecef(&vec),
            Either::Right(tup) => Self::from_ecef(&tup.into_dvec3().into()),
        }
    }

    /// Construct a GeoPosition from an ECEF (Earth Centered, Earth Fixed) vec3 in meters
    ///
    /// # Arguments
    ///
    /// - `ecef` (`DVec3`) - ECEF location in meters
    ///
    #[staticmethod]
    pub fn from_ecef(ecef_m: &DVec3) -> Self {
        Self {
            ecef: ecef_m.clone(),
        }
    }
    /// Construct a GeoPosition from a WGS84 Latitude, Longitude, Altitude in deg,deg,meters
    ///
    /// # Arguments
    ///
    /// - `lla` (`(float, float, float)`) - WGS84 lat, lon, alt in [[degrees, degrees, meters]]
    ///
    #[staticmethod]
    pub fn from_lla(lla_ddm: (f64, f64, f64)) -> Self {
        Self {
            ecef: lla2ecef(lla_ddm).into(),
        }
    }
    /// Construct a GeoPosition from a local east, north, up vector in meters relative to a reference location
    ///
    /// # Arguments
    ///
    /// - `enu_m` (`&DVec3`) - East, North, Up vector in meters
    /// - `reference` (`EitherGeoPosOrLLATup`) - Reference location
    ///
    #[staticmethod]
    pub fn from_enu(enu_m: &DVec3, reference: EitherGeoPosOrLLATup) -> Self {
        Self {
            ecef: enu2ecef(enu_m.into_dvec3(), reference).into(),
        }
    }
    /// Construct a GeoPosition from a local north, east, down vector in meters relative to a reference location
    ///
    /// # Arguments
    ///
    /// - `ned_m` (`&DVec3`) - North, East, Down vector in meters
    /// - `reference` (`EitherGeoPosOrLLATup`) - Reference location
    ///
    #[staticmethod]
    pub fn from_ned(ned_m: &DVec3, reference: EitherGeoPosOrLLATup) -> Self {
        Self {
            ecef: ned2ecef(ned_m.into_dvec3(), reference).into(),
        }
    }
    /// Construct a GeoPosition from a local az, el, range vector in deg,deg,meters relative to a reference location
    ///
    /// # Arguments
    ///
    /// - `aer_ddm` (`&DVec3`) - Azimuth, Elevation, Range in deg,deg,meters
    /// - `reference` (`EitherGeoPosOrLLATup`) - Reference location
    ///
    #[staticmethod]
    pub fn from_aer(aer_ddm: &DVec3, reference: EitherGeoPosOrLLATup) -> Self {
        Self {
            ecef: aer2ecef(aer_ddm.into_dvec3(), reference).into(),
        }
    }
    /// Get the MSL altitude in meters
    pub fn alt(&self) -> f64 {
        self.lla().2
    }
    ///  Sets the altitude of this position while preserving the lat/lon
    ///
    /// # Arguments
    ///
    /// - `alt_m` (`f64`) - New MSL altitude in meters
    pub fn set_alt(&mut self, alt_m: f64) {
        let lla = self.lla();
        self.ecef = lla2ecef((lla.0, lla.1, alt_m)).into();
    }
    /// Gets the WGS84 Latitude, Longitude, and Altitude of this position
    ///
    /// # Arguments
    ///
    /// - `&self` (`undefined`) - Describe this parameter.
    ///
    /// # Returns
    ///
    /// - `(f64, f64, f64)` - (lat, lon, alt) in (deg, deg, meters)
    ///
    #[cfg(feature = "pyo3")]
    #[getter]
    fn get_lla(&self) -> (f64, f64, f64) {
        self.lla()
    }

    /// Compute the bearing to another geoposition
    ///
    /// This uses the vincenty inverse method
    ///
    /// # Arguments
    ///
    /// - `to_point` (`&GeoPosition`) - Target position
    ///
    /// # Returns
    ///
    /// - `f64` - Clockwise angle off true north in degrees
    ///
    pub fn bearing_to(&self, to_point: &GeoPosition) -> f64 {
        let lla_a = self.lla();
        let lla_b = to_point.lla();

        // inputs to this should never be invalid since we're sending geo positions
        let (_, bearing, _) =
            vincenty_inverse(lla_a.0, lla_a.1, lla_b.0, lla_b.1, 1e-10, 200).unwrap();

        return utils::wrap_to_0_360(bearing);
    }
    /// Compute the East, North, Up vector from this point to another geo vector
    ///
    /// # Arguments
    ///
    /// - `to_point` (`&GeoPosition`) - target geo point
    ///
    /// # Returns
    ///
    /// - `DVec3` - east north up vector relative to this, in (meters)
    ///
    pub fn enu_to(&self, to_point: &GeoPosition) -> DVec3 {
        ecef2enu(&to_point.ecef, &self.lla()).into()
    }
    /// Compute the Az, El, Range vector from this point to another geo vector
    ///
    /// # Arguments
    ///
    /// - `to_point` (`&GeoPosition`) - target geo point
    ///
    /// # Returns
    ///
    /// - `DVec3` - az, el, range as a 3 component vector in (deg, deg, m)
    ///
    pub fn aer_to(&self, to_point: &GeoPosition) -> DVec3 {
        ecef2aer(&to_point.ecef, &self.lla()).into()
    }
    /// Compute the North East Up vector from this point to another geo vector
    ///
    /// # Arguments
    ///
    /// - `to_point` (`&GeoPosition`) - target geo point
    ///
    /// # Returns
    ///
    /// - `DVec3` - north east up vector relative to this, in (meters)
    ///
    pub fn ned_to(&self, to_point: &GeoPosition) -> DVec3 {
        ecef2ned(&to_point.ecef, &self.lla()).into()
    }

    /// Compute the bearing angle from another point to this one
    ///
    /// This uses the vincenty inverse algorithm
    ///
    /// # Arguments
    ///
    /// - `from_point` (`&GeoPosition`) - Source point
    ///
    /// # Returns
    ///
    /// - `f64` - Clockwise angle off true north from the other point to this one
    ///
    pub fn bearing_from(&self, from_point: &GeoPosition) -> f64 {
        let lla_a = self.lla();
        let lla_b = from_point.lla();

        // inputs to this should never be invalid since we're sending geo positions
        let (_, _, bearing) =
            vincenty_inverse(lla_a.0, lla_a.1, lla_b.0, lla_b.1, 1e-10, 200).unwrap();
        return utils::wrap_to_0_360(bearing);
    }
    /// Compute the east north up vector from another point to this one
    ///
    /// # Arguments
    ///
    /// - `from_point` (`&GeoPosition`) - source point
    ///
    /// # Returns
    ///
    /// - `DVec3` - east north up vector in meters
    ///
    pub fn enu_from(&self, from_point: &GeoPosition) -> DVec3 {
        ecef2enu(&self.ecef, &from_point.lla()).into()
    }
    /// Compute the az el range vector from another point to this one
    ///
    /// # Arguments
    ///
    /// - `from_point` (`&GeoPosition`) - source point
    ///
    /// # Returns
    ///
    /// - `DVec3` - az el range in (deg, deg, meters)
    ///
    pub fn aer_from(&self, from_point: &GeoPosition) -> DVec3 {
        ecef2aer(&self.ecef, &from_point.lla()).into()
    }
    /// Compute the north east, down vector from another point to this one
    ///
    /// # Arguments
    ///
    /// - `from_point` (`&GeoPosition`) - source point
    ///
    /// # Returns
    ///
    /// - `DVec3` - north east down in (meters)
    ///
    pub fn ned_from(&self, from_point: &GeoPosition) -> DVec3 {
        ecef2ned(&self.ecef, &from_point.lla()).into()
    }

    /// Compute the euclidean distance between two geo positions in meters
    ///
    /// # Arguments
    ///
    /// - `to_point` (`&GeoPosition`) - Distance to this point
    ///
    /// # Returns
    ///
    /// - `f64` - Euclidean distance in meters
    ///
    pub fn dist_euclidean(&self, to_point: &GeoPosition) -> f64 {
        (self.ecef - to_point.ecef).length()
    }

    /// Compute the ellipsoidal distance between two geo positions in meters
    ///
    /// # Arguments
    ///
    /// - `to_point` (`&GeoPosition`) - Distance to this point
    ///
    /// # Returns
    ///
    /// - `f64` - Elliptical distance in meters
    ///
    pub fn dist_ellipsoidal(&self, to_point: &GeoPosition) -> f64 {
        let lla_a = self.lla();
        let lla_b = to_point.lla();
        // inputs to this should never be invalid since we're sending geo positions
        let (range, _, _) =
            vincenty_inverse(lla_a.0, lla_a.1, lla_b.0, lla_b.1, 1e-10, 200).unwrap();
        range
    }

    /// Get the lat lon of this point in degrees.minutes.seconds
    ///
    /// # Arguments
    ///
    /// - `&self` (`undefined`) - Describe this parameter.
    ///
    /// # Returns
    ///
    /// - `String` - Ex: "25:22:44.738N, "25:22:44.738E"
    ///
    pub fn lat_lon_dms(&self) -> String {
        let ll = self.lla();
        return format!("{}, {}", dd2dms(ll.0, true), dd2dms(ll.1, false));
    }

    /// Rotate the geo position by an ecef rotation, but preserve the starting altitude
    pub fn rotate_lat_lon(&mut self, ecef_rot: &pyglam::DQuat) {
        let starting_alt = self.alt();
        let new_ecef = ecef_rot * self.ecef;
        let new_lat_lon = ecef2lla(&new_ecef).xy();
        self.ecef = lla2ecef(&glam::dvec3(new_lat_lon.x, new_lat_lon.y, starting_alt)).into();
    }

    /// Python __repr__ to pretty print the geo position as degrees.minutes.seconds
    fn __repr__(&self) -> String {
        format!("{:?}", self)
    }

    /// Adds a relative ecef vector to this position
    ///
    /// # Arguments
    ///
    /// - `rhs` (`Either<GeoVector, DVec3>`) - Either a simple ecef_uvw vector or a GeoVector
    ///
    /// # Returns
    ///
    /// - `PyResult<GeoPosition>` - New position
    ///
    fn __add__(&self, rhs: Either<GeoVector, DVec3>) -> GeoPosition {
        match rhs {
            Either::Left(vec) => self + vec,
            Either::Right(vec) => GeoPosition::from_ecef(&(self.ecef + vec)),
        }
    }
    fn __radd__(&self, rhs: Either<GeoVector, DVec3>) -> GeoPosition {
        self.__add__(rhs)
    }
    /// Subtract a relative ecef vector from this position
    ///
    /// Subtracting a GeoPos from a GeoPos results in a GeoVector
    /// Subtracting a GeoVector from a GeoPos results in a new GeoPos
    ///
    /// # Arguments
    ///
    /// - `rhs` (`Either<GeoVector, GeoPosition>`) - Either another GeoPosition, or a GeoVector
    ///
    /// # Returns
    ///
    /// - `PyResult<Either<GeoVector, GeoPosition>>` - GeoVector or GeoPos
    ///
    #[cfg(feature = "pyo3")]
    fn __sub__(&self, rhs: Either<GeoVector, GeoPosition>) -> Either<GeoVector, GeoPosition> {
        match rhs {
            Either::Left(vec) => Either::Right(self - vec),
            Either::Right(vec) => Either::Left(self - vec),
        }
    }
    #[cfg(feature = "pyo3")]
    fn __rsub__(&self, lhs: GeoPosition) -> GeoVector {
        lhs - self
    }
}

impl Debug for GeoPosition {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", format!("<GeoPosition @ {}>", self.lat_lon_dms()))
    }
}

/// Adding a GeoVector to a GeoPosition results in a new GeoPosition
macro_rules! geo_vec_adds_with_geopos {
    ($a:ty, $b:ty) => {
        impl Add<$a> for $b {
            type Output = GeoPosition;

            fn add(self, rhs: $a) -> Self::Output {
                GeoPosition {
                    ecef: self.ecef() + rhs.ecef_uvw(),
                }
            }
        }
    };
}
geo_vec_adds_with_geopos!(GeoVector, GeoPosition);
geo_vec_adds_with_geopos!(&GeoVector, GeoPosition);
geo_vec_adds_with_geopos!(GeoVector, &GeoPosition);
geo_vec_adds_with_geopos!(&GeoVector, &GeoPosition);

/// Adding an ecef_uvw glam::DVec3 results in a new GeoPosition
macro_rules! dvec3_adds_with_geopos {
    ($a:ty, $b:ty) => {
        impl Add<$a> for $b {
            type Output = GeoPosition;

            fn add(self, rhs: $a) -> Self::Output {
                GeoPosition {
                    ecef: self.ecef() + rhs,
                }
            }
        }
    };
}
dvec3_adds_with_geopos!(glam::DVec3, GeoPosition);
dvec3_adds_with_geopos!(&glam::DVec3, GeoPosition);
dvec3_adds_with_geopos!(glam::DVec3, &GeoPosition);
dvec3_adds_with_geopos!(&glam::DVec3, &GeoPosition);
dvec3_adds_with_geopos!(pyglam::DVec3, GeoPosition);
dvec3_adds_with_geopos!(&pyglam::DVec3, GeoPosition);
dvec3_adds_with_geopos!(pyglam::DVec3, &GeoPosition);
dvec3_adds_with_geopos!(&pyglam::DVec3, &GeoPosition);

/// Subtracting two GeoPositions results in a GeoVector starting at RHS -> LHS
macro_rules! geo_pos_subs_with_geopos {
    ($a:ty, $b:ty) => {
        impl Sub<$a> for $b {
            type Output = GeoVector;

            fn sub(self, rhs: $a) -> Self::Output {
                GeoVector::from_ecef(&(self.ecef - rhs.ecef), rhs.into_either())
            }
        }
    };
}
geo_pos_subs_with_geopos!(GeoPosition, GeoPosition);
geo_pos_subs_with_geopos!(&GeoPosition, GeoPosition);
geo_pos_subs_with_geopos!(GeoPosition, &GeoPosition);
geo_pos_subs_with_geopos!(&GeoPosition, &GeoPosition);

/// Subtracting a GeoVector from a GeoPosition results in a new GeoPosition
macro_rules! geo_vec_subs_with_geopos {
    ($a:ty, $b:ty) => {
        impl Sub<$a> for $b {
            type Output = GeoPosition;

            fn sub(self, rhs: $a) -> Self::Output {
                GeoPosition {
                    ecef: self.ecef() - rhs.ecef_uvw(),
                }
            }
        }
    };
}
geo_vec_subs_with_geopos!(GeoVector, GeoPosition);
geo_vec_subs_with_geopos!(&GeoVector, GeoPosition);
geo_vec_subs_with_geopos!(GeoVector, &GeoPosition);
geo_vec_subs_with_geopos!(&GeoVector, &GeoPosition);

/// Subtracting a ecef_uvw glam::DVec3 results in a new GeoPosition
macro_rules! dvec3_subs_with_geopos {
    ($a:ty, $b:ty) => {
        impl Sub<$a> for $b {
            type Output = GeoPosition;

            fn sub(self, rhs: $a) -> Self::Output {
                GeoPosition {
                    ecef: self.ecef() - rhs,
                }
            }
        }
    };
}
dvec3_subs_with_geopos!(glam::DVec3, GeoPosition);
dvec3_subs_with_geopos!(&glam::DVec3, GeoPosition);
dvec3_subs_with_geopos!(glam::DVec3, &GeoPosition);
dvec3_subs_with_geopos!(&glam::DVec3, &GeoPosition);
dvec3_subs_with_geopos!(pyglam::DVec3, GeoPosition);
dvec3_subs_with_geopos!(&pyglam::DVec3, GeoPosition);
dvec3_subs_with_geopos!(pyglam::DVec3, &GeoPosition);
dvec3_subs_with_geopos!(&pyglam::DVec3, &GeoPosition);

impl From<&GeoPosition> for GeoPosition {
    fn from(value: &GeoPosition) -> Self {
        value.clone()
    }
}
impl From<&GeoVector> for GeoPosition {
    fn from(value: &GeoVector) -> Self {
        Self { ecef: value.ecef() }
    }
}
impl From<GeoVector> for GeoPosition {
    fn from(value: GeoVector) -> Self {
        Self { ecef: value.ecef() }
    }
}

#[cfg(test)]
mod test_geo_pos {
    use super::*;
    use crate::{dvec3, wgs84};

    mod test_constructors {
        use crate::wgs84;

        use super::*;

        #[test]
        fn test_from_ecef() {
            let actual = GeoPosition::from_ecef(&dvec3(wgs84::EARTH_SEMI_MAJOR_AXIS, 0., 0.));
            assert!(actual
                .ecef()
                .abs_diff_eq(glam::dvec3(wgs84::EARTH_SEMI_MAJOR_AXIS, 0., 0.), 1e-10));
        }
        #[test]
        fn test_from_lla() {
            let actual = GeoPosition::from_lla((0., 0., 0.));
            assert!(actual
                .ecef()
                .abs_diff_eq(glam::dvec3(wgs84::EARTH_SEMI_MAJOR_AXIS, 0., 0.), 1e-10));
        }
        #[test]
        fn test_from_enu() {
            let actual = GeoPosition::from_enu(&dvec3(100., 0., 0.), (0., 0., 0.).into_either());
            assert!(actual
                .ecef()
                .abs_diff_eq(glam::dvec3(wgs84::EARTH_SEMI_MAJOR_AXIS, 100., 0.), 1e-10));
        }
        #[test]
        fn test_from_ned() {
            let actual = GeoPosition::from_ned(&dvec3(0., 100., 0.), (0., 0., 0.).into_either());
            assert!(actual
                .ecef()
                .abs_diff_eq(glam::dvec3(wgs84::EARTH_SEMI_MAJOR_AXIS, 100., 0.), 1e-10));
        }
    }

    mod test_to_from {
        use super::*;

        #[test]
        fn test_to() {
            let a = GeoPosition::from_lla((0., 0., 0.));
            let b = GeoPosition::from_ecef(&dvec3(wgs84::EARTH_SEMI_MAJOR_AXIS, 100., 0.));

            assert!(a.enu_to(&b).abs_diff_eq(glam::dvec3(100., 0., 0.), 1e-6));
            assert!(a.ned_to(&b).abs_diff_eq(glam::dvec3(0., 100., 0.), 1e-6));
            assert!(a.aer_to(&b).abs_diff_eq(glam::dvec3(90., 0., 100.), 1e-6));
            almost::equal_with(a.bearing_to(&b), 90., 1e-7);
        }
        #[test]
        fn test_from() {
            let a = GeoPosition::from_lla((0., 0., 0.));
            let b = GeoPosition::from_ecef(&dvec3(wgs84::EARTH_SEMI_MAJOR_AXIS, 100., 0.));

            assert!(a.enu_from(&b).abs_diff_eq(glam::dvec3(-100., 0., 0.), 1e-2));
            assert!(a.ned_from(&b).abs_diff_eq(glam::dvec3(0., -100., 0.), 1e-2));
            assert!(a
                .aer_from(&b)
                .abs_diff_eq(glam::dvec3(270., 0., 100.), 1e-2));
            almost::equal_with(a.bearing_from(&b), -90., 1e-7);
        }
    }

    mod test_lla {
        use super::*;
        use core::f64;

        #[test]
        fn test_lla() {
            let actual = GeoPosition::from_ecef(&dvec3(wgs84::EARTH_SEMI_MAJOR_AXIS, 0., 0.));
            almost::equal_with(actual.lla().0, 0., 1e-5);
            almost::equal_with(actual.lla().1, 0., 1e-5);
            almost::equal_with(actual.lla().2, 0., 1e-5);
        }
        #[test]
        fn test_set_alt() {
            let mut actual = GeoPosition::from_ecef(&dvec3(0., wgs84::EARTH_SEMI_MAJOR_AXIS, 0.)); // vector straight up at 90lon
            actual.set_alt(1000.);

            assert!(actual.ecef().abs_diff_eq(
                glam::dvec3(0., wgs84::EARTH_SEMI_MAJOR_AXIS + 1000., 0.),
                1e-5
            ));
        }

        #[test]
        fn test_rot_alt() {
            let mut actual = GeoPosition::from_lla((0., 0., 100.));
            let rot = glam::DQuat::from_axis_angle(glam::dvec3(0., 0., 1.), f64::consts::PI);
            actual.rotate_lat_lon(&rot.into());

            almost::equal_with(actual.lla().0, 0., 1e-5);
            almost::equal_with(actual.lla().1, 90., 1e-5);
            almost::equal_with(actual.lla().2, 100., 1e-5);
        }
    }

    #[test]
    fn test_distance() {
        let a = GeoPosition::from_ecef(&dvec3(wgs84::EARTH_SEMI_MAJOR_AXIS, 0., 0.));
        let b = GeoPosition::from_ecef(&dvec3(wgs84::EARTH_SEMI_MAJOR_AXIS, 100., 0.));

        almost::equal_with(a.dist_euclidean(&b), 100., 1e-10);
    }

    mod test_ops {
        use super::*;

        mod test_sub {
            use super::*;
            #[test]
            fn test_pos_pos_sub() {
                let rhs = GeoPosition::from_lla((0., 0., 0.));
                let lhs = GeoPosition::from_ecef(&dvec3(wgs84::EARTH_SEMI_MAJOR_AXIS, 1000., 0.));
                let actual = lhs - rhs;

                let actual_enu = actual.enu();
                assert!(actual_enu.abs_diff_eq(glam::dvec3(1000., 0., 0.), 1e-6));
            }
            #[test]
            fn test_pos_dvec3_sub() {
                let lhs = GeoPosition::from_lla((0., 0., 0.));
                let actual = lhs - glam::dvec3(0., 1000., 0.);

                assert!(actual
                    .ecef
                    .abs_diff_eq(glam::dvec3(wgs84::EARTH_SEMI_MAJOR_AXIS, -1000., 0.), 1e-6));
            }
            #[test]
            fn test_pos_vec_sub() {
                let lhs = GeoPosition::from_lla((0., 0., 0.));
                let rhs = GeoVector::from_ecef(&dvec3(0., 1000., 0.), (0., 0., 0.).into_either());
                let actual = lhs - rhs;

                assert!(actual
                    .ecef
                    .abs_diff_eq(glam::dvec3(wgs84::EARTH_SEMI_MAJOR_AXIS, -1000., 0.), 1e-6));
            }
        }

        mod test_add {
            use super::*;

            #[test]
            fn test_pos_dvec3_add() {
                let lhs = GeoPosition::from_lla((0., 0., 0.));
                let actual = lhs.clone() + glam::dvec3(0., 1000., 0.);
                let actual_ref = &lhs + glam::dvec3(0., 1000., 0.);

                assert!(actual
                    .ecef
                    .abs_diff_eq(glam::dvec3(wgs84::EARTH_SEMI_MAJOR_AXIS, 1000., 0.), 1e-6));
                assert!(actual_ref
                    .ecef
                    .abs_diff_eq(glam::dvec3(wgs84::EARTH_SEMI_MAJOR_AXIS, 1000., 0.), 1e-6));
            }

            #[test]
            fn test_pos_vec_add() {
                let lhs = GeoPosition::from_lla((0., 0., 0.));
                let rhs = GeoVector::from_ecef(&dvec3(0., 1000., 0.), (0., 0., 0.).into_either());
                let actual = lhs.clone() + rhs.clone();
                let actual_ref = &lhs + rhs;

                assert!(actual
                    .ecef
                    .abs_diff_eq(glam::dvec3(wgs84::EARTH_SEMI_MAJOR_AXIS, 1000., 0.), 1e-6));
                assert!(actual_ref
                    .ecef
                    .abs_diff_eq(glam::dvec3(wgs84::EARTH_SEMI_MAJOR_AXIS, 1000., 0.), 1e-6));
            }
        }
    }

    #[cfg(feature = "serde")]
    mod test_serde {
        use super::*;

        #[test]
        fn test_deserialize() {
            let json = r#"
            {
                "ecef": [
                    119962.85915496295,
                    -5189589.602611365,
                    3693569.6778840856
                ]
            }"#;
            let actual = GeoPosition::model_validate_json(json).unwrap();
            almost::equal_with(actual.ecef.x, 119962.85915496295, 1e-10);
        }

        #[test]
        fn test_serialize() {
            let pos = GeoPosition::from_lla((0., 0., 0.));
            let actual = pos.model_dump_json();
            assert!(actual.is_ok());
        }
    }
}

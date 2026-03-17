use crate::{
    geo_objects::{
        geo_position::{EitherGeoPosOrLLATup, GeoPosition},
        geo_vector::GeoVector,
    },
    mach,
    traits::IntoEitherLLATupOrGeoPos,
    transforms::*,
    DVec3,
};
#[cfg(feature = "pydantic-serde")]
use crate::{utils, validator_wrapper_fn};
#[allow(unused_imports)]
#[cfg(feature = "bevy")]
use bevy::prelude::*;
#[cfg(feature = "pyo3")]
use either::Either;
#[cfg(not(feature = "pyo3"))]
use map3d_derive::*;
#[cfg(feature = "pyo3")]
use pyo3::{exceptions::PyValueError, prelude::*, types::*};
#[cfg(feature = "pyo3")]
use pyo3_stub_gen::derive::*;
#[cfg(feature = "py-bevy")]
use simple_py_bevy::*;
use std::ops::{Add, Div, Mul, Sub};

/// Represents a 3D velocity vector in geo space
/// Velocity is stored as a direction and speed so that a 0 velocity still has a direction associated with it
#[derive(Clone, Copy, Default, PartialEq)]
#[cfg_attr(feature = "pyo3", gen_stub_pyclass, pyclass)]
#[cfg_attr(
    all(feature = "py-bevy", feature = "pyo3"),
    derive(PyBevyCompRef, PyStructRef)
)]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
#[cfg_attr(feature = "bevy", derive(Component, Reflect), reflect(Component))]
#[cfg_attr(not(feature = "pyo3"), derive(DummyPyO3))]
pub struct GeoVelocity {
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
    dir_ecef: DVec3,
    #[pyo3(get, set)]
    speed: f64,
}

impl GeoVelocity {
    pub fn speed(&self) -> &f64 {
        &self.speed
    }
    pub fn speed_mut(&mut self) -> &mut f64 {
        &mut self.speed
    }
    pub fn set_speed(&mut self, speed: f64) {
        self.speed = speed;
    }
    pub fn direction_mut(&mut self) -> &mut glam::DVec3 {
        &mut self.dir_ecef
    }
    pub fn set_direction(&mut self, dir: glam::DVec3) {
        self.dir_ecef = dir.into()
    }

    /// Computes the mach number for this velocity at a given geo position
    ///
    /// # Arguments
    ///
    /// - `reference` (`&GeoPosition`) - Position to compute mach
    ///
    /// # Returns
    ///
    /// - `f64` - Mach number as an index
    ///
    pub fn mach(
        &self,
        reference: &GeoPosition,
    ) -> Result<f64, mach::OutOfBoundsAtmosphericLookupError> {
        mach::mach(self.speed, reference.alt())
    }
}

#[cfg(all(feature = "pydantic-serde", feature = "pyo3"))]
validator_wrapper_fn!(GeoVelocity, "GeoVelocity"); // need to create a non-classmethod validator for pydantic to hook into
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
    let float_schema = pydantic.float_schema_fn.call0()?;
    let desc = utils::pydantic::create_pydantic_schema_description(py, "Speed in Meters/second")?;
    let desc_kwargs = HashMap::from([desc]).into_py_dict(py)?;
    let float_field = pydantic
        .model_field_fn
        .call((float_schema,), Some(&desc_kwargs))?
        .unbind();

    let desc = utils::pydantic::create_pydantic_schema_description(
        py,
        "Unit Direction in ECEF frame. Normalized to 1.",
    )?;
    let desc_kwargs = HashMap::from([desc]).into_py_dict(py)?;
    let ecef_schema = pydantic
        .model_field_fn
        .call((dvec3_schema,), Some(&desc_kwargs))?
        .unbind();

    let model_field_schema_kwargs =
        HashMap::from([("dir_ecef", ecef_schema), ("speed", float_field)]);
    let fields = pydantic
        .model_fields_schema_fn
        .call1((model_field_schema_kwargs,))?;
    let geo_vel_class = GeoVelocity::type_object(py).unbind();

    let out = pydantic.model_schema_fn.call1((geo_vel_class, fields))?;
    Ok(out.unbind())
}

#[cfg_attr(
    all(feature = "py-bevy", feature = "pyo3"),
    py_bevy_methods,
    py_ref_methods
)]
#[cfg_attr(feature = "pyo3", gen_stub_pymethods, pymethods)]
impl GeoVelocity {
    /// Load from a json string
    /// ```
    /// {
    ///     "dir_ecef": [
    ///         1., 0., 0.
    ///     ],
    ///     "speed": 100.
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
    ///     "dir_ecef": [
    ///         1., 0., 0.
    ///     ],
    ///     "speed": 100.
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
    ///     "dir_ecef": [
    ///         1., 0., 0.
    ///     ],
    ///     "speed": 100.
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
    ///     "dir_ecef": [
    ///         1., 0., 0.
    ///     ],
    ///     "speed": 100.
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
    ///     ve;: rustmap3d.GeoVelocity
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

    /// Construct a velocity from an ecef unit direction and speed
    ///
    /// # Arguments
    ///
    /// - `ecef_dir` (`&DVec3`) - Unit vector in ecef frame
    /// - `speed_mps` (`f64`) - speed in meters per second
    ///
    #[staticmethod]
    pub fn from_dir_speed(ecef_dir: &DVec3, speed_mps: f64) -> Self {
        return GeoVelocity {
            dir_ecef: ecef_dir.normalize().into(),
            speed: speed_mps.into(),
        };
    }
    /// Construct a velocity from an ecef vector in meters/second
    ///
    /// # Arguments
    ///
    /// - `ecef` (`&DVec3`) - Velocity vector in ecef frame in meters/second
    ///
    #[staticmethod]
    pub fn from_ecef_uvw(ecef_uvw_mps: &DVec3) -> Self {
        return GeoVelocity {
            dir_ecef: ecef_uvw_mps.normalize().into(),
            speed: ecef_uvw_mps.length(),
        };
    }
    /// Construct a velocity from a local enu velocity vector in meters/second
    ///
    /// # Arguments
    ///
    /// - `enu_mps` (`&DVec3`) - Local enu velocity in meters/second
    /// - `reference` (`tuple[float, float, float] | GeoPosition`) - Reference location
    ///
    #[staticmethod]
    pub fn from_enu(enu_mps: &DVec3, reference: EitherGeoPosOrLLATup) -> GeoVelocity {
        let ecef = enu2ecef_uvw(enu_mps, reference);
        GeoVelocity {
            dir_ecef: ecef.normalize().into(),
            speed: ecef.length(),
        }
    }
    /// Construct a velocity from a local ned velocity vector in meters/second
    ///
    /// # Arguments
    ///
    /// - `ned_mps` (`&DVec3`) - Local ned velocity in meters/second
    /// - `reference` (`tuple[float, float, float] | GeoPosition`) - Reference location
    ///
    #[staticmethod]
    pub fn from_ned(ned_mps: &DVec3, reference: EitherGeoPosOrLLATup) -> GeoVelocity {
        let ecef = ned2ecef_uvw(ned_mps, reference);
        GeoVelocity {
            dir_ecef: ecef.normalize().into(),
            speed: ecef.length(),
        }
    }

    /// Get this velocity in ecef frame
    ///
    /// # Returns
    ///
    /// - `DVec3` - ECEF velocity in m/s
    ///
    #[getter]
    pub fn get_ecef_uvw(&self) -> DVec3 {
        return self.dir_ecef * self.speed;
    }
    /// Sets this velocity from an ecef uvw vector in m/s
    ///
    /// # Returns
    ///
    /// - `DVec3` - ECEF velocity in m/s
    ///
    #[setter]
    pub fn set_ecef_uvw(&mut self, ecef_uvw_mps: &DVec3) {
        self.dir_ecef = ecef_uvw_mps.normalize().into();
        self.speed = ecef_uvw_mps.length();
    }

    /// Get this velocity in a local enu frame in m/s
    ///
    /// # Arguments
    ///
    /// - `reference` (`GeoPosition`) - enu reference frame
    ///
    pub fn enu(&self, reference: &GeoPosition) -> DVec3 {
        ecef_uvw2enu(&self.get_ecef_uvw(), &reference.lla()).into()
    }
    /// Get this velocity in a local ned frame in m/s
    ///
    /// # Arguments
    ///
    /// - `reference` (`GeoPosition`) - ned reference frame
    ///
    pub fn ned(&self, reference: &GeoPosition) -> DVec3 {
        ecef_uvw2ned(&self.get_ecef_uvw(), &reference.lla()).into()
    }

    /// Computes the mach number for this velocity at a given geo position
    ///
    /// # Arguments
    ///
    /// - `reference` (`&GeoPosition`) - Position to compute mach
    ///
    /// # Returns
    ///
    /// - `f64` - Mach number as an index
    ///
    #[cfg(feature = "pyo3")]
    #[pyo3(name = "mach")] // we already have mach function, but need a python specific one
    fn py_mach(&self, reference: &GeoPosition) -> PyResult<f64> {
        match self.mach(reference) {
            Ok(m) => Ok(m),
            Err(e) => Err(PyValueError::new_err(format!("{:?}", e))),
        }
    }

    /// Multiply this GeoVelocity with either another GeoVelocity or time
    /// Multiplying by a float will produce a GeoVector equal to v * dt
    ///
    /// # Arguments
    ///
    /// - `rhs` (`Either<GeoVelocity, f64>`) - Velocity or time to multiply
    ///
    /// # Returns
    ///
    /// - `PyResult<Either<GeoVelocity, GeoVector>>` - Either a component-wise velocity multiply or a new GeoVector in meters
    ///
    #[cfg(feature = "pyo3")]
    fn __mul__(&self, rhs: Either<GeoVelocity, f64>) -> PyResult<Either<GeoVelocity, GeoVector>> {
        match rhs {
            Either::Left(vel) => Ok(Either::Left(self * vel)),
            Either::Right(time_s) => Ok(Either::Right(self * time_s)),
        }
    }
    #[cfg(feature = "pyo3")]
    fn __rmul__(&self, rhs: Either<GeoVelocity, f64>) -> PyResult<Either<GeoVelocity, GeoVector>> {
        self.__mul__(rhs)
    }
    /// Component-wise addition of velocity
    #[cfg(feature = "pyo3")]
    fn __add__(&self, rhs: GeoVelocity) -> PyResult<GeoVelocity> {
        Ok(self + rhs)
    }
    /// Component-wise subtraction of velocity
    #[cfg(feature = "pyo3")]
    fn __sub__(&self, rhs: GeoVelocity) -> PyResult<GeoVelocity> {
        Ok(self - rhs)
    }
    /// Component-wise division of velocity
    #[cfg(feature = "pyo3")]
    fn __div__(&self, rhs: GeoVelocity) -> PyResult<GeoVelocity> {
        Ok(self / rhs)
    }
    /// Component-wise addition of velocity
    #[cfg(feature = "pyo3")]
    fn __radd__(&self, lhs: GeoVelocity) -> PyResult<GeoVelocity> {
        Ok(self + lhs)
    }
    /// Component-wise subtraction of velocity
    #[cfg(feature = "pyo3")]
    fn __rsub__(&self, lhs: GeoVelocity) -> PyResult<GeoVelocity> {
        Ok(lhs - self)
    }
    /// Component-wise division of velocity
    #[cfg(feature = "pyo3")]
    fn __rdiv__(&self, lhs: GeoVelocity) -> PyResult<GeoVelocity> {
        Ok(lhs / self)
    }
}

macro_rules! geo_vel_mul_time {
    ($a:ty, $b:ty) => {
        impl Mul<$a> for $b {
            type Output = GeoVector;
            fn mul(self, time_s: $a) -> Self::Output {
                let delta_pos = self.get_ecef_uvw() * time_s;
                GeoVector::from_ecef(&delta_pos, (0., 0., 0.).into_either())
            }
        }
    };
}
geo_vel_mul_time!(f64, GeoVelocity);
geo_vel_mul_time!(&f64, GeoVelocity);
geo_vel_mul_time!(f64, &GeoVelocity);
geo_vel_mul_time!(&f64, &GeoVelocity);

macro_rules! ops_with_self {
    ($a:ty, $b:ty) => {
        impl Add<$a> for $b {
            type Output = GeoVelocity;
            fn add(self, rhs: $a) -> Self::Output {
                let new_vel = self.get_ecef_uvw() + rhs.get_ecef_uvw();
                GeoVelocity::from_ecef_uvw(&new_vel)
            }
        }
        impl Sub<$a> for $b {
            type Output = GeoVelocity;
            fn sub(self, rhs: $a) -> Self::Output {
                let new_vel = self.get_ecef_uvw() - rhs.get_ecef_uvw();
                GeoVelocity::from_ecef_uvw(&new_vel)
            }
        }
        impl Mul<$a> for $b {
            type Output = GeoVelocity;
            fn mul(self, rhs: $a) -> Self::Output {
                let new_vel = self.get_ecef_uvw() * rhs.get_ecef_uvw();
                GeoVelocity::from_ecef_uvw(&new_vel)
            }
        }
        impl Div<$a> for $b {
            type Output = GeoVelocity;
            fn div(self, rhs: $a) -> Self::Output {
                let new_vel = self.get_ecef_uvw() / rhs.get_ecef_uvw();
                GeoVelocity::from_ecef_uvw(&new_vel)
            }
        }
    };
}
ops_with_self!(GeoVelocity, GeoVelocity);
ops_with_self!(&GeoVelocity, GeoVelocity);
ops_with_self!(GeoVelocity, &GeoVelocity);
ops_with_self!(&GeoVelocity, &GeoVelocity);

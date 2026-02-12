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
use either::Either;
use pyo3::{exceptions::PyValueError, prelude::*};
use pyo3_stub_gen::derive::*;
#[cfg(feature = "py-bevy")]
use simple_py_bevy::*;
#[cfg(feature = "bevy")]
use bevy::prelude::*;
use std::ops::{Add, Div, Mul, Sub};

/// Represents a 3D velocity vector in geo space
/// Velocity is stored as a direction and speed so that a 0 velocity still has a direction associated with it
#[derive(Clone, Copy, Default, PartialEq)]
#[pyclass]
#[gen_stub_pyclass]
#[cfg_attr(feature = "py-bevy", derive(PyBevyCompRef, PyStructRef))]
#[cfg_attr(
    feature = "bevy",
    derive(
        Component,
        Reflect,
        serde::Deserialize,
        serde::Serialize,
    ),
    reflect(Component)
)]
pub struct GeoVelocity {
    #[cfg_attr(feature = "py-bevy", py_bevy(get_ref = pyglam::DVec3Ref))]
    #[pyo3(get, set)]
    dir_ecef: DVec3,
    #[pyo3(get, set)]
    speed: f64,
}

impl GeoVelocity {
    pub fn speed_mut(&mut self) -> &mut f64 {
        return &mut self.speed;
    }
    pub fn direction_mut(&mut self) -> &mut glam::DVec3 {
        return &mut self.dir_ecef;
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

#[cfg_attr(feature = "py-bevy", py_bevy_methods, py_ref_methods)]
#[pymethods]
#[gen_stub_pymethods]
impl GeoVelocity {
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
    fn __mul__(&self, rhs: Either<GeoVelocity, f64>) -> PyResult<Either<GeoVelocity, GeoVector>> {
        match rhs {
            Either::Left(vel) => Ok(Either::Left(self * vel)),
            Either::Right(time_s) => Ok(Either::Right(self * time_s)),
        }
    }
    fn __rmul__(&self, rhs: Either<GeoVelocity, f64>) -> PyResult<Either<GeoVelocity, GeoVector>> {
        self.__mul__(rhs)
    }
    /// Component-wise addition of velocity
    fn __add__(&self, rhs: GeoVelocity) -> PyResult<GeoVelocity> {
        Ok(self + rhs)
    }
    /// Component-wise subtraction of velocity
    fn __sub__(&self, rhs: GeoVelocity) -> PyResult<GeoVelocity> {
        Ok(self - rhs)
    }
    /// Component-wise division of velocity
    fn __div__(&self, rhs: GeoVelocity) -> PyResult<GeoVelocity> {
        Ok(self / rhs)
    }
    /// Component-wise addition of velocity
    fn __radd__(&self, lhs: GeoVelocity) -> PyResult<GeoVelocity> {
        Ok(self + lhs)
    }
    /// Component-wise subtraction of velocity
    fn __rsub__(&self, lhs: GeoVelocity) -> PyResult<GeoVelocity> {
        Ok(lhs - self)
    }
    /// Component-wise division of velocity
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

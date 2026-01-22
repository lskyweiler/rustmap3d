use crate::{
    geo_objects::{
        geo_position::{EitherGeoPosOrLLATup, GeoPosition},
        geo_vector::GeoVector,
    },
    traits::IntoEitherLLATupOrGeoPos,
    transforms::*,
};
use either::Either;
use pyglam;
use pyo3::prelude::*;
use pyo3_stub_gen::derive::*;
use std::ops::{Add, Div, Mul, Sub};

/// Represents a 3D velocity vector in Geo space
#[derive(Clone)]
#[gen_stub_pyclass]
#[pyclass]
pub struct GeoVelocity {
    dir_ecef: pyglam::DVec3,
    speed: f64,
}

impl GeoVelocity {
    pub fn speed_mut(&mut self) -> &mut f64 {
        return &mut self.speed;
    }
    pub fn direction_mut(&mut self) -> &mut glam::DVec3 {
        return &mut self.dir_ecef;
    }
}

#[gen_stub_pymethods]
#[pymethods]
impl GeoVelocity {
    /// Construct a velocity from an ecef unit direction and speed
    ///
    /// # Arguments
    ///
    /// - `ecef_dir` (`&DVec3`) - Unit vector in ecef frame
    /// - `speed_mps` (`f64`) - speed in meters per second
    ///
    #[staticmethod]
    pub fn from_dir_speed(ecef_dir: &pyglam::DVec3, speed_mps: f64) -> Self {
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
    pub fn from_ecef_uvw(ecef_uvw_mps: &pyglam::DVec3) -> Self {
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
    pub fn from_enu(enu_mps: &pyglam::DVec3, reference: EitherGeoPosOrLLATup) -> GeoVelocity {
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
    pub fn from_ned(ned_mps: &pyglam::DVec3, reference: EitherGeoPosOrLLATup) -> GeoVelocity {
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
    pub fn get_ecef_uvw(&self) -> pyglam::DVec3 {
        return self.dir_ecef * self.speed;
    }
    /// Gets the speed of this velocity in m/s
    #[getter]
    pub fn get_speed(&self) -> f64 {
        return self.speed;
    }
    #[setter]
    pub fn set_speed(&mut self, speed: f64) {
        self.speed = speed;
    }
    /// Get this velocity's direction in the ecef frame
    #[getter]
    pub fn get_direction(&self) -> pyglam::DVec3 {
        return self.dir_ecef;
    }
    #[setter]
    pub fn set_direction(&mut self, dir_ecef: &pyglam::DVec3) {
        self.dir_ecef = dir_ecef.to_owned();
    }

    /// Get this velocity in a local enu frame in m/s
    ///
    /// # Arguments
    ///
    /// - `reference` (`GeoPosition`) - enu reference frame
    ///
    pub fn enu(&self, reference: GeoPosition) -> pyglam::DVec3 {
        ecef_uvw2enu(&self.get_ecef_uvw(), &reference.lla()).into()
    }
    /// Get this velocity in a local ned frame in m/s
    ///
    /// # Arguments
    ///
    /// - `reference` (`GeoPosition`) - ned reference frame
    ///
    pub fn ned(&self, reference: GeoPosition) -> pyglam::DVec3 {
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
    pub fn mach(&self, reference: &GeoPosition) -> f64 {
        let _alt = reference.alt();
        return self.speed / 343.; // todo: lookup up speed of sound at alt
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

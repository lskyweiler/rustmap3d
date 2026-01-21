use crate::{geo_objects::geo_position::GeoPosition, transforms::*};
use pyglam;
use pyo3::prelude::*;
use pyo3_stub_gen::derive::*;

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
    /// - `ecef_dir` (`&pyglam`) - Unit vector in ecef frame
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
    /// - `ecef` (`&pyglam`) - Velocity vector in ecef frame in meters/second
    ///
    #[staticmethod]
    pub fn from_ecef(ecef_mps: &pyglam::DVec3) -> Self {
        return GeoVelocity {
            dir_ecef: ecef_mps.normalize().into(),
            speed: ecef_mps.length(),
        };
    }
    /// Construct a velocity from a local enu velocity vector in meters/second
    ///
    /// # Arguments
    ///
    /// - `enu_mps` (`&pyglam`) - Local enu velocity in meters/second
    /// - `reference` (`&GeoPosition`) - ENU reference location
    ///
    #[staticmethod]
    pub fn from_enu(enu_mps: &pyglam::DVec3, reference: &GeoPosition) -> GeoVelocity {
        let lla = reference.lla();
        let ecef = enu2ecef_uvw(enu_mps, &lla);
        GeoVelocity {
            dir_ecef: ecef.normalize().into(),
            speed: ecef.length(),
        }
    }
    /// Construct a velocity from a local ned velocity vector in meters/second
    ///
    /// # Arguments
    ///
    /// - `ned_mps` (`&pyglam`) - Local ned velocity in meters/second
    /// - `reference` (`&GeoPosition`) - NED reference location
    ///
    #[staticmethod]
    pub fn from_ned(ned_mps: &pyglam::DVec3, reference: &GeoPosition) -> GeoVelocity {
        let lla = reference.lla();
        let ecef = ned2ecef_uvw(ned_mps, &lla);
        GeoVelocity {
            dir_ecef: ecef.normalize().into(),
            speed: ecef.length(),
        }
    }

    /// Get this velocity in ecef frame
    ///
    /// # Returns
    ///
    /// - `pyglam::DVec3` - ECEF velocity in m/s
    ///
    #[getter]
    pub fn get_ecef_vel(&self) -> pyglam::DVec3 {
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
        ecef_uvw2enu(&self.get_ecef_vel(), &reference.lla()).into()
    }
    /// Get this velocity in a local ned frame in m/s
    ///
    /// # Arguments
    ///
    /// - `reference` (`GeoPosition`) - ned reference frame
    ///
    pub fn ned(&self, reference: GeoPosition) -> pyglam::DVec3 {
        ecef_uvw2ned(&self.get_ecef_vel(), &reference.lla()).into()
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
}
// impl Mul<f64> for GeoVelocity {
//     type Output = GeoVector;
//     fn mul(self, time_s: f64) -> Self::Output {
//         let delta_pos = self.ecef() * time_s;
//         GeoVector::from_ecef(delta_pos)
//     }
// }
// impl Mul<f64> for &GeoVelocity {
//     type Output = GeoVector;
//     fn mul(self, time_s: f64) -> Self::Output {
//         let delta_pos = self.ecef() * time_s;
//         GeoVector::from_ecef(delta_pos)
//     }
// }

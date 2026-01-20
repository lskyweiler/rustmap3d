use crate::{
    geo_objects::{geo_position::GeoPosition, geo_vector::GeoVector},
    traits::*,
    transforms::*,
};
use glam;
use std::{
    fmt::Debug,
    ops::{Add, Mul, Sub},
};

#[derive(Clone)]
pub struct GeoVelocity {
    dir_ecef: glam::DVec3,
    speed: f64,
}
impl GeoVelocity {
    pub fn from_dir_speed(ecef_dir: impl IntoDVec3, speed_mps: impl Into<f64>) -> Self {
        return GeoVelocity {
            dir_ecef: ecef_dir.into_dvec3(),
            speed: speed_mps.into(),
        };
    }
    pub fn from_ecef(ecef: impl IntoDVec3) -> Self {
        let ecef = ecef.into_dvec3();
        return GeoVelocity {
            dir_ecef: ecef.normalize(),
            speed: ecef.length(),
        };
    }
    pub fn from_enu(enu: impl IntoDVec3, reference: GeoPosition) -> GeoVelocity {
        let lla = reference.lla();
        let ecef = enu2ecef_uvw(enu, &lla);
        GeoVelocity {
            dir_ecef: ecef.normalize(),
            speed: ecef.length(),
        }
    }
    pub fn from_ned(ned: impl IntoDVec3, reference: GeoPosition) -> GeoVelocity {
        let lla = reference.lla();
        let ecef = ned2ecef_uvw(ned, &lla);
        GeoVelocity {
            dir_ecef: ecef.normalize(),
            speed: ecef.length(),
        }
    }

    pub fn ecef(&self) -> glam::DVec3 {
        return self.dir_ecef * self.speed;
    }
    pub fn speed(&self) -> f64 {
        return self.speed;
    }
    pub fn direction(&self) -> glam::DVec3 {
        return self.dir_ecef;
    }
    pub fn speed_mut(&mut self) -> &mut f64 {
        return &mut self.speed;
    }
    pub fn direction_mut(&mut self) -> &mut glam::DVec3 {
        return &mut self.dir_ecef;
    }
    pub fn enu(&self, reference: GeoPosition) -> glam::DVec3 {
        return ecef_uvw2enu(&self.ecef(), &reference.lla());
    }
    pub fn ned(&self, reference: GeoPosition) -> glam::DVec3 {
        return ecef_uvw2ned(&self.ecef(), &reference.lla());
    }

    pub fn mach(&self, reference: &GeoPosition) -> f64 {
        let _alt = reference.alt();
        return self.speed / 343.; // todo: lookup up speed of sound at alt
    }

    pub fn set_speed(&mut self, speed: f64) {
        self.speed = speed;
    }
    pub fn set_direction(&mut self, dir_ecef: &glam::DVec3) {
        self.dir_ecef = dir_ecef.to_owned();
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

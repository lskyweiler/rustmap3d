use crate::{geo_objects::geo_vector::GeoVector, traits::*, transforms::*, utils, vincenty::*};
use either::Either;
use glam::{self, swizzles::*};
use pyglam;
use pyo3::prelude::*;
use pyo3_stub_gen::derive::*;
use std::{
    fmt::Debug,
    ops::{Add, Sub},
};

/// Represents a position on the earth
#[derive(Clone)]
#[gen_stub_pyclass]
#[pyclass]
pub struct GeoPosition {
    /// Store the position in an [ecef](https://en.wikipedia.org/wiki/Earth-centered,_Earth-fixed_coordinate_system) vector since this is the most exact representation
    ecef: pyglam::DVec3,
}

impl GeoPosition {
    pub fn ecef(&self) -> &pyglam::DVec3 {
        &self.ecef
    }
    pub fn ecef_mut(&mut self) -> &mut pyglam::DVec3 {
        return &mut self.ecef;
    }
    pub fn lla(&self) -> (f64, f64, f64) {
        ecef2lla(&self.ecef).into()
    }
}

#[gen_stub_pymethods]
#[pymethods]
impl GeoPosition {
    #[staticmethod]
    pub fn from_ecef(ecef: &pyglam::DVec3) -> Self {
        Self { ecef: ecef.clone() }
    }
    #[staticmethod]
    pub fn from_lla(lla: (f64, f64, f64)) -> Self {
        Self {
            ecef: lla2ecef(lla).into(),
        }
    }
    #[staticmethod]
    pub fn from_enu(enu: &pyglam::DVec3, lla_ref: (f64, f64, f64)) -> Self {
        Self {
            ecef: enu2ecef(enu.into_dvec3(), lla_ref).into(),
        }
    }
    #[staticmethod]
    pub fn from_ned(ned: &pyglam::DVec3, lla_ref: (f64, f64, f64)) -> Self {
        Self {
            ecef: ned2ecef(ned.into_dvec3(), lla_ref).into(),
        }
    }

    #[getter]
    fn get_ecef(&self) -> pyglam::DVec3 {
        self.ecef
    }
    #[setter]
    fn set_ecef(&mut self, ecef: pyglam::DVec3) {
        self.ecef = ecef
    }
    #[getter]
    fn get_lla(&self) -> (f64, f64, f64) {
        self.lla()
    }
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

    pub fn bearing_to(&self, to_point: &GeoPosition) -> f64 {
        let lla_a = self.lla();
        let lla_b = to_point.lla();

        let (_, bearing, _) =
            vincenty_inverse(lla_a.0, lla_a.1, lla_b.0, lla_b.1, 1e-10, 200).unwrap();

        return utils::wrap_to_0_360(bearing);
    }
    pub fn enu_to(&self, to_point: &GeoPosition) -> pyglam::DVec3 {
        ecef2enu(&to_point.ecef, &self.lla()).into()
    }
    pub fn aer_to(&self, to_point: &GeoPosition) -> pyglam::DVec3 {
        ecef2aer(&to_point.ecef, &self.lla()).into()
    }
    pub fn ned_to(&self, to_point: &GeoPosition) -> pyglam::DVec3 {
        ecef2ned(&to_point.ecef, &self.lla()).into()
    }

    pub fn bearing_from(&self, from_point: &GeoPosition) -> f64 {
        let lla_a = self.lla();
        let lla_b = from_point.lla();

        let (_, _, bearing) =
            vincenty_inverse(lla_a.0, lla_a.1, lla_b.0, lla_b.1, 1e-10, 200).unwrap();
        return utils::wrap_to_0_360(bearing);
    }
    pub fn enu_from(&self, from_point: &GeoPosition) -> pyglam::DVec3 {
        ecef2enu(&self.ecef, &from_point.lla()).into()
    }
    pub fn aer_from(&self, from_point: &GeoPosition) -> pyglam::DVec3 {
        ecef2aer(&self.ecef, &from_point.lla()).into()
    }
    pub fn ned_from(&self, from_point: &GeoPosition) -> pyglam::DVec3 {
        ecef2ned(&self.ecef, &from_point.lla()).into()
    }

    pub fn distance(&self, other: &GeoPosition) -> f64 {
        (self.ecef - other.ecef).length()
    }

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

    fn __add__(&self, rhs: Either<GeoVector, pyglam::DVec3>) -> PyResult<GeoPosition> {
        match rhs {
            Either::Left(vec) => Ok(self + vec),
            Either::Right(vec) => Ok(GeoPosition::from_ecef(&(self.ecef + vec))),
        }
    }
    fn __radd__(&self, rhs: Either<GeoVector, pyglam::DVec3>) -> PyResult<GeoPosition> {
        self.__add__(rhs)
    }
    fn __sub__(
        &self,
        rhs: Either<GeoVector, GeoPosition>,
    ) -> PyResult<Either<GeoVector, GeoPosition>> {
        match rhs {
            Either::Left(vec) => Ok(Either::Right(self - vec)),
            Either::Right(vec) => Ok(Either::Left(self - vec)),
        }
    }
    fn __rsub__(&self, lhs: GeoPosition) -> PyResult<GeoVector> {
        Ok(lhs - self)
    }
}

impl Debug for GeoPosition {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.lat_lon_dms())
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

/// Subtracting two GeoPositions results in a GeoVector starting at RHS -> LHS
macro_rules! geo_pos_subs_with_geopos {
    ($a:ty, $b:ty) => {
        impl Sub<$a> for $b {
            type Output = GeoVector;

            fn sub(self, rhs: $a) -> Self::Output {
                GeoVector::from_ecef(&(self.ecef - rhs.ecef), rhs.lla())
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
    use crate::wgs84;

    mod test_constructors {
        use crate::wgs84;

        use super::*;

        #[test]
        fn test_from_ecef() {
            let actual =
                GeoPosition::from_ecef(&pyglam::dvec3(wgs84::EARTH_SEMI_MAJOR_AXIS, 0., 0.));
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
            let actual = GeoPosition::from_enu(&pyglam::dvec3(100., 0., 0.), (0., 0., 0.));
            assert!(actual
                .ecef()
                .abs_diff_eq(glam::dvec3(wgs84::EARTH_SEMI_MAJOR_AXIS, 100., 0.), 1e-10));
        }
        #[test]
        fn test_from_ned() {
            let actual = GeoPosition::from_ned(&pyglam::dvec3(0., 100., 0.), (0., 0., 0.));
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
            let b = GeoPosition::from_ecef(&pyglam::dvec3(wgs84::EARTH_SEMI_MAJOR_AXIS, 100., 0.));

            assert!(a.enu_to(&b).abs_diff_eq(glam::dvec3(100., 0., 0.), 1e-6));
            assert!(a.ned_to(&b).abs_diff_eq(glam::dvec3(0., 100., 0.), 1e-6));
            assert!(a.aer_to(&b).abs_diff_eq(glam::dvec3(90., 0., 100.), 1e-6));
            almost::equal_with(a.bearing_to(&b), 90., 1e-7);
        }
        #[test]
        fn test_from() {
            let a = GeoPosition::from_lla((0., 0., 0.));
            let b = GeoPosition::from_ecef(&pyglam::dvec3(wgs84::EARTH_SEMI_MAJOR_AXIS, 100., 0.));

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
            let actual =
                GeoPosition::from_ecef(&pyglam::dvec3(wgs84::EARTH_SEMI_MAJOR_AXIS, 0., 0.));
            almost::equal_with(actual.lla().0, 0., 1e-5);
            almost::equal_with(actual.lla().1, 0., 1e-5);
            almost::equal_with(actual.lla().2, 0., 1e-5);
        }
        #[test]
        fn test_set_alt() {
            let mut actual =
                GeoPosition::from_ecef(&pyglam::dvec3(0., wgs84::EARTH_SEMI_MAJOR_AXIS, 0.)); // vector straight up at 90lon
            actual.set_alt(1000.);

            assert!(actual.ecef().abs_diff_eq(
                glam::dvec3(0., wgs84::EARTH_SEMI_MAJOR_AXIS + 1000., 0.),
                1e-5
            ));
        }

        #[test]
        fn test_rot_alt() {
            let mut actual = GeoPosition::from_lla((0., 0., 100.));
            let rot = pyglam::DQuat::from_axis_angle(&pyglam::dvec3(0., 0., 1.), f64::consts::PI);
            actual.rotate_lat_lon(&rot);

            almost::equal_with(actual.lla().0, 0., 1e-5);
            almost::equal_with(actual.lla().1, 90., 1e-5);
            almost::equal_with(actual.lla().2, 100., 1e-5);
        }
    }

    #[test]
    fn test_distance() {
        let a = GeoPosition::from_ecef(&pyglam::dvec3(wgs84::EARTH_SEMI_MAJOR_AXIS, 0., 0.));
        let b = GeoPosition::from_ecef(&pyglam::dvec3(wgs84::EARTH_SEMI_MAJOR_AXIS, 100., 0.));

        almost::equal_with(a.distance(&b), 100., 1e-10);
    }

    mod test_ops {
        use super::*;

        mod test_sub {
            use super::*;
            #[test]
            fn test_pos_pos_sub() {
                let rhs = GeoPosition::from_lla((0., 0., 0.));
                let lhs =
                    GeoPosition::from_ecef(&pyglam::dvec3(wgs84::EARTH_SEMI_MAJOR_AXIS, 1000., 0.));
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
                let rhs = GeoVector::from_ecef(&pyglam::dvec3(0., 1000., 0.), (0., 0., 0.));
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
                let rhs = GeoVector::from_ecef(&pyglam::dvec3(0., 1000., 0.), (0., 0., 0.));
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
}

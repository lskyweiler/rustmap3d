use crate::{geo_objects::geo_vector::GeoVector, traits::*, transforms::*, utils, vincenty::*};
use glam::{self, swizzles::*};
use std::{
    fmt::Debug,
    ops::{Add, Sub},
};

/// Represents a position on the earth
#[derive(Clone, Copy)]
pub struct GeoPosition {
    /// Store the position in an [ecef](https://en.wikipedia.org/wiki/Earth-centered,_Earth-fixed_coordinate_system) vector since this is the most exact representation
    ecef: glam::DVec3,
}

impl GeoPosition {
    pub fn from_ecef(ecef: impl IntoDVec3) -> Self {
        Self {
            ecef: ecef.into_dvec3(),
        }
    }
    pub fn from_lla(lla: impl IntoLatLonTriple) -> Self {
        Self {
            ecef: lla2ecef(lla),
        }
    }
    pub fn from_enu(enu: impl IntoDVec3, lla_ref: impl IntoLatLonTriple) -> Self {
        Self {
            ecef: enu2ecef(enu, lla_ref),
        }
    }
    pub fn from_ned(ned: impl IntoDVec3, lla_ref: impl IntoLatLonTriple) -> Self {
        Self {
            ecef: ned2ecef(ned, lla_ref),
        }
    }

    pub fn ecef(&self) -> &glam::DVec3 {
        &self.ecef
    }
    pub fn ecef_mut(&mut self) -> &mut glam::DVec3 {
        return &mut self.ecef;
    }

    pub fn lla(&self) -> (f64, f64, f64) {
        ecef2lla(&self.ecef).into()
    }
    pub fn alt(&self) -> f64 {
        return self.lla().2;
    }
    ///  Sets the altitude of this position while preserving the lat/lon
    ///
    /// # Arguments
    ///
    /// - `alt_m` (`f64`) - New MSL altitude in meters
    pub fn set_alt(&mut self, alt_m: f64) {
        let lla = self.lla();
        self.ecef = lla2ecef((lla.0, lla.1, alt_m));
    }

    pub fn bearing_to(&self, to_point: &GeoPosition) -> f64 {
        let lla_a = self.lla();
        let lla_b = to_point.lla();

        let (_, bearing, _) =
            vincenty_inverse(lla_a.0, lla_a.1, lla_b.0, lla_b.1, 1e-10, 200).unwrap();

        return utils::wrap_to_0_360(bearing);
    }
    pub fn enu_to(&self, to_point: &GeoPosition) -> glam::DVec3 {
        return ecef2enu(&to_point.ecef, &self.lla());
    }
    pub fn aer_to(&self, to_point: &GeoPosition) -> glam::DVec3 {
        return ecef2aer(&to_point.ecef, &self.lla());
    }
    pub fn ned_to(&self, to_point: &GeoPosition) -> glam::DVec3 {
        return ecef2ned(&to_point.ecef, &self.lla());
    }

    pub fn bearing_from(&self, from_point: &GeoPosition) -> f64 {
        let lla_a = self.lla();
        let lla_b = from_point.lla();

        let (_, _, bearing) =
            vincenty_inverse(lla_a.0, lla_a.1, lla_b.0, lla_b.1, 1e-10, 200).unwrap();
        return utils::wrap_to_0_360(bearing);
    }
    pub fn enu_from(&self, from_point: &GeoPosition) -> glam::DVec3 {
        return ecef2enu(&self.ecef, &from_point.lla());
    }
    pub fn aer_from(&self, from_point: &GeoPosition) -> glam::DVec3 {
        return ecef2aer(&self.ecef, &from_point.lla());
    }
    pub fn ned_from(&self, from_point: &GeoPosition) -> glam::DVec3 {
        return ecef2ned(&self.ecef, &from_point.lla());
    }

    pub fn distance(&self, other: &GeoPosition) -> f64 {
        return self.ecef.distance(*other.ecef());
    }

    pub fn lat_lon_dms(&self) -> String {
        let ll = self.lla();
        return format!("{}, {}", dd2dms(ll.0, true), dd2dms(ll.1, false));
    }

    /// Rotate the geo position by an ecef rotation, but preserve the starting altitude
    pub fn rotate_lat_lon(&mut self, ecef_rot: glam::DQuat) {
        let starting_alt = self.alt();
        let new_ecef = ecef_rot * self.ecef;
        let new_lat_lon = ecef2lla(&new_ecef).xy();
        self.ecef = lla2ecef(&glam::dvec3(new_lat_lon.x, new_lat_lon.y, starting_alt));
    }
}

impl Debug for GeoPosition {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.lat_lon_dms())
    }
}
impl PartialEq for GeoPosition {
    fn eq(&self, other: &Self) -> bool {
        self.ecef == other.ecef
    }
    fn ne(&self, other: &Self) -> bool {
        self.ecef != other.ecef
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
impl Sub for GeoPosition {
    type Output = GeoVector;

    fn sub(self, rhs: Self) -> Self::Output {
        GeoVector::from_ecef(self.ecef - rhs.ecef, rhs.lla())
    }
}
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
            let actual = GeoPosition::from_ecef(glam::dvec3(wgs84::EARTH_SEMI_MAJOR_AXIS, 0., 0.));
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
            let actual = GeoPosition::from_enu(glam::dvec3(100., 0., 0.), (0., 0., 0.));
            assert!(actual
                .ecef()
                .abs_diff_eq(glam::dvec3(wgs84::EARTH_SEMI_MAJOR_AXIS, 100., 0.), 1e-10));
        }
        #[test]
        fn test_from_ned() {
            let actual = GeoPosition::from_ned(glam::dvec3(0., 100., 0.), (0., 0., 0.));
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
            let b = GeoPosition::from_ecef(glam::dvec3(wgs84::EARTH_SEMI_MAJOR_AXIS, 100., 0.));

            assert!(a.enu_to(&b).abs_diff_eq(glam::dvec3(100., 0., 0.), 1e-6));
            assert!(a.ned_to(&b).abs_diff_eq(glam::dvec3(0., 100., 0.), 1e-6));
            assert!(a.aer_to(&b).abs_diff_eq(glam::dvec3(90., 0., 100.), 1e-6));
            almost::equal_with(a.bearing_to(&b), 90., 1e-7);
        }
        #[test]
        fn test_from() {
            let a = GeoPosition::from_lla((0., 0., 0.));
            let b = GeoPosition::from_ecef(glam::dvec3(wgs84::EARTH_SEMI_MAJOR_AXIS, 100., 0.));

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
            let actual = GeoPosition::from_ecef(glam::dvec3(wgs84::EARTH_SEMI_MAJOR_AXIS, 0., 0.));
            almost::equal_with(actual.lla().0, 0., 1e-5);
            almost::equal_with(actual.lla().1, 0., 1e-5);
            almost::equal_with(actual.lla().2, 0., 1e-5);
        }
        #[test]
        fn test_set_alt() {
            let mut actual =
                GeoPosition::from_ecef(glam::dvec3(0., wgs84::EARTH_SEMI_MAJOR_AXIS, 0.)); // vector straight up at 90lon
            actual.set_alt(1000.);

            assert!(actual.ecef().abs_diff_eq(
                glam::dvec3(0., wgs84::EARTH_SEMI_MAJOR_AXIS + 1000., 0.),
                1e-5
            ));
        }

        #[test]
        fn test_rot_alt() {
            let mut actual = GeoPosition::from_lla(glam::dvec3(0., 0., 100.));
            let rot = glam::DQuat::from_axis_angle(glam::DVec3::Z, f64::consts::PI);
            actual.rotate_lat_lon(rot);

            almost::equal_with(actual.lla().0, 0., 1e-5);
            almost::equal_with(actual.lla().1, 90., 1e-5);
            almost::equal_with(actual.lla().2, 100., 1e-5);
        }
    }

    #[test]
    fn test_distance() {
        let a = GeoPosition::from_ecef(glam::dvec3(wgs84::EARTH_SEMI_MAJOR_AXIS, 0., 0.));
        let b = GeoPosition::from_ecef(glam::dvec3(wgs84::EARTH_SEMI_MAJOR_AXIS, 100., 0.));

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
                    GeoPosition::from_ecef(glam::dvec3(wgs84::EARTH_SEMI_MAJOR_AXIS, 1000., 0.));
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
                let rhs = GeoVector::from_ecef(glam::dvec3(0., 1000., 0.), (0., 0., 0.));
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
                let rhs = GeoVector::from_ecef(glam::dvec3(0., 1000., 0.), (0., 0., 0.));
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

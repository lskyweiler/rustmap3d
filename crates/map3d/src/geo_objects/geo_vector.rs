use crate::{traits::*, transforms::*};
use glam;

/// Represents a vector relative to a reference point
#[derive(Clone, Copy)]
pub struct GeoVector {
    ecef_uvw: glam::DVec3,
    lla_ref: (f64, f64, f64),
}

impl GeoVector {
    pub fn from_ecef(ecef_uvw: impl IntoDVec3, lla_ref: impl IntoLatLonTriple) -> Self {
        Self {
            ecef_uvw: ecef_uvw.into_dvec3(),
            lla_ref: lla_ref.into_lat_lon_triple(),
        }
    }
    pub fn from_enu(enu: impl IntoDVec3, lla_ref: impl IntoLatLonTriple) -> Self {
        let lla_ref = lla_ref.into_lat_lon_triple();
        Self {
            ecef_uvw: enu2ecef_uvw(enu, &lla_ref),
            lla_ref: lla_ref,
        }
    }
    pub fn from_ned(ned: impl IntoDVec3, lla_ref: impl IntoLatLonTriple) -> Self {
        let lla_ref = lla_ref.into_lat_lon_triple();
        Self {
            ecef_uvw: ned2ecef_uvw(ned, &lla_ref),
            lla_ref: lla_ref,
        }
    }
    pub fn from_aer(aer: impl IntoDVec3, lla_ref: impl IntoLatLonTriple) -> Self {
        let lla_ref = lla_ref.into_lat_lon_triple();
        Self {
            ecef_uvw: aer2ecef_uvw(aer, &lla_ref),
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

    pub fn ecef(&self) -> glam::DVec3 {
        self.ecef_uvw + lla2ecef(self.lla_ref)
    }
    pub fn ecef_uvw(&self) -> &glam::DVec3 {
        &self.ecef_uvw
    }
    pub fn enu(&self) -> glam::DVec3 {
        ecef_uvw2enu(&self.ecef_uvw, &self.lla_ref)
    }
    pub fn ned(&self) -> glam::DVec3 {
        ecef_uvw2ned(&self.ecef_uvw, &self.lla_ref)
    }
    pub fn aer(&self) -> glam::DVec3 {
        ecef_uvw2aer(&self.ecef_uvw, &self.lla_ref)
    }

    pub fn north(&self) -> f64 {
        self.enu().y
    }
    pub fn south(&self) -> f64 {
        -self.enu().y
    }
    pub fn east(&self) -> f64 {
        self.enu().x
    }
    pub fn west(&self) -> f64 {
        -self.enu().x
    }
    pub fn up(&self) -> f64 {
        self.enu().z
    }
    pub fn down(&self) -> f64 {
        self.ned().z
    }

    /// Compute the clockwise angle off north for this vector relative to its reference
    /// Angle is always between [0., 360.]
    ///
    /// # Returns
    ///
    /// * `azimuth` - Clockwise angle off true north in [[degrees]]
    pub fn azimuth(&self) -> f64 {
        self.aer().x
    }
}

#[cfg(test)]
mod test_geo_vector {
    use super::*;

    mod test_constructors {
        use super::*;

        #[test]
        fn test_from_ecef() {
            let actual = GeoVector::from_ecef(glam::dvec3(100., 0., 0.), (0., 0., 0.));
            assert!(actual
                .ecef_uvw()
                .abs_diff_eq(glam::dvec3(100., 0., 0.), 1e-10));
        }
        #[test]
        fn test_from_enu() {
            let actual = GeoVector::from_enu(glam::dvec3(100., 0., 0.), (0., 0., 0.));
            assert!(actual.enu().abs_diff_eq(glam::dvec3(100., 0., 0.), 1e-10));
        }
        #[test]
        fn test_from_ned() {
            let actual = GeoVector::from_ned(glam::dvec3(100., 0., 0.), (0., 0., 0.));
            assert!(actual.ned().abs_diff_eq(glam::dvec3(100., 0., 0.), 1e-10));
        }
        #[test]
        fn test_from_aer() {
            let actual = GeoVector::from_aer(glam::dvec3(100., 0., 100.), (0., 0., 0.));
            assert!(actual.aer().abs_diff_eq(glam::dvec3(100., 0., 100.), 1e-10));
        }
    }

    mod test_utils {
        use super::*;

        #[test]
        fn test_cardinals() {
            let actual = GeoVector::from_ecef(glam::dvec3(100., 50., -100.), (0., 0., 0.));

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
            let actual = GeoVector::from_ecef(glam::dvec3(0., 100., 0.), (0., 0., 0.));
            almost::equal_with(actual.azimuth(), 90., 1e-10);
        }

        #[test]
        fn test_length() {
            let actual = GeoVector::from_ecef(glam::dvec3(-100., 0., 0.), (0., 0., 0.));
            almost::equal_with(actual.length(), 100., 1e-10);
        }
    }
}

use crate::geo_objects::geo_position::GeoPosition;
use either::Either;
use glam;
use pyglam;

pub trait IntoDVec3 {
    fn into_dvec3(&self) -> glam::DVec3;
}
pub trait IntoLatLonTuple {
    fn into_lat_lon_tuple(&self) -> (f64, f64);
}
pub trait IntoLatLonTriple {
    fn into_lat_lon_triple(&self) -> (f64, f64, f64);
}

impl IntoDVec3 for &glam::DVec3 {
    fn into_dvec3(&self) -> glam::DVec3 {
        (*self).clone()
    }
}
impl IntoDVec3 for glam::DVec3 {
    fn into_dvec3(&self) -> glam::DVec3 {
        self.clone()
    }
}
impl IntoDVec3 for &pyglam::DVec3 {
    fn into_dvec3(&self) -> glam::DVec3 {
        (**self).into()
    }
}
impl IntoDVec3 for pyglam::DVec3 {
    fn into_dvec3(&self) -> glam::DVec3 {
        (**self).into()
    }
}
impl IntoDVec3 for (f64, f64, f64) {
    fn into_dvec3(&self) -> glam::DVec3 {
        glam::dvec3(self.0, self.1, self.2)
    }
}

impl IntoLatLonTuple for &glam::DVec3 {
    fn into_lat_lon_tuple(&self) -> (f64, f64) {
        (self.x, self.y)
    }
}
impl IntoLatLonTuple for &glam::DVec2 {
    fn into_lat_lon_tuple(&self) -> (f64, f64) {
        (self.x, self.y)
    }
}
impl IntoLatLonTuple for glam::DVec3 {
    fn into_lat_lon_tuple(&self) -> (f64, f64) {
        (self.x, self.y)
    }
}
impl IntoLatLonTuple for glam::DVec2 {
    fn into_lat_lon_tuple(&self) -> (f64, f64) {
        (self.x, self.y)
    }
}
impl IntoLatLonTuple for (f64, f64) {
    fn into_lat_lon_tuple(&self) -> (f64, f64) {
        (self.0, self.1)
    }
}
impl IntoLatLonTuple for (f64, f64, f64) {
    fn into_lat_lon_tuple(&self) -> (f64, f64) {
        (self.0, self.1)
    }
}
impl IntoLatLonTuple for &(f64, f64) {
    fn into_lat_lon_tuple(&self) -> (f64, f64) {
        (self.0, self.1)
    }
}
impl IntoLatLonTuple for &(f64, f64, f64) {
    fn into_lat_lon_tuple(&self) -> (f64, f64) {
        (self.0, self.1)
    }
}

impl IntoLatLonTriple for &glam::DVec3 {
    fn into_lat_lon_triple(&self) -> (f64, f64, f64) {
        (self.x, self.y, self.z)
    }
}
impl IntoLatLonTriple for glam::DVec3 {
    fn into_lat_lon_triple(&self) -> (f64, f64, f64) {
        (self.x, self.y, self.z)
    }
}
impl IntoLatLonTriple for (f64, f64, f64) {
    fn into_lat_lon_triple(&self) -> (f64, f64, f64) {
        (self.0, self.1, self.2)
    }
}
impl IntoLatLonTriple for &(f64, f64, f64) {
    fn into_lat_lon_triple(&self) -> (f64, f64, f64) {
        (self.0, self.1, self.2)
    }
}

// I dont like this solution, but we need a way to enable passing in either lla | GeoPosition | GeoPositionBevyRef
//  This should be better handled in the macros


#[cfg(not(feature = "py-bevy"))]
pub type EitherGeoPosOrLLATup = Either<(f64, f64, f64), GeoPosition>;

#[cfg(feature = "py-bevy")]
mod either_geo_or_lla_bevy {
    use crate::geo_objects::geo_position::{GeoPosition, GeoPositionBevyRef};
    use either::Either;
    use pyo3::pyclass;
    use pyo3::{exceptions::PyValueError, FromPyObject};
    use pyo3_stub_gen::derive::*;

    #[pyclass]
    #[gen_stub_pyclass]
    pub struct EitherGeoPosOrLLATup(pub(crate) Either<(f64, f64, f64), GeoPosition>);
    impl<'a, 'py> FromPyObject<'a, 'py> for EitherGeoPosOrLLATup {
        type Error = pyo3::PyErr;
        fn extract(
            obj: pyo3::prelude::Borrowed<'a, 'py, pyo3::prelude::PyAny>,
        ) -> Result<Self, Self::Error> {
            if let Ok(tup) = obj.extract::<(f64, f64, f64)>() {
                return Ok(EitherGeoPosOrLLATup(Either::Left(tup)));
            } else if let Ok(pos) = obj.extract::<GeoPosition>() {
                return Ok(EitherGeoPosOrLLATup(Either::Right(pos)));
            } else if let Ok(pos_ref) = obj.cast::<GeoPositionBevyRef>() {
                let owned = pos_ref.borrow().get_inner_ref_mut()?.clone();
                return Ok(EitherGeoPosOrLLATup(Either::Right(owned)));
            }
            Err(PyValueError::new_err(format!(
                "Unknown type for GeoPosOrLLATuple"
            )))
        }
    }
}
#[cfg(feature = "py-bevy")]
pub use either_geo_or_lla_bevy::*;

pub trait IntoEitherLLATupOrGeoPos {
    fn into_either(self) -> EitherGeoPosOrLLATup;
    fn into_lla_tuple(self) -> (f64, f64, f64);
    fn into_geo_pos(self) -> GeoPosition;
}

impl IntoEitherLLATupOrGeoPos for (f64, f64, f64) {
    fn into_either(self) -> EitherGeoPosOrLLATup {
        #[cfg(feature = "py-bevy")]
        return EitherGeoPosOrLLATup(Either::Left(self));
        #[cfg(not(feature = "py-bevy"))]
        return Either::Left(self);
    }
    fn into_lla_tuple(self) -> (f64, f64, f64) {
        self
    }
    fn into_geo_pos(self) -> GeoPosition {
        GeoPosition::from_lla(self)
    }
}
impl IntoEitherLLATupOrGeoPos for &(f64, f64, f64) {
    fn into_either(self) -> EitherGeoPosOrLLATup {
        #[cfg(feature = "py-bevy")]
        return EitherGeoPosOrLLATup(Either::Left(*self));
        #[cfg(not(feature = "py-bevy"))]
        return Either::Left(*self);
    }
    fn into_lla_tuple(self) -> (f64, f64, f64) {
        *self
    }
    fn into_geo_pos(self) -> GeoPosition {
        GeoPosition::from_lla(*self)
    }
}
impl IntoEitherLLATupOrGeoPos for GeoPosition {
    fn into_either(self) -> EitherGeoPosOrLLATup {
        #[cfg(feature = "py-bevy")]
        return EitherGeoPosOrLLATup(Either::Right(self.clone()));
        #[cfg(not(feature = "py-bevy"))]
        return Either::Right(self.clone());
    }
    fn into_lla_tuple(self) -> (f64, f64, f64) {
        self.lla()
    }
    fn into_geo_pos(self) -> GeoPosition {
        self
    }
}
impl IntoEitherLLATupOrGeoPos for &GeoPosition {
    fn into_either(self) -> EitherGeoPosOrLLATup {
        #[cfg(feature = "py-bevy")]
        return EitherGeoPosOrLLATup(Either::Right(self.clone()));
        #[cfg(not(feature = "py-bevy"))]
        return Either::Right(self.clone());
    }
    fn into_lla_tuple(self) -> (f64, f64, f64) {
        self.lla()
    }
    fn into_geo_pos(self) -> GeoPosition {
        self.clone()
    }
}
impl IntoLatLonTriple for EitherGeoPosOrLLATup {
    fn into_lat_lon_triple(&self) -> (f64, f64, f64) {
        #[cfg(feature = "py-bevy")]
        let either = self.0;
        #[cfg(not(feature = "py-bevy"))]
        let either = self;

        match either {
            Either::Left(tup) => *tup,
            Either::Right(pos) => pos.lla(),
        }
    }
}
impl IntoLatLonTuple for EitherGeoPosOrLLATup {
    fn into_lat_lon_tuple(&self) -> (f64, f64) {
        #[cfg(feature = "py-bevy")]
        let either = self.0;
        #[cfg(not(feature = "py-bevy"))]
        let either = self;

        match either {
            Either::Left(tup) => (tup.0, tup.1),
            Either::Right(pos) => {
                let lla = pos.lla();
                (lla.0, lla.1)
            }
        }
    }
}

impl Into<EitherGeoPosOrLLATup> for GeoPosition {
    fn into(self) -> EitherGeoPosOrLLATup {
        #[cfg(feature = "py-bevy")]
        return EitherGeoPosOrLLATup(Either::Right(self));
        #[cfg(not(feature = "py-bevy"))]
        return Either::Right(self);
    }
}
impl Into<EitherGeoPosOrLLATup> for &GeoPosition {
    fn into(self) -> EitherGeoPosOrLLATup {
        #[cfg(feature = "py-bevy")]
        return EitherGeoPosOrLLATup(Either::Right(self.clone()));
        #[cfg(not(feature = "py-bevy"))]
        return Either::Right(self.clone());
    }
}
impl Into<GeoPosition> for EitherGeoPosOrLLATup {
    fn into(self) -> GeoPosition {
        #[cfg(feature = "py-bevy")]
        let either = self.0;
        #[cfg(not(feature = "py-bevy"))]
        let either = self;

        match either {
            Either::Left(lla_tup) => GeoPosition::from_lla(lla_tup),
            Either::Right(pos) => pos,
        }
    }
}

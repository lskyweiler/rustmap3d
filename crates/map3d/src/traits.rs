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
pub trait IntoEitherLLATupOrGeoPos {
    fn into_either(self) -> Either<(f64, f64, f64), GeoPosition>;
    fn into_lla_tuple(self) -> (f64, f64, f64);
    fn into_geo_pos(self) -> GeoPosition;
}

impl IntoEitherLLATupOrGeoPos for (f64, f64, f64) {
    fn into_either(self) -> Either<(f64, f64, f64), GeoPosition> {
        Either::Left(self)
    }
    fn into_lla_tuple(self) -> (f64, f64, f64) {
        self
    }
    fn into_geo_pos(self) -> GeoPosition {
        GeoPosition::from_lla(self)
    }
}
impl IntoEitherLLATupOrGeoPos for &(f64, f64, f64) {
    fn into_either(self) -> Either<(f64, f64, f64), GeoPosition> {
        Either::Left(*self)
    }
    fn into_lla_tuple(self) -> (f64, f64, f64) {
        *self
    }
    fn into_geo_pos(self) -> GeoPosition {
        GeoPosition::from_lla(*self)
    }
}
impl IntoEitherLLATupOrGeoPos for GeoPosition {
    fn into_either(self) -> Either<(f64, f64, f64), GeoPosition> {
        Either::Right(self.clone())
    }
    fn into_lla_tuple(self) -> (f64, f64, f64) {
        self.lla()
    }
    fn into_geo_pos(self) -> GeoPosition {
        self
    }
}
impl IntoEitherLLATupOrGeoPos for &GeoPosition {
    fn into_either(self) -> Either<(f64, f64, f64), GeoPosition> {
        Either::Right(self.clone())
    }
    fn into_lla_tuple(self) -> (f64, f64, f64) {
        self.lla()
    }
    fn into_geo_pos(self) -> GeoPosition {
        self.clone()
    }
}
impl IntoLatLonTriple for Either<(f64, f64, f64), GeoPosition> {
    fn into_lat_lon_triple(&self) -> (f64, f64, f64) {
        match self {
            Either::Left(tup) => *tup,
            Either::Right(pos) => pos.lla(),
        }
    }
}
impl IntoLatLonTuple for Either<(f64, f64, f64), GeoPosition> {
    fn into_lat_lon_tuple(&self) -> (f64, f64) {
        match self {
            Either::Left(tup) => (tup.0, tup.1),
            Either::Right(pos) => {
                let lla = pos.lla();
                (lla.0, lla.1)
            }
        }
    }
}

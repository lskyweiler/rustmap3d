use glam;

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

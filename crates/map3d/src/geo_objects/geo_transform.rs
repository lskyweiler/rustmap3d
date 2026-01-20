use crate::{
    geo_objects::{geo_orientation::GeoOrientation, geo_position::GeoPosition},
    traits::*,
    transforms::*,
};
use glam;

pub struct GeoTransform {
    position: GeoPosition,
    orientation: GeoOrientation,
}

impl GeoTransform {}

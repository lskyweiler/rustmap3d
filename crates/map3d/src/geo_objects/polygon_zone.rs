//! Geodetic polygon zone: a closed (lat, lon) ring extruded between two altitudes.
//!
//! Queries are evaluated in a local East-North-Up tangent frame anchored at the
//! ring centroid, so containment, segment intersection and distance are planar
//! geometry in meters. Tangent-plane error grows as `d^3 / (6 R^2)` with the
//! distance `d` from the centroid: about 4 m at 100 km, 17 m at 160 km and
//! 130 m at 300 km. The type is intended for zones up to a few hundred km across.
//!
//! Points more than 90 degrees from the centroid would fold back onto the
//! tangent plane (the antipode projects onto the centroid), so they are treated
//! as beyond the frame's horizon: never contained, and measured by straight-line
//! distance instead of planar distance.

use crate::traits::*;
use crate::transforms::*;
use crate::vincenty::vincenty_direct;
use core::fmt;
use glam::{DMat3, DVec2, DVec3};
#[cfg(not(feature = "pyo3"))]
use map3d_derive::*;
#[cfg(feature = "pyo3")]
use pyo3::{exceptions::PyValueError, prelude::*};
#[cfg(feature = "pyo3")]
use pyo3_stub_gen::derive::*;

/// Tolerance used when comparing projected coordinates (meters).
const EPS_M: f64 = 1e-9;
/// Default number of sides for [`PolygonZone::from_circle`].
pub const DEFAULT_CIRCLE_SIDES: usize = 25;
/// Default altitude safety buffer (meters). This is to prevent floating point error
/// from throwing containment false negatives (i.e., if a point is 1m under the zone,
/// then consider it to be in the zone.)
pub const DEFAULT_ALT_BUFFER_M: f64 = 1.0;

/// Reasons a [`PolygonZone`] could not be built.
#[derive(Debug, Clone)]
pub enum PolygonZoneError {
    /// The first and last vertex differ.
    RingNotClosed,
    /// Fewer than three distinct vertices.
    TooFewVertices,
    /// `alt_min_m > alt_max_m`, or an altitude is NaN.
    InvalidAltitude,
    /// Negative or NaN altitude buffer.
    InvalidBuffer,
    /// `from_circle` with fewer than three sides.
    InvalidSides,
    /// `from_circle` with a non-positive radius.
    InvalidRadius,
    /// A vertex or circle-centre coordinate is NaN or infinite.
    NonFinite,
    /// A vertex or circle-centre latitude is outside `[-90, 90]`.
    LatitudeOutOfRange,
    /// A vertex is 90 degrees or more from the ring centroid, so the ring
    /// cannot be represented in a single tangent plane.
    RingTooLarge,
}

impl fmt::Display for PolygonZoneError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::RingNotClosed => write!(f, "vertices must form a closed ring (first == last)"),
            Self::TooFewVertices => write!(f, "a polygon zone needs at least 3 distinct vertices"),
            Self::InvalidAltitude => write!(f, "alt_min_m must be <= alt_max_m and not NaN"),
            Self::InvalidBuffer => write!(f, "alt_buffer_m must be >= 0"),
            Self::InvalidSides => write!(f, "n_sides must be >= 3"),
            Self::InvalidRadius => write!(f, "radius_m must be > 0"),
            Self::NonFinite => write!(f, "coordinates must be finite"),
            Self::LatitudeOutOfRange => write!(f, "latitudes must be in [-90, 90] degrees"),
            Self::RingTooLarge => write!(
                f,
                "every vertex must be less than 90 degrees from the ring centroid"
            ),
        }
    }
}

/// A closed `(lat, lon)` ring in degrees, extruded between `alt_min_m` and `alt_max_m`.
///
/// Containment, segment intersection and distance are computed in a local
/// East-North-Up tangent frame at the ring centroid. The horizontal projection
/// is taken at the floor altitude so a vertical column of points maps to one
/// planar point; altitude is tested separately against the band
/// `[alt_min_m - alt_buffer_m, alt_max_m + alt_buffer_m]` (inclusive).
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "pyo3", gen_stub_pyclass)]
#[cfg_attr(
    all(feature = "pyo3", feature = "set-pyclass-module"),
    pyclass(module = "rustmap3d", from_py_object)
)]
#[cfg_attr(
    all(feature = "pyo3", not(feature = "set-pyclass-module")),
    pyclass(from_py_object)
)]
#[cfg_attr(not(feature = "pyo3"), derive(DummyPyO3))]
pub struct PolygonZone {
    /// Closed `(lat_deg, lon_deg)` ring exactly as supplied; first == last.
    ring: Vec<(f64, f64)>,
    /// Floor altitude [m], without the buffer.
    alt_min_m: f64,
    /// Ceiling altitude [m], without the buffer; may be `+inf`.
    alt_max_m: f64,
    /// Margin added below the floor and above the ceiling [m].
    alt_buffer_m: f64,
    /// Altitude of the tangent plane [m]: `alt_min_m` if finite, else 0. Every
    /// point is projected at this altitude so a vertical column maps to one
    /// planar point.
    ref_alt_m: f64,
    /// ECEF origin of the ENU frame: the ring centroid at `ref_alt_m` [m].
    ref_ecef: DVec3,
    /// Rotation from ECEF offsets to East-North-Up at `ref_ecef`.
    ecef2enu: DMat3,
    /// Height of `ref_ecef` above the plane through the Earth's centre that is
    /// normal to the local up axis [m]. A point is on the near side of the frame
    /// when its up coordinate exceeds `-horizon_m` (less than 90 degrees away).
    horizon_m: f64,
    /// Ring projected to (east, north) [m]; closed like `ring`.
    local: Vec<DVec2>,
    /// South-west corner of the bounding box of `local` [m], for fast rejection.
    local_min: DVec2,
    /// North-east corner of the bounding box of `local` [m], for fast rejection.
    local_max: DVec2,
}

impl PolygonZone {
    /// Build a zone from a closed ring of `(lat_deg, lon_deg)` vertices.
    ///
    /// # Arguments
    ///
    /// - `ring` (`Vec<(f64, f64)>`) - Closed ring: the last vertex must equal the first [[degrees]]
    /// - `alt_min_m` (`f64`) - Floor altitude [[meters]]
    /// - `alt_max_m` (`f64`) - Ceiling altitude, may be infinite [[meters]]
    /// - `alt_buffer_m` (`f64`) - Safety margin added below the floor and above the ceiling [[meters]]
    pub fn from_vertices(
        ring: Vec<(f64, f64)>,
        alt_min_m: f64,
        alt_max_m: f64,
        alt_buffer_m: f64,
    ) -> Result<Self, PolygonZoneError> {
        if ring.iter().any(|v| !v.0.is_finite() || !v.1.is_finite()) {
            return Err(PolygonZoneError::NonFinite);
        }
        if ring.iter().any(|v| v.0.abs() > 90.0) {
            return Err(PolygonZoneError::LatitudeOutOfRange);
        }
        if alt_min_m.is_nan() || alt_max_m.is_nan() || alt_min_m > alt_max_m {
            return Err(PolygonZoneError::InvalidAltitude);
        }
        if alt_buffer_m.is_nan() || alt_buffer_m < 0.0 {
            return Err(PolygonZoneError::InvalidBuffer);
        }
        if ring.len() < 4 {
            // Minimum for polygon is 3 + 1 for closure.
            return Err(PolygonZoneError::TooFewVertices);
        }
        if ring[0] != ring[ring.len() - 1] {
            return Err(PolygonZoneError::RingNotClosed);
        }
        let mut distinct: Vec<(f64, f64)> = Vec::new();
        for &v in &ring[..ring.len() - 1] {
            if !distinct.contains(&v) {
                distinct.push(v);
            }
        }
        if distinct.len() < 3 {
            return Err(PolygonZoneError::TooFewVertices);
        }

        let ref_alt_m = if alt_min_m.is_finite() {
            alt_min_m
        } else {
            0.0
        };
        // Centroid through ECEF so rings across the antimeridian get a sane frame.
        let mean_ecef = distinct
            .iter()
            .map(|&(lat, lon)| lla2ecef((lat, lon, ref_alt_m)))
            .sum::<DVec3>()
            / distinct.len() as f64;
        let c = ecef2lla(mean_ecef);
        let ref_lla = (c.x, c.y, ref_alt_m);
        let ref_ecef = lla2ecef(ref_lla);
        let ecef2enu = ecef2enu_dcm(ref_lla);
        // Up coordinate of the Earth's centre is -(up . ref_ecef).
        let horizon_m = (ecef2enu * ref_ecef).z;

        let mut zone = Self {
            ring,
            alt_min_m,
            alt_max_m,
            alt_buffer_m,
            ref_alt_m,
            ref_ecef,
            ecef2enu,
            horizon_m,
            local: Vec::new(),
            local_min: DVec2::ZERO,
            local_max: DVec2::ZERO,
        };
        let enu: Vec<DVec3> = zone
            .ring
            .iter()
            .map(|&(lat, lon)| zone.project_enu(lat, lon))
            .collect();
        if enu.iter().any(|p| !zone.on_near_side(p.z)) {
            return Err(PolygonZoneError::RingTooLarge);
        }
        zone.local = enu.iter().map(|p| p.truncate()).collect();
        zone.local_min = zone.local.iter().fold(DVec2::INFINITY, |m, p| m.min(*p));
        zone.local_max = zone
            .local
            .iter()
            .fold(DVec2::NEG_INFINITY, |m, p| m.max(*p));
        Ok(zone)
    }

    /// Build a regular polygon inscribed in a geodesic circle.
    ///
    /// Vertices lie exactly on the circle at evenly spaced bearings starting at
    /// true north, so points between the inradius `r cos(pi / n_sides)` and `r`
    /// near an edge midpoint are outside the zone.
    ///
    /// # Arguments
    ///
    /// - `lat_deg`, `lon_deg` (`f64`) - Circle centre [[degrees]]
    /// - `radius_m` (`f64`) - Circle radius [[meters]]
    /// - `alt_min_m`, `alt_max_m` (`f64`) - Floor and ceiling [[meters]]
    /// - `n_sides` (`usize`) - Number of polygon sides, at least 3
    /// - `alt_buffer_m` (`f64`) - Altitude safety margin [[meters]]
    pub fn from_circle(
        lat_deg: f64,
        lon_deg: f64,
        radius_m: f64,
        alt_min_m: f64,
        alt_max_m: f64,
        n_sides: usize,
        alt_buffer_m: f64,
    ) -> Result<Self, PolygonZoneError> {
        if n_sides < 3 {
            return Err(PolygonZoneError::InvalidSides);
        }
        if radius_m.is_nan() || radius_m <= 0.0 || !radius_m.is_finite() {
            return Err(PolygonZoneError::InvalidRadius);
        }
        if !lat_deg.is_finite() || !lon_deg.is_finite() {
            return Err(PolygonZoneError::NonFinite);
        }
        if lat_deg.abs() > 90.0 {
            return Err(PolygonZoneError::LatitudeOutOfRange);
        }
        let mut ring = Vec::with_capacity(n_sides + 1);
        for i in 0..n_sides {
            let bearing = 360.0 * i as f64 / n_sides as f64;
            // Inputs were validated above, so Vincenty's domain checks cannot fail.
            let (lat, lon) = vincenty_direct(lat_deg, lon_deg, radius_m, bearing, 1e-12, 200)
                .expect("circle centre and radius already validated");
            ring.push((lat, wrap_lon(lon)));
        }
        ring.push(ring[0]);
        Self::from_vertices(ring, alt_min_m, alt_max_m, alt_buffer_m)
    }

    /// Project a geodetic point to (east, north, up) meters at the frame's reference altitude.
    fn project_enu(&self, lat_deg: f64, lon_deg: f64) -> DVec3 {
        let ecef = lla2ecef((lat_deg, lon_deg, self.ref_alt_m));
        self.ecef2enu * (ecef - self.ref_ecef)
    }

    /// Is a projected point (by its up coordinate) less than 90 degrees from the
    /// centroid? Far-side points fold back onto the plane, so their (east, north)
    /// is meaningless. NaN compares false, so it counts as far side.
    fn on_near_side(&self, up_m: f64) -> bool {
        up_m > -self.horizon_m
    }

    /// Straight-line ECEF distance from a point to the nearest ring vertex, with
    /// the vertex altitude clamped to the buffered band. Used for points beyond
    /// the frame's horizon, where planar distance is meaningless.
    fn far_side_distance(&self, lat_deg: f64, lon_deg: f64, alt_m: f64) -> f64 {
        let p = lla2ecef((lat_deg, lon_deg, alt_m));
        let (lo, hi) = self.alt_band();
        let v_alt = alt_m.clamp(lo, hi);
        self.ring
            .iter()
            .map(|&(lat, lon)| lla2ecef((lat, lon, v_alt)).distance(p))
            .fold(f64::INFINITY, f64::min)
    }

    fn alt_band(&self) -> (f64, f64) {
        (
            self.alt_min_m - self.alt_buffer_m,
            self.alt_max_m + self.alt_buffer_m,
        )
    }

    fn in_alt_band(&self, alt_m: f64) -> bool {
        let (lo, hi) = self.alt_band();
        alt_m >= lo && alt_m <= hi
    }

    /// Even-odd ray cast against the projected ring, with a bounding-box reject.
    fn contains_local(&self, p: DVec2) -> bool {
        if p.x < self.local_min.x - EPS_M
            || p.x > self.local_max.x + EPS_M
            || p.y < self.local_min.y - EPS_M
            || p.y > self.local_max.y + EPS_M
        {
            return false;
        }
        let mut inside = false;
        for w in self.local.windows(2) {
            let (a, b) = (w[0], w[1]);
            if (a.y > p.y) != (b.y > p.y) {
                let x = a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y);
                if p.x < x {
                    inside = !inside;
                }
            }
        }
        inside
    }

    /// Planar distance from `p` to the ring boundary.
    fn boundary_distance_local(&self, p: DVec2) -> f64 {
        self.local
            .windows(2)
            .map(|w| point_segment_distance(p, w[0], w[1]))
            .fold(f64::INFINITY, f64::min)
    }
}

#[cfg_attr(feature = "pyo3", gen_stub_pymethods, pymethods)]
impl PolygonZone {
    /// The closed `(lat_deg, lon_deg)` ring exactly as supplied.
    #[getter]
    pub fn vertices(&self) -> Vec<(f64, f64)> {
        self.ring.clone()
    }

    /// Floor altitude in meters (without the safety buffer).
    #[getter]
    pub fn alt_min_m(&self) -> f64 {
        self.alt_min_m
    }

    /// Ceiling altitude in meters (without the safety buffer).
    #[getter]
    pub fn alt_max_m(&self) -> f64 {
        self.alt_max_m
    }

    /// Altitude safety buffer in meters applied below the floor and above the ceiling.
    #[getter]
    pub fn alt_buffer_m(&self) -> f64 {
        self.alt_buffer_m
    }

    /// Is the point inside the zone?
    ///
    /// # Arguments
    ///
    /// - `point` (`tuple[float, float, float] | GeoPosition`) - Query point; tuples are `(lat_deg, lon_deg, alt_m)`
    ///
    /// # Returns
    ///
    /// - `bool` - True if the point is inside the footprint and within the buffered altitude band
    pub fn contains(&self, point: EitherGeoPosOrLLATup) -> bool {
        let (lat, lon, alt) = point.into_lat_lon_triple();
        if !self.in_alt_band(alt) {
            return false;
        }
        let enu = self.project_enu(lat, lon);
        self.on_near_side(enu.z) && self.contains_local(enu.truncate())
    }

    /// Shortest distance from the point to the zone, in meters (0 when inside).
    ///
    /// # Arguments
    ///
    /// - `point` (`tuple[float, float, float] | GeoPosition`) - Query point; tuples are `(lat_deg, lon_deg, alt_m)`
    ///
    /// # Returns
    ///
    /// - `float` - Euclidean distance in the local frame to the extruded solid; for points more than 90 degrees from the centroid, straight-line distance to the nearest vertex [[meters]]
    pub fn distance_to(&self, point: EitherGeoPosOrLLATup) -> f64 {
        let (lat, lon, alt) = point.into_lat_lon_triple();
        let enu = self.project_enu(lat, lon);
        if !self.on_near_side(enu.z) {
            return self.far_side_distance(lat, lon, alt);
        }
        let p = enu.truncate();
        let horizontal = if self.contains_local(p) {
            0.0
        } else {
            self.boundary_distance_local(p)
        };
        let (lo, hi) = self.alt_band();
        let vertical = (lo - alt).max(alt - hi).max(0.0);
        horizontal.hypot(vertical)
    }

    /// Does the straight segment between two points pass through the zone?
    ///
    /// # Arguments
    ///
    /// - `a`, `b` (`tuple[float, float, float] | GeoPosition`) - Segment endpoints; tuples are `(lat_deg, lon_deg, alt_m)`
    ///
    /// # Returns
    ///
    /// - `bool` - True if any part of the segment lies inside the zone
    pub fn intersects_segment(&self, a: EitherGeoPosOrLLATup, b: EitherGeoPosOrLLATup) -> bool {
        let (lat1, lon1, alt1) = a.into_lat_lon_triple();
        let (lat2, lon2, alt2) = b.into_lat_lon_triple();
        let e1 = self.project_enu(lat1, lon1);
        let e2 = self.project_enu(lat2, lon2);
        let (p1, p2) = (e1.truncate(), e2.truncate());

        // Clip the parametric segment to the altitude band.
        let (lo, hi) = self.alt_band();
        let dalt = alt2 - alt1;
        let (mut t0, mut t1) = if dalt.abs() < 1e-12 {
            if alt1 < lo || alt1 > hi {
                return false;
            }
            (0.0, 1.0)
        } else {
            let ta = (lo - alt1) / dalt;
            let tb = (hi - alt1) / dalt;
            (ta.min(tb).max(0.0), ta.max(tb).min(1.0))
        };

        // Clip to the near side of the frame so a far-side endpoint, whose
        // projection has folded back onto the plane, cannot fake a crossing.
        let (n1, n2) = (self.on_near_side(e1.z), self.on_near_side(e2.z));
        if !n1 && !n2 {
            return false;
        }
        if n1 != n2 {
            let t_h = (-self.horizon_m - e1.z) / (e2.z - e1.z);
            if n1 {
                t1 = t1.min(t_h);
            } else {
                t0 = t0.max(t_h);
            }
        }
        if !(t0 <= t1) {
            return false;
        }
        let c1 = p1.lerp(p2, t0);
        let c2 = p1.lerp(p2, t1);

        if self.contains_local(c1) || self.contains_local(c2) {
            return true;
        }
        if c1.distance_squared(c2) < EPS_M * EPS_M {
            return false;
        }
        self.local
            .windows(2)
            .any(|w| segments_cross(c1, c2, w[0], w[1]))
    }

    /// Python repr.
    fn __repr__(&self) -> String {
        format!(
            "PolygonZone({} vertices, alt={}-{} m, buffer={} m)",
            self.ring.len() - 1,
            self.alt_min_m,
            self.alt_max_m,
            self.alt_buffer_m
        )
    }

    /// Construct from a closed ring of `(lat_deg, lon_deg)` vertices.
    ///
    /// # Arguments
    ///
    /// - `vertices` (`list[tuple[float, float]]`) - Closed ring: the last vertex must equal the first [[degrees]]
    /// - `alt_min_m` (`float`) - Floor altitude [[meters]]
    /// - `alt_max_m` (`float`) - Ceiling altitude, may be `inf` [[meters]]
    /// - `alt_buffer_m` (`float`) - Safety margin added below the floor and above the ceiling [[meters]]
    ///
    /// # Raises
    ///
    /// - `ValueError` if the ring is not closed, has fewer than 3 distinct vertices, has a latitude outside [-90, 90], spans 90 degrees or more from its centroid, or the altitudes are invalid
    #[cfg(feature = "pyo3")]
    #[new]
    #[pyo3(signature = (vertices, alt_min_m, alt_max_m, alt_buffer_m = DEFAULT_ALT_BUFFER_M))]
    fn py_new(
        vertices: Vec<(f64, f64)>,
        alt_min_m: f64,
        alt_max_m: f64,
        alt_buffer_m: f64,
    ) -> PyResult<Self> {
        Self::from_vertices(vertices, alt_min_m, alt_max_m, alt_buffer_m)
            .map_err(|e| PyValueError::new_err(e.to_string()))
    }

    /// Regular polygon inscribed in a geodesic circle.
    ///
    /// Vertices lie on the circle at evenly spaced bearings starting at true
    /// north, so points between the inradius `r cos(pi / n_sides)` and `r` near
    /// an edge midpoint are outside the zone.
    ///
    /// # Arguments
    ///
    /// - `lat_deg`, `lon_deg` (`float`) - Circle centre [[degrees]]
    /// - `radius_m` (`float`) - Circle radius [[meters]]
    /// - `alt_min_m`, `alt_max_m` (`float`) - Floor and ceiling [[meters]]
    /// - `n_sides` (`int`) - Number of polygon sides, at least 3
    /// - `alt_buffer_m` (`float`) - Altitude safety margin [[meters]]
    ///
    /// # Raises
    ///
    /// - `ValueError` if the radius or side count is invalid, or the centre is not a valid lat/lon
    #[cfg(feature = "pyo3")]
    #[staticmethod]
    #[pyo3(name = "from_circle", signature = (lat_deg, lon_deg, radius_m, alt_min_m, alt_max_m, n_sides = DEFAULT_CIRCLE_SIDES, alt_buffer_m = DEFAULT_ALT_BUFFER_M))]
    fn py_from_circle(
        lat_deg: f64,
        lon_deg: f64,
        radius_m: f64,
        alt_min_m: f64,
        alt_max_m: f64,
        n_sides: usize,
        alt_buffer_m: f64,
    ) -> PyResult<Self> {
        Self::from_circle(
            lat_deg,
            lon_deg,
            radius_m,
            alt_min_m,
            alt_max_m,
            n_sides,
            alt_buffer_m,
        )
        .map_err(|e| PyValueError::new_err(e.to_string()))
    }

    /// Two zones are equal when their rings, altitudes and buffers are identical.
    #[cfg(feature = "pyo3")]
    fn __eq__(&self, rhs: PolygonZone) -> bool {
        self.ring == rhs.ring
            && self.alt_min_m == rhs.alt_min_m
            && self.alt_max_m == rhs.alt_max_m
            && self.alt_buffer_m == rhs.alt_buffer_m
    }

    /// Support for pickle/deepcopy.
    #[cfg(feature = "pyo3")]
    fn __getnewargs__(&self) -> PyResult<(Vec<(f64, f64)>, f64, f64, f64)> {
        Ok((
            self.ring.clone(),
            self.alt_min_m,
            self.alt_max_m,
            self.alt_buffer_m,
        ))
    }
}

/// Wrap a longitude in degrees to `(-180, 180]`.
fn wrap_lon(lon_deg: f64) -> f64 {
    let w = (lon_deg + 180.0).rem_euclid(360.0) - 180.0;
    if w == -180.0 {
        180.0
    } else {
        w
    }
}

/// Distance from `p` to the segment `a..b`; a zero-length segment is a point.
fn point_segment_distance(p: DVec2, a: DVec2, b: DVec2) -> f64 {
    let ab = b - a;
    let len_sq = ab.length_squared();
    if len_sq < EPS_M * EPS_M {
        return p.distance(a);
    }
    let t = ((p - a).dot(ab) / len_sq).clamp(0.0, 1.0);
    p.distance(a + ab * t)
}

/// Do the segments `p..q` and `a..b` cross? Parallel segments never cross here;
/// collinear overlap is caught by the endpoint containment test that precedes
/// this in `intersects_segment`.
fn segments_cross(p: DVec2, q: DVec2, a: DVec2, b: DVec2) -> bool {
    let d1 = q - p;
    let d2 = b - a;
    let denom = d1.perp_dot(d2);
    if denom.abs() < EPS_M {
        return false;
    }
    let r = a - p;
    let t = r.perp_dot(d2) / denom;
    let u = r.perp_dot(d1) / denom;
    (-EPS_M..=1.0 + EPS_M).contains(&t) && (-EPS_M..=1.0 + EPS_M).contains(&u)
}

#[cfg(test)]
mod test_polygon_zone {
    use super::*;
    use crate::geo_objects::geo_position::GeoPosition;
    use crate::vincenty_inverse;
    use rstest::*;

    const INF: f64 = f64::INFINITY;

    /// Square about 11 km on a side centred on the equator / prime meridian.
    fn square() -> Vec<(f64, f64)> {
        vec![
            (-0.05, -0.05),
            (-0.05, 0.05),
            (0.05, 0.05),
            (0.05, -0.05),
            (-0.05, -0.05),
        ]
    }

    fn square_zone(alt_min: f64, alt_max: f64) -> PolygonZone {
        PolygonZone::from_vertices(square(), alt_min, alt_max, 1.0).unwrap()
    }

    /// L shape: a 2x2 square with the north-east quadrant removed.
    fn l_shape() -> Vec<(f64, f64)> {
        vec![
            (37.0, -78.0),
            (37.0, -76.0),
            (38.0, -76.0),
            (38.0, -77.0),
            (39.0, -77.0),
            (39.0, -78.0),
            (37.0, -78.0),
        ]
    }

    fn dest(lat: f64, lon: f64, bearing: f64, range_m: f64) -> (f64, f64) {
        vincenty_direct(lat, lon, range_m, bearing, 1e-12, 200).unwrap()
    }

    // --- constructor validation -------------------------------------------

    #[test]
    fn unclosed_ring_is_rejected() {
        let mut ring = square();
        ring.pop();
        assert!(matches!(
            PolygonZone::from_vertices(ring, 0.0, 1000.0, 1.0),
            Err(PolygonZoneError::RingNotClosed)
        ));
    }

    #[test]
    fn fewer_than_three_distinct_vertices_is_rejected() {
        let ring = vec![(0.0, 0.0), (0.0, 1.0), (0.0, 0.0)];
        assert!(matches!(
            PolygonZone::from_vertices(ring, 0.0, 1000.0, 1.0),
            Err(PolygonZoneError::TooFewVertices)
        ));
        let ring = vec![(0.0, 0.0), (0.0, 1.0), (0.0, 1.0), (0.0, 0.0)];
        assert!(matches!(
            PolygonZone::from_vertices(ring, 0.0, 1000.0, 1.0),
            Err(PolygonZoneError::TooFewVertices)
        ));
    }

    #[test]
    fn inverted_altitude_band_is_rejected() {
        assert!(matches!(
            PolygonZone::from_vertices(square(), 1000.0, 0.0, 1.0),
            Err(PolygonZoneError::InvalidAltitude)
        ));
    }

    #[test]
    fn non_finite_vertex_is_rejected() {
        let mut ring = square();
        ring[1].0 = f64::NAN;
        assert!(matches!(
            PolygonZone::from_vertices(ring, 0.0, 1000.0, 1.0),
            Err(PolygonZoneError::NonFinite)
        ));
    }

    #[test]
    fn triangle_is_accepted() {
        let ring = vec![(0.0, 0.0), (0.0, 0.1), (0.1, 0.05), (0.0, 0.0)];
        let z = PolygonZone::from_vertices(ring, 0.0, 1000.0, 1.0).unwrap();
        assert!(z.contains((0.03, 0.05, 500.0).into_either()));
        assert!(!z.contains((0.09, 0.0, 500.0).into_either()));
    }

    #[test]
    fn vertex_latitude_out_of_range_is_rejected() {
        let ring = vec![(89.0, 0.0), (89.0, 90.0), (95.0, 45.0), (89.0, 0.0)];
        assert!(matches!(
            PolygonZone::from_vertices(ring, 0.0, 1000.0, 1.0),
            Err(PolygonZoneError::LatitudeOutOfRange)
        ));
    }

    #[test]
    fn circle_centre_is_validated() {
        assert!(matches!(
            PolygonZone::from_circle(91.0, 0.0, 1000.0, 0.0, 100.0, 25, 1.0),
            Err(PolygonZoneError::LatitudeOutOfRange)
        ));
        assert!(matches!(
            PolygonZone::from_circle(f64::NAN, 0.0, 1000.0, 0.0, 100.0, 25, 1.0),
            Err(PolygonZoneError::NonFinite)
        ));
        assert!(matches!(
            PolygonZone::from_circle(0.0, f64::INFINITY, 1000.0, 0.0, 100.0, 25, 1.0),
            Err(PolygonZoneError::NonFinite)
        ));
    }

    #[test]
    fn hemisphere_sized_ring_is_rejected() {
        // Centroid is near (0, 0); the vertex at lon 170 is ~170 degrees away.
        let ring = vec![
            (-10.0, -10.0),
            (-10.0, 10.0),
            (10.0, 10.0),
            (0.0, 170.0),
            (10.0, -10.0),
            (-10.0, -10.0),
        ];
        assert!(matches!(
            PolygonZone::from_vertices(ring, 0.0, 1000.0, 1.0),
            Err(PolygonZoneError::RingTooLarge)
        ));
    }

    #[test]
    fn negative_buffer_is_rejected() {
        assert!(matches!(
            PolygonZone::from_vertices(square(), 0.0, 1000.0, -1.0),
            Err(PolygonZoneError::InvalidBuffer)
        ));
    }

    #[test]
    fn circle_needs_three_sides_and_positive_radius() {
        assert!(matches!(
            PolygonZone::from_circle(0.0, 0.0, 1000.0, 0.0, 100.0, 2, 1.0),
            Err(PolygonZoneError::InvalidSides)
        ));
        assert!(matches!(
            PolygonZone::from_circle(0.0, 0.0, 0.0, 0.0, 100.0, 25, 1.0),
            Err(PolygonZoneError::InvalidRadius)
        ));
    }

    #[test]
    fn infinite_ceiling_is_allowed() {
        let z = square_zone(0.0, INF);
        assert!(z.contains((0.0, 0.0, 1e7).into_either()));
        assert_eq!(z.alt_max_m(), INF);
    }

    #[test]
    fn getters_return_the_inputs_unchanged() {
        let z = PolygonZone::from_vertices(square(), 10.0, 500.0, 2.5).unwrap();
        assert_eq!(z.vertices(), square());
        assert_eq!(z.alt_min_m(), 10.0);
        assert_eq!(z.alt_max_m(), 500.0);
        assert_eq!(z.alt_buffer_m(), 2.5);
    }

    // --- containment -----------------------------------------------------------

    #[test]
    fn square_inside_and_outside() {
        let z = square_zone(0.0, 10_000.0);
        assert!(z.contains((0.0, 0.0, 5000.0).into_either()));
        assert!(z.contains((0.04, -0.04, 5000.0).into_either()));
        assert!(!z.contains((0.06, 0.0, 5000.0).into_either()));
        assert!(!z.contains((0.0, -0.06, 5000.0).into_either()));
    }

    #[test]
    fn concave_notch_is_outside_for_both_orientations() {
        let ccw = PolygonZone::from_vertices(l_shape(), 0.0, INF, 1.0).unwrap();
        let mut rev = l_shape();
        rev.reverse();
        let cw = PolygonZone::from_vertices(rev, 0.0, INF, 1.0).unwrap();
        for z in [&ccw, &cw] {
            assert!(z.contains((37.5, -76.5, 100.0).into_either()));
            assert!(z.contains((38.5, -77.5, 100.0).into_either()));
            assert!(!z.contains((38.5, -76.5, 100.0).into_either()));
        }
    }

    #[test]
    fn horizontal_verdict_does_not_depend_on_altitude() {
        let z = square_zone(0.0, 12_000.0);
        // The east edge runs along lon 0.05 (about 5565 m east of the centre).
        let (lat, lon) = dest(0.0, 0.0, 90.0, 5565.0 - 20.0);
        for alt in [0.0, 6_000.0, 11_999.0] {
            assert!(z.contains((lat, lon, alt).into_either()), "alt {alt}");
        }
        let (lat, lon) = dest(0.0, 0.0, 90.0, 5565.0 + 20.0);
        for alt in [0.0, 6_000.0, 11_999.0] {
            assert!(!z.contains((lat, lon, alt).into_either()), "alt {alt}");
        }
    }

    #[test]
    fn geo_position_and_tuple_inputs_agree() {
        let z = square_zone(0.0, 10_000.0);
        for p in [
            (0.0, 0.0, 5000.0),
            (0.06, 0.0, 5000.0),
            (0.0, 0.0, 20_000.0),
        ] {
            let pos = GeoPosition::from_lla(p);
            assert_eq!(z.contains(p.into_either()), z.contains(pos.into_either()));
            let dt = z.distance_to(p.into_either());
            let dp = z.distance_to(pos.into_either());
            assert!((dt - dp).abs() < 1e-6, "{dt} vs {dp}");
        }
    }

    #[test]
    fn altitude_buffer_widens_the_band() {
        let with = PolygonZone::from_vertices(square(), 0.0, 1000.0, 1.0).unwrap();
        let without = PolygonZone::from_vertices(square(), 0.0, 1000.0, 0.0).unwrap();
        assert!(with.contains((0.0, 0.0, 1000.5).into_either()));
        assert!(!without.contains((0.0, 0.0, 1000.5).into_either()));
        assert!(with.contains((0.0, 0.0, -0.5).into_either()));
        assert!(!without.contains((0.0, 0.0, -0.5).into_either()));
        // Nominal bounds are reported, not the widened ones.
        assert_eq!(with.alt_max_m(), 1000.0);
    }

    // --- segments ---------------------------------------------------------------

    fn seg(z: &PolygonZone, a: (f64, f64, f64), b: (f64, f64, f64)) -> bool {
        z.intersects_segment(a.into_either(), b.into_either())
    }

    #[rstest]
    #[case((-0.1, 0.0, 5000.0), (0.1, 0.0, 5000.0), true)] // through, both ends outside
    #[case((0.1, -0.1, 5000.0), (0.1, 0.1, 5000.0), false)] // misses to the north
    #[case((-0.1, 0.0, 15_000.0), (0.1, 0.0, 15_000.0), false)] // above the band
    #[case((-0.1, 0.0, -5_000.0), (0.1, 0.0, -5_000.0), false)] // below the band
    #[case((0.0, 0.0, 15_000.0), (0.0, 0.0, 5_000.0), true)] // descends into the band
    #[case((0.0, 0.0, 15_000.0), (0.0, 0.0, -5_000.0), true)] // vertical through, inside footprint
    #[case((0.2, 0.2, 15_000.0), (0.2, 0.2, -5_000.0), false)] // vertical, outside footprint
    #[case((0.01, 0.01, 5000.0), (-0.01, -0.01, 5000.0), true)] // both ends inside
    #[case((-0.1, -0.1, 5000.0), (0.1, 0.1, 5000.0), true)] // diagonal through two vertices
    #[case((-0.1, 0.0, 15_000.0), (0.0, 0.0, 9_999.0), true)] // enters the band only at the far end, over the footprint
    fn segment_cases(#[case] a: (f64, f64, f64), #[case] b: (f64, f64, f64), #[case] hit: bool) {
        let z = square_zone(0.0, 10_000.0);
        assert_eq!(seg(&z, a, b), hit);
    }

    #[test]
    fn zero_length_segment_is_containment() {
        let z = square_zone(0.0, 10_000.0);
        assert!(seg(&z, (0.0, 0.0, 5000.0), (0.0, 0.0, 5000.0)));
        assert!(!seg(&z, (0.2, 0.0, 5000.0), (0.2, 0.0, 5000.0)));
    }

    // --- distance ---------------------------------------------------------------

    #[test]
    fn distance_is_zero_inside() {
        let z = square_zone(0.0, 10_000.0);
        assert_eq!(z.distance_to((0.0, 0.0, 5000.0).into_either()), 0.0);
    }

    #[test]
    fn distance_beside_above_and_combined() {
        let z = PolygonZone::from_vertices(square(), 0.0, 10_000.0, 0.0).unwrap();
        // East edge runs along lon 0.05; 1 km further east.
        let (lat, lon) = dest(0.0, 0.05, 90.0, 1000.0);
        let beside = z.distance_to((lat, lon, 5000.0).into_either());
        assert!((beside - 1000.0).abs() < 1.0, "beside {beside}");
        let above = z.distance_to((0.0, 0.0, 11_000.0).into_either());
        assert!((above - 1000.0).abs() < 1e-6, "above {above}");
        let both = z.distance_to((lat, lon, 11_000.0).into_either());
        assert!((both - 1000.0 * 2f64.sqrt()).abs() < 1.0, "both {both}");
    }

    // --- from_circle -------------------------------------------------------------

    #[test]
    fn circle_vertices_lie_on_the_radius_and_ring_is_closed() {
        let z = PolygonZone::from_circle(36.15, -115.1, 20_000.0, 0.0, 5000.0, 25, 1.0).unwrap();
        let ring = z.vertices();
        assert_eq!(ring.len(), 26);
        assert_eq!(ring[0], ring[25]);
        for &(lat, lon) in &ring[..25] {
            let (d, _, _) = vincenty_inverse(36.15, -115.1, lat, lon, 1e-12, 200).unwrap();
            assert!((d - 20_000.0).abs() < 0.01, "vertex at {d} m");
        }
        let eight = PolygonZone::from_circle(36.15, -115.1, 20_000.0, 0.0, 5000.0, 8, 1.0).unwrap();
        assert_eq!(eight.vertices().len(), 9);
    }

    #[test]
    fn circle_is_an_inscribed_polygon() {
        let r = 20_000.0;
        let z = PolygonZone::from_circle(36.15, -115.1, r, 0.0, 5000.0, 25, 1.0).unwrap();
        for i in 0..36 {
            let (lat, lon) = dest(36.15, -115.1, 10.0 * i as f64, 0.99 * r);
            assert!(
                z.contains((lat, lon, 1000.0).into_either()),
                "bearing {}",
                10 * i
            );
            let (lat, lon) = dest(36.15, -115.1, 10.0 * i as f64, 1.001 * r);
            assert!(
                !z.contains((lat, lon, 1000.0).into_either()),
                "bearing {}",
                10 * i
            );
        }
        // Midway between vertex 0 (bearing 0) and vertex 1 (bearing 14.4): the
        // inradius is r cos(pi/25) = 0.9921 r, so 0.995 r is outside the polygon.
        let (lat, lon) = dest(36.15, -115.1, 7.2, 0.995 * r);
        assert!(!z.contains((lat, lon, 1000.0).into_either()));
    }

    #[test]
    fn circle_across_the_antimeridian() {
        let z = PolygonZone::from_circle(0.0, 179.95, 20_000.0, 0.0, 5000.0, 25, 1.0).unwrap();
        assert!(z.contains((0.0, -179.99, 1000.0).into_either()));
        assert!(z.contains((0.0, 179.9, 1000.0).into_either()));
        assert!(!z.contains((0.0, 179.5, 1000.0).into_either()));
        assert!(z.vertices().iter().all(|v| v.1 > -180.0 && v.1 <= 180.0));
    }

    /// 2 x 2 degree box straddling the antimeridian, NW -> NE -> SE -> SW.
    fn pacific_box() -> PolygonZone {
        let ring = vec![
            (10.0, 179.0),
            (10.0, -179.0),
            (8.0, -179.0),
            (8.0, 179.0),
            (10.0, 179.0),
        ];
        PolygonZone::from_vertices(ring, 0.0, 10_000.0, 1.0).unwrap()
    }

    #[test]
    fn box_across_the_antimeridian_takes_the_short_way() {
        let z = pacific_box();
        for lon in [179.01, 179.5, 180.0, -180.0, -179.5, -179.01] {
            assert!(z.contains((9.0, lon, 1000.0).into_either()), "lon {lon}");
        }
        // Just outside each side, and the long way round (lon 0 lies "between"
        // 179 and -179 if longitudes were interpolated naively).
        for (lat, lon) in [(9.0, 178.9), (9.0, -178.9), (10.1, 180.0), (7.9, 180.0), (9.0, 0.0)] {
            assert!(!z.contains((lat, lon, 1000.0).into_either()), "({lat}, {lon})");
        }
        // A segment along lat 9 from 178.5 to -178.5 crosses the 2 degree box.
        assert!(seg(&z, (9.0, 178.5, 1000.0), (9.0, -178.5, 1000.0)));
        // East of the box, going further east: never touches it.
        assert!(!seg(&z, (9.0, -178.5, 1000.0), (9.0, -170.0, 1000.0)));
    }

    #[test]
    fn same_box_with_longitudes_in_0_to_360_is_identical() {
        let ring = vec![
            (10.0, 179.0),
            (10.0, 181.0),
            (8.0, 181.0),
            (8.0, 179.0),
            (10.0, 179.0),
        ];
        let a = PolygonZone::from_vertices(ring, 0.0, 10_000.0, 1.0).unwrap();
        let b = pacific_box();
        for lon in [178.9, 179.5, 180.0, -179.5, -178.9, 0.0] {
            let p = (9.0, lon, 1000.0);
            assert_eq!(a.contains(p.into_either()), b.contains(p.into_either()), "lon {lon}");
        }
    }

    #[test]
    fn far_side_points_are_never_inside() {
        let z = square_zone(0.0, 10_000.0);
        // The antipode of the centroid projects onto the centroid itself.
        assert!(!z.contains((0.0, 180.0, 5000.0).into_either()));
        assert!(!z.contains((-0.01, 179.99, 5000.0).into_either()));
        let d = z.distance_to((0.0, 180.0, 5000.0).into_either());
        assert!(d > 12_000_000.0, "antipode distance {d}");
        // Pacific box and the matching point on the far side of the Earth.
        assert!(!pacific_box().contains((-9.0, 0.0, 1000.0).into_either()));
    }

    #[test]
    fn segment_with_a_far_side_endpoint_is_clipped() {
        let z = square_zone(0.0, 10_000.0);
        // From well east of the square to its antipode: the folded projection of
        // the far end would sit on the centroid, but the near part never enters.
        assert!(!seg(&z, (0.0, 1.0, 5000.0), (0.0, 180.0, 5000.0)));
        assert!(!seg(&z, (0.0, 180.0, 5000.0), (0.0, 1.0, 5000.0)));
        // Both endpoints on the far side.
        assert!(!seg(&z, (0.0, 179.0, 5000.0), (0.0, -179.0, 5000.0)));
        // Starting inside and heading to the far side still hits.
        assert!(seg(&z, (0.0, 0.0, 5000.0), (0.0, 180.0, 5000.0)));
    }

    #[test]
    fn circle_near_the_pole() {
        let z = PolygonZone::from_circle(89.99, 0.0, 50_000.0, 0.0, 5000.0, 25, 1.0).unwrap();
        assert!(z.contains((90.0, 123.0, 1000.0).into_either()));
        assert!(z.contains((89.8, 180.0, 1000.0).into_either()));
        assert!(!z.contains((89.0, 0.0, 1000.0).into_either()));
    }

    // --- accuracy ----------------------------------------------------------------

    #[test]
    fn diagonal_edge_of_a_50_km_diamond_is_on_the_ellipsoid() {
        let c = (36.15, -115.1);
        let d = |b: f64, m: f64| dest(c.0, c.1, b, m);
        let ring = vec![
            d(180.0, 50_000.0),
            d(90.0, 50_000.0),
            d(0.0, 50_000.0),
            d(270.0, 50_000.0),
            d(180.0, 50_000.0),
        ];
        let z = PolygonZone::from_vertices(ring, 0.0, 5_000.0, 1.0).unwrap();
        let mid = 50_000.0 / std::f64::consts::SQRT_2;
        let inside = d(45.0, mid - 20.0);
        let outside = d(45.0, mid + 20.0);
        assert!(z.contains((inside.0, inside.1, 1_000.0).into_either()));
        assert!(!z.contains((outside.0, outside.1, 1_000.0).into_either()));
    }

    #[test]
    fn repr_mentions_vertex_count() {
        let z = square_zone(0.0, 10_000.0);
        let r = z.__repr__();
        assert!(r.contains("PolygonZone"), "{r}");
        assert!(r.contains('4'), "{r}");
    }
}

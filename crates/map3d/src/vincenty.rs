use crate::{constants::wgs84, utils};
use core::fmt;
use std::f64::{consts::PI, INFINITY};

mod vincenty_const {
    pub static INVERSE_SIN_SIGMA_TOL: f64 = 1.0e-10;
    pub static INVERSE_COS2A_TOL: f64 = 1.0e-10;
}

#[derive(Debug, Clone)]
pub struct DomainError {
    pub var: String,
    pub value: f64,
    pub min: f64,
    pub max: f64,
    pub min_inclusive: bool, // If true, min is included in domain as an acceptable value.
    pub max_inclusive: bool, // If true, max is included in domain as an acceptable value.
}

impl fmt::Display for DomainError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let min_inequality: &str = if self.min_inclusive { "[" } else { "(" };
        let max_inequality: &str = if self.max_inclusive { "]" } else { ")" };

        write!(
            f,
            "{} must be in domain {}{}, {}{} but received {}",
            self.var, min_inequality, self.min, self.max, max_inequality, self.value
        )
    }
}

/// Calculates the LLA location that is a fixed range and bearing from a reference LLA. This function uses an iterative
/// solution to determine outputs using the WGS84 ellipsoidal Earth model.
///
/// See reference:
/// https://en.wikipedia.org/wiki/Vincenty%27s_formulae.
///
/// # Arguments
///
/// * `lat_deg` - Latitude reference [[degrees]].
/// * `lon_deg` - Longitude reference [[degrees]].
/// * `range_m` - Range (i.e., distance) from point A to point B [[meters]].
/// * `bearing_deg` - Bearing (i.e., azimuth) from point A to point B relative to true north [[degrees]].
/// * `abs_tol` - Absolute tolerance used for convergence.
/// * `max_iters` - Maximum possible number of iterations before early termination.
///
/// # Returns
///
/// A tuple `(lat_deg, lon_deg)` where:
/// * `lat_deg` - Latitude location [[degrees]].
/// * `lon_deg` - Longitude location [[degrees]].
pub fn vincenty_direct(
    lat_deg: f64,
    lon_deg: f64,
    range_m: f64,
    bearing_deg: f64,
    // TODO: need to make this easier to set defaults
    atol: f64,
    max_iters: u16,
) -> Result<(f64, f64), DomainError> {
    if lat_deg.abs() > 90.0 {
        return Err(DomainError {
            var: "lat_deg".into(),
            value: lat_deg,
            min: -90.0,
            max: 90.0,
            min_inclusive: true,
            max_inclusive: true,
        });
    }

    if range_m < 0.0 {
        return Err(DomainError {
            var: "range_m".into(),
            value: range_m,
            min: 0.0,
            max: INFINITY,
            min_inclusive: true,
            max_inclusive: false,
        });
    }

    if atol <= 0.0 {
        return Err(DomainError {
            var: "atol".into(),
            value: atol,
            min: 0.0,
            max: INFINITY,
            min_inclusive: false,
            max_inclusive: false,
        });
    }

    let lat_rad = lat_deg.to_radians();
    let lon_rad = lon_deg.to_radians();
    let bearing_rad = bearing_deg.to_radians();

    let u1 = ((1.0 - wgs84::EARTH_FLATTENING_FACTOR) * lat_rad.tan()).atan();
    let s1 = u1.tan().atan2(bearing_rad.cos());
    let sina = u1.cos() * bearing_rad.sin();
    let cos2a = 1.0 - sina.powi(2);
    let u2 = cos2a * (wgs84::EARTH_SEMI_MAJOR_AXIS_2 - wgs84::EARTH_SEMI_MINOR_AXIS_2)
        / wgs84::EARTH_SEMI_MINOR_AXIS_2;

    let k1 = ((1.0 + u2).sqrt() - 1.0) / ((1.0 + u2).sqrt() + 1.0);
    let a = (1.0 + 0.25 * k1.powi(2)) / (1.0 - k1);
    let b = k1 * (1.0 - 3. / 8.0 * k1.powi(2));

    // Loop variables that get updated throughout iteration.
    let mut steps: u16 = 0;
    let mut sigma = range_m / (wgs84::EARTH_SEMI_MINOR_AXIS * a);
    let mut twosigm = 2.0 * s1 + sigma;
    let mut cos2sm = twosigm.cos();
    let mut sins = sigma.sin();
    let mut coss = sigma.cos();

    loop {
        let delta_sigma_term_1 = coss * (-1.0 + 2.0 * cos2sm.powi(2));
        let delta_sigma_term_2 = (-3.0 + 4.0 * sins.powi(2)) * (-3.0 + 4.0 * cos2sm.powi(2));
        let delta_sigma_term_3 = b / 6.0 * cos2sm * delta_sigma_term_2;
        let delta_sigma_term_4 = delta_sigma_term_1 - delta_sigma_term_3;
        let delta_sigma = b * sins * (cos2sm + 0.25 * b * delta_sigma_term_4);
        let sigma_old = sigma;

        sigma = range_m / (wgs84::EARTH_SEMI_MINOR_AXIS * a) + delta_sigma;
        twosigm = 2.0 * s1 + sigma;
        cos2sm = twosigm.cos();
        sins = sigma.sin();
        coss = sigma.cos();

        steps += 1;
        if ((sigma_old - sigma).abs() < atol) || (steps >= max_iters) {
            break;
        }
    }

    let phi2_term_1 = u1.sin() * coss + u1.cos() * sins * bearing_rad.cos();
    let phi2_term_2 = (u1.sin() * sins - u1.cos() * coss * bearing_rad.cos()).powi(2);
    let phi2_term_3 =
        (1.0 - wgs84::EARTH_FLATTENING_FACTOR) * (sina.powi(2) + phi2_term_2).sqrt();
    let phi2 = phi2_term_1.atan2(phi2_term_3);

    let lam =
        (sins * bearing_rad.sin()).atan2(u1.cos() * coss - u1.sin() * sins * bearing_rad.cos());
    let c = wgs84::EARTH_FLATTENING_FACTOR / 16.0
        * cos2a
        * (4.0 + wgs84::EARTH_FLATTENING_FACTOR * (4.0 - 3.0 * cos2a));

    let l_term_1 = sigma + c * sins * (cos2sm + c * coss * (-1.0 + 2.0 * cos2sm.powi(2)));
    let l = lam - (1.0 - c) * wgs84::EARTH_FLATTENING_FACTOR * sina * l_term_1;
    let l2 = l + lon_rad;

    Ok((phi2.to_degrees(), l2.to_degrees()))
}

/// Calculates range and bearings between two latitude-longitude points. This function uses an iterative solution to
/// determine outputs using the WGS84 ellipsoidal Earth model.
///
/// See reference:
/// https://en.wikipedia.org/wiki/Vincenty%27s_formulae.
///
/// # Arguments
///
/// * `lat_a_deg` - Latitude point A [[degrees]].
/// * `lon_a_deg` - Longitude point A [[degrees]].
/// * `lat_b_deg` - Latitude point A [[degrees]].
/// * `lon_b_deg` - Longitude point A [[degrees]].
/// * `atol` - Absolute tolerance used for convergence.  WHATS A GOOD DEFAULT? 
/// * `max_iters` - Maximum possible number of iterations before early termination. WHATS A GOOD DEFAULT? 
///
/// # Returns
///
/// A tuple `(range_m, bearing_ab_deg, bearing_ba_deg)` where:
/// * `range_m` - Range (i.e., distance) from point A to point B [[meters]].
/// * `bearing_ab_deg` - Bearing (i.e., azimuth) from point A to point B relative to true north [[degrees]].
/// * `bearing_ba_deg` - Bearing (i.e., azimuth) from point B to point A relative to true north [[degrees]].
pub fn vincenty_inverse(
    lat_a_deg: f64,
    lon_a_deg: f64,
    lat_b_deg: f64,
    lon_b_deg: f64,
    // TODO: need to make this easier to set defaults
    atol: f64,
    max_iters: u16,
) -> Result<(f64, f64, f64), DomainError> {
    if lat_a_deg.abs() > 90.0 {
        return Err(DomainError {
            var: "lat_a_deg".into(),
            value: lat_a_deg,
            min: -90.0,
            max: 90.0,
            min_inclusive: true,
            max_inclusive: true,
        });
    }

    if lat_b_deg.abs() > 90.0 {
        return Err(DomainError {
            var: "lat_b_deg".into(),
            value: lat_b_deg,
            min: -90.0,
            max: 90.0,
            min_inclusive: true,
            max_inclusive: true,
        });
    }

    if atol <= 0.0 {
        return Err(DomainError {
            var: "atol".into(),
            value: atol,
            min: 0.0,
            max: INFINITY,
            min_inclusive: false,
            max_inclusive: false,
        });
    }

    let lat_a_rad = lat_a_deg.to_radians();
    let lon_a_rad = lon_a_deg.to_radians();
    let lat_b_rad = lat_b_deg.to_radians();
    let lon_b_rad = lon_b_deg.to_radians();

    let u1 = ((1.0 - wgs84::EARTH_FLATTENING_FACTOR) * lat_a_rad.tan()).atan();
    let u1_sin = u1.sin();
    let u1_cos = u1.cos();

    let u2 = ((1.0 - wgs84::EARTH_FLATTENING_FACTOR) * lat_b_rad.tan()).atan();
    let u2_sin = u2.sin();
    let u2_cos = u2.cos();

    let l = lon_b_rad - lon_a_rad;

    // Early termination if the provided LLA points are coincident.
    if ((lat_a_rad - lat_b_rad).abs() < vincenty_const::INVERSE_SIN_SIGMA_TOL)
        && (l.abs() < vincenty_const::INVERSE_SIN_SIGMA_TOL)
    {
        return Ok((0.0, 0.0, 0.0));
    }

    // Loop variables that get updated throughout iteration.
    let mut steps: u16 = 0;
    let mut lam = l;
    let mut cos2sm;
    let mut cos2a;
    let mut sigma;
    let mut sin_sigma;
    let mut cos_sigma;

    loop {
        let sin_sigma_term_1 = (u2_cos * lam.sin()).powi(2);
        let sin_sigma_term_2 = (u1_cos * u2_sin - u1_sin * u2_cos * lam.cos()).powi(2);
        sin_sigma = (sin_sigma_term_1 + sin_sigma_term_2).sqrt();

        if sin_sigma < vincenty_const::INVERSE_SIN_SIGMA_TOL {
            if (lat_a_rad - lat_b_rad).abs() < vincenty_const::INVERSE_SIN_SIGMA_TOL {
                // Handles when points that share common latitudes converge to be coincident during iteration.
                return Ok((0.0, 0.0, 0.0));
            } else {
                // Else the points are anti-podal and a small perturbation is added to avoid divide by zero.
                sin_sigma = vincenty_const::INVERSE_SIN_SIGMA_TOL;
            }
        }

        cos_sigma = u1_sin * u2_sin + u1_cos * u2_cos * lam.cos();
        sigma = sin_sigma.atan2(cos_sigma);
        let sina = u1_cos * u2_cos * lam.sin() / sigma.sin();

        // Handle case where both points are on the equator, yielding a divide by zero error.
        cos2a = 1.0 - sina.powi(2);
        if cos2a.abs() < vincenty_const::INVERSE_COS2A_TOL {
            cos2sm = 0.0;
        } else {
            cos2sm = cos_sigma - 2.0 * u1_sin * u2_sin / cos2a;
        }

        let c = wgs84::EARTH_FLATTENING_FACTOR / 16.0
            * cos2a
            * (4.0 + wgs84::EARTH_FLATTENING_FACTOR * (4.0 - 3.0 * cos2a));
        let lam_old = lam;
        let lam_end_term = cos2sm + c * cos_sigma * (-1.0 + 2.0 * cos2sm.powi(2));
        lam = l
            + (1.0 - c)
                * wgs84::EARTH_FLATTENING_FACTOR
                * sina
                * (sigma + c * sin_sigma * lam_end_term);
        steps += 1;
        if ((lam_old - lam).abs() < atol) || (steps > max_iters) {
            break;
        }
    }

    let g2 = cos2a * (wgs84::EARTH_SEMI_MAJOR_AXIS_2 - wgs84::EARTH_SEMI_MINOR_AXIS_2)
        / wgs84::EARTH_SEMI_MINOR_AXIS_2;
    let k1 = ((1.0 + g2).sqrt() - 1.0) / ((1.0 + g2).sqrt() + 1.0);
    let a = (1.0 + 0.25 * k1.powi(2)) / (1.0 - k1);
    let b = k1 * (1.0 - 3.0 / 8.0 * k1.powi(2));

    let delta_sigma_term_1 = cos_sigma * (-1.0 + 2.0 * cos2sm.powi(2));
    let delta_sigma_term_2 = (-3.0 + 4.0 * sin_sigma.powi(2)) * (-3.0 + 4.0 * cos2sm.powi(2));
    let delta_sigma_term_3 = delta_sigma_term_1 - b / 6.0 * cos2sm * delta_sigma_term_2;
    let delta_sigma = b * sin_sigma * (cos2sm + 0.25 * b * delta_sigma_term_3);

    // For clarity, pi is added to the bearing from B to A so that the angle return is the azimuth from B to A, rather
    // than the forward azimuth from A's perspective. However, to keep both angles in the same domain (+/- pi), the
    // modified angle is wrapped.
    let range_m = wgs84::EARTH_SEMI_MINOR_AXIS * a * (sigma - delta_sigma);
    let bearing_ab_rad = (u2_cos * lam.sin()).atan2(u1_cos * u2_sin - u1_sin * u2_cos * lam.cos());
    let bearing_ba_rad = utils::wrap_to_pi(
        (u1_cos * lam.sin()).atan2(-u1_sin * u2_cos + u1_cos * u2_sin * lam.cos()) + PI,
    );

    Ok((
        range_m,
        bearing_ab_rad.to_degrees(),
        bearing_ba_rad.to_degrees(),
    ))
}

#[cfg(test)]
mod test_vincenty {
    use super::*;
    use approx::relative_eq;
    use rstest::*;

    #[rstest]
    #[case((45.0, 120.0, 0.0, 0.0), (45.0, 120.0))] // Edge case (same location)
    #[case((45.0, 120.0, 3000.0, 30.0), (45.02337670419947, 120.0190319660863))] // Sample case 1
    #[case((-10.0, -80.0, 840000.0, -113.0), (-12.884359520148694, -87.12194168465778))] // Sample case 2
    #[case((-60.0, 24.0, 12000000.0, -45.0), (37.34976350706342, -33.49808323545486))] // Sample case 3
    fn test_vincenty_direct(#[case] args: (f64, f64, f64, f64), #[case] truth: (f64, f64)) {
        let result = vincenty_direct(args.0, args.1, args.2, args.3, 1.0E-13, 2000);
        assert!(result.is_ok());

        let test = result.unwrap();
        assert!(relative_eq!(
            test.0,
            truth.0,
            max_relative = 1e-8,
            epsilon = 1e-9
        ));
        assert!(relative_eq!(
            test.1,
            truth.1,
            max_relative = 1e-8,
            epsilon = 1e-9
        ));
    }

    #[rstest]
    #[case((45.0, 120.0, 45.0, 120.0), (0.0, 0.0, 0.0))] // Edge case | Same location
    #[case((90.0, 0.0, -90.0, 0.0), (20003931.457984865, 180.0, 0.0))] // Edge case | Anti-podal
    #[case((0.0, -10.0, 0.0, 10.0), (2226389.8158654678, 90.0, -90.0))] // Edge case | Both on equator
    #[case((-60.0, 24.0, -64.0, 26.0), (457876.09259014280, 167.65057682653136, -14.116435240448425))] // Sample case 1
    #[case((18.0, -175.0, -30.0, 150.0), (6503644.0543462737, -144.26832642124467, 39.866234335863595))] // Sample case 2
    fn test_vincenty_inverse(#[case] args: (f64, f64, f64, f64), #[case] truth: (f64, f64, f64)) {
        let result = vincenty_inverse(args.0, args.1, args.2, args.3, 1.0E-13, 2000);
        assert!(result.is_ok());

        let test = result.unwrap();
        assert!(relative_eq!(
            test.0,
            truth.0,
            max_relative = 1e-4,
            epsilon = 1e-5
        ));
        assert!(relative_eq!(
            test.1,
            truth.1,
            max_relative = 1e-10,
            epsilon = 1e-11
        ));
        assert!(relative_eq!(
            test.2,
            truth.2,
            max_relative = 1e-10,
            epsilon = 1e-11
        ));
    }
}

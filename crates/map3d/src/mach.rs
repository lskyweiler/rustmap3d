use std::fmt;

mod atm_consts {
    pub static ALTITUDES: [f64; 44] = [
        0.000e00, 2.000e03, 4.000e03, 6.000e03, 8.000e03, 1.000e04, 1.200e04, 1.400e04, 1.600e04,
        1.800e04, 2.000e04, 2.200e04, 2.400e04, 2.600e04, 2.800e04, 3.000e04, 3.200e04, 3.400e04,
        3.600e04, 3.800e04, 4.000e04, 4.200e04, 4.400e04, 4.600e04, 4.800e04, 5.000e04, 5.200e04,
        5.400e04, 5.600e04, 5.800e04, 6.000e04, 6.200e04, 6.400e04, 6.600e04, 6.800e04, 7.000e04,
        7.200e04, 7.400e04, 7.600e04, 7.800e04, 8.000e04, 8.200e04, 8.400e04, 8.600e04,
    ];
    pub static SPEED_OF_SOUNDS: [f64; 44] = [
        3.403e02, 3.325e02, 3.246e02, 3.165e02, 3.081e02, 2.995e02, 2.951e02, 2.951e02, 2.951e02,
        2.951e02, 2.951e02, 2.964e02, 2.977e02, 2.991e02, 3.004e02, 3.017e02, 3.030e02, 3.065e02,
        3.101e02, 3.137e02, 3.172e02, 3.207e02, 3.241e02, 3.275e02, 3.298e02, 3.298e02, 3.288e02,
        3.254e02, 3.220e02, 3.186e02, 3.151e02, 3.115e02, 3.080e02, 3.044e02, 3.007e02, 2.971e02,
        2.934e02, 2.907e02, 2.880e02, 2.853e02, 2.825e02, 2.797e02, 2.769e02, 2.741e02,
    ];
}

#[derive(Debug, Clone)]
pub struct OutOfBoundsAtmosphericLookupError {
    pub alt_m: f64,
}

impl fmt::Display for OutOfBoundsAtmosphericLookupError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Altitude must be between [0, 86000] meters. Got {}",
            self.alt_m,
        )
    }
}

/// Compute the mach number at a given altitude
/// https://en.wikipedia.org/wiki/Mach_number
///
/// This is an approximation that uses a speed of sound lookup table for altitudes [0m, 86000m]
/// Out of bounds altitudes will be clamped to the ends of the table
///
/// # Arguments
///
/// - `speed_mps` (`f64`) - Speed in meters/second
/// - `alt_m` (`f64`) - MSL altitude in meters
///
/// # Returns
///
/// - `f64` - Mach number as a float
///
pub fn mach(speed_mps: f64, alt_m: f64) -> Result<f64, OutOfBoundsAtmosphericLookupError> {
    let sos = speed_of_sound(alt_m)?;
    Ok(speed_mps / sos)
}

/// Get the speed of sound at a given altitude
///
/// # Arguments
///
/// - `alt_m` (`f64`) - MSL altitude in meters
///
/// # Returns
///
/// - `f64` - speed of sound in meters
///
pub fn speed_of_sound(alt_m: f64) -> Result<f64, OutOfBoundsAtmosphericLookupError> {
    if alt_m < atm_consts::ALTITUDES[0] || alt_m > *atm_consts::ALTITUDES.last().unwrap() {
        return Err(OutOfBoundsAtmosphericLookupError { alt_m: alt_m });
    }

    let partition = atm_consts::ALTITUDES.partition_point(|a: &f64| a <= &alt_m);
    let idx = (partition - 1).max(0).min(atm_consts::ALTITUDES.len() - 1);
    Ok(atm_consts::SPEED_OF_SOUNDS[idx])
}

#[cfg(test)]
mod test_mach {
    use super::*;

    #[test]
    fn test_mach_in_bounds() {
        let actual = mach(343., 0.).unwrap();
        almost::equal_with(actual, 1.0079341757272995, 1e-6);
    }
    #[test]
    fn test_mach_gt_bounds() {
        let actual = mach(343., 100e4);
        assert!(actual.is_err());
    }
    #[test]
    fn test_mach_lt_bounds() {
        let actual = mach(343., -100.);
        assert!(actual.is_err());
    }
}

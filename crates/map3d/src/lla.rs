use crate::{
    traits::{IntoDVec3, IntoLatLonTriple, IntoLatLonTuple},
    util,
};
use almost;
use core::fmt;

/// WGS84 Earth constants
pub mod wgs84_const {
    pub static EARTH_SEMI_MAJOR_AXIS: f64 = 6378137.0; // Equatorial radius.
    pub static EARTH_SEMI_MAJOR_AXIS_2: f64 = EARTH_SEMI_MAJOR_AXIS * EARTH_SEMI_MAJOR_AXIS;
    pub static EARTH_SEMI_MINOR_AXIS: f64 = 6356752.314245; // Polar radius.
    pub static EARTH_SEMI_MINOR_AXIS_2: f64 = EARTH_SEMI_MINOR_AXIS * EARTH_SEMI_MINOR_AXIS;
    pub static EARTH_FLATTENING_FACTOR: f64 = 0.003352810664740;
    pub static EARTH_ANGULAR_VEL_RADPS: f64 = 7.292115900000000e-05;
    pub static EARTH_E: f64 = 521854.0084255785;
    pub static EARTH_E_2: f64 = EARTH_E * EARTH_E;

    pub static ECEF2LLA_A: f64 = EARTH_SEMI_MAJOR_AXIS;
    pub static ECEF2LLA_B: f64 = ECEF2LLA_A * (1.0 - EARTH_FLATTENING_FACTOR);
    pub static ECEF2LLA_A2: f64 = ECEF2LLA_A * ECEF2LLA_A;
    pub static ECEF2LLA_B2: f64 = ECEF2LLA_B * ECEF2LLA_B;
    pub static ECEF2LLA_E2: f64 = 1.0 - ECEF2LLA_B2 / ECEF2LLA_A2;
    pub static ECEF2LLA_EP2: f64 = ECEF2LLA_A2 / ECEF2LLA_B2 - 1.0;
    pub static ECEF2LLA_EP22: f64 = ECEF2LLA_EP2 * ECEF2LLA_EP2;
}

/// Converts ECEF to LLA using Ferrari's solution:
/// https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#Ferrari's_solution
///
/// # Arguments
///
/// * `ecef` - Vector represented in ECEF coordinates [[meters]].
///
/// # Returns
///
/// * `lla` - Vector represented in LLA coordinates [[degrees-degrees-meters]].
pub fn ecef2lla_ferarri(ecef: impl IntoDVec3) -> glam::DVec3 {
    let ecef = ecef.into_dvec3();

    let z_ecef_squared: f64 = f64::powf(ecef.z, 2.);
    let range_squared: f64 = f64::powf(ecef.x, 2.) + f64::powf(ecef.y, 2.);
    let range_: f64 = f64::sqrt(range_squared);
    let f_term: f64 = 54. * f64::powf(wgs84_const::EARTH_SEMI_MINOR_AXIS, 2.) * z_ecef_squared;
    let g_term: f64 = range_squared + (1. - wgs84_const::ECEF2LLA_E2) * z_ecef_squared
        - wgs84_const::ECEF2LLA_E2
            * (f64::powf(wgs84_const::EARTH_SEMI_MAJOR_AXIS, 2.)
                - f64::powf(wgs84_const::EARTH_SEMI_MINOR_AXIS, 2.));

    let c_term: f64 = wgs84_const::ECEF2LLA_EP22 * f_term * range_squared / f64::powf(g_term, 3.);
    let c_term_sqrt_mod: f64 = f64::sqrt(f64::powf(c_term, 2.) + 2. * c_term);
    let s_term: f64 = f64::powf(1. + c_term + c_term_sqrt_mod, 1. / 3.);
    let p_term: f64 =
        f_term / (3. * f64::powf(s_term + 1. / s_term + 1., 2.) * f64::powf(g_term, 2.));
    let q_term: f64 = f64::sqrt(1. + 2. * wgs84_const::ECEF2LLA_EP22 * p_term);
    let sqrt_mod2: f64 = f64::sqrt(
        0.5 * f64::powf(wgs84_const::EARTH_SEMI_MAJOR_AXIS, 2.) * (1. + 1. / q_term)
            - p_term * (1. - wgs84_const::ECEF2LLA_E2) * z_ecef_squared / (q_term * (1. + q_term))
            - 0.5 * p_term * range_squared,
    );
    let r0_term: f64 = -(p_term * wgs84_const::ECEF2LLA_E2 * range_) / (1. + q_term) + sqrt_mod2;
    let uv_subterm: f64 = f64::powf(range_ - wgs84_const::ECEF2LLA_E2 * r0_term, 2.);
    let u_term: f64 = f64::sqrt(uv_subterm + z_ecef_squared);
    let v_term: f64 = f64::sqrt(uv_subterm + (1. - wgs84_const::ECEF2LLA_E2) * z_ecef_squared);
    let z0_term: f64 = f64::powf(wgs84_const::EARTH_SEMI_MINOR_AXIS, 2.) * ecef.z
        / (wgs84_const::EARTH_SEMI_MAJOR_AXIS * v_term);
    let lat_rad: f64 = if range_ != 0. {
        f64::atan2(ecef.z + wgs84_const::ECEF2LLA_EP2 * z0_term, range_)
    } else {
        0.0
    };
    let lon_rad: f64 = f64::atan2(ecef.y, ecef.x);
    let alt: f64 = u_term
        * (1.
            - f64::powf(wgs84_const::EARTH_SEMI_MINOR_AXIS, 2.)
                / (wgs84_const::EARTH_SEMI_MAJOR_AXIS * v_term));

    let lat: f64 = f64::to_degrees(lat_rad);
    let lon: f64 = f64::to_degrees(lon_rad);
    return glam::DVec3::new(lat, lon, alt);
}

/// Converts ECEF to LLA using formulation from `pymap3d`:
/// https://github.com/geospace-code/pymap3d/blob/b4dcee4cd7a43641666cef840c97dd73d4e5ed61/src/pymap3d/ecef.py#L87
///
/// # Arguments
///
/// * `ecef` - Vector represented in ECEF coordinates [[meters]].
///
/// # Returns
///
/// * `lla` - Vector represented in LLA coordinates [[degrees-degrees-meters]].
pub fn ecef2lla_map3d(ecef: impl IntoDVec3) -> glam::DVec3 {
    let ecef = ecef.into_dvec3();

    let r = f64::sqrt(ecef.x * ecef.x + ecef.y * ecef.y + ecef.z * ecef.z);
    let r2 = r * r;
    let u = f64::sqrt(
        0.5 * (r2 - wgs84_const::EARTH_E_2)
            + 0.5
                * f64::hypot(
                    r2 - wgs84_const::EARTH_E_2,
                    2.0 * wgs84_const::EARTH_E * ecef.z,
                ),
    );
    let hxy = f64::hypot(ecef.x, ecef.y);
    let hue = f64::hypot(u, wgs84_const::EARTH_E);

    // rust signum returns 1 for 0.0, but we need 0.0 here for sign
    let sign = if ecef.z != 0.0 {
        f64::signum(ecef.z)
    } else {
        0.0
    };
    let mut beta = std::f64::consts::FRAC_PI_2 * sign;

    if !almost::zero(u) && !almost::zero(hxy) {
        beta = f64::atan(hue / u * ecef.z / hxy);
        beta += ((wgs84_const::EARTH_SEMI_MINOR_AXIS * u
            - wgs84_const::EARTH_SEMI_MAJOR_AXIS * hue
            + wgs84_const::EARTH_E_2)
            * f64::sin(beta))
            / (wgs84_const::EARTH_SEMI_MAJOR_AXIS * hue * 1. / f64::cos(beta)
                - wgs84_const::EARTH_E_2 * f64::cos(beta))
    }

    let lat = f64::atan(
        wgs84_const::EARTH_SEMI_MAJOR_AXIS / wgs84_const::EARTH_SEMI_MINOR_AXIS * f64::tan(beta),
    );
    let lon = f64::atan2(ecef.y, ecef.x);

    let mut alt = f64::hypot(
        ecef.z - wgs84_const::EARTH_SEMI_MINOR_AXIS * f64::sin(beta),
        hxy - wgs84_const::EARTH_SEMI_MAJOR_AXIS * f64::cos(beta),
    );
    let inside = ecef.x * ecef.x / wgs84_const::EARTH_SEMI_MAJOR_AXIS_2
        + ecef.y * ecef.y / wgs84_const::EARTH_SEMI_MAJOR_AXIS_2
        + ecef.z * ecef.z / wgs84_const::EARTH_SEMI_MINOR_AXIS_2
        < 1.;
    alt *= if inside { -1. } else { 1.0 };
    return glam::DVec3::new(f64::to_degrees(lat), f64::to_degrees(lon), alt);
}

/// Converts ECEF to LLA.
///
/// # Arguments
///
/// * `ecef` - Vector represented in ECEF coordinates [[meters]].
///
/// # Returns
///
/// * `lla` - Vector represented in LLA coordinates [[degrees-degrees-meters]].
pub fn ecef2lla(ecef: impl IntoDVec3) -> glam::DVec3 {
    return ecef2lla_ferarri(ecef);
}

/// Converts LLA to ECEF.
///
/// # Arguments
///
/// * `lla` - Vector represented in LLA coordinates [[degrees-degrees-meters]].
///
/// # Returns
///
/// * `ecef` - Vector represented in ECEF coordinates [[meters]].
pub fn lla2ecef(lla: impl IntoLatLonTriple) -> glam::DVec3 {
    let lla = lla.into_lat_lon_triple();

    let lat = f64::to_radians(lla.0);
    let lon = f64::to_radians(lla.1);
    let alt = lla.2;

    let alt_correction = wgs84_const::EARTH_SEMI_MAJOR_AXIS_2
        / f64::hypot(
            wgs84_const::EARTH_SEMI_MAJOR_AXIS * f64::cos(lat),
            wgs84_const::EARTH_SEMI_MINOR_AXIS * f64::sin(lat),
        );
    let x = (alt_correction + lla.2) * f64::cos(lat) * f64::cos(lon);
    let y = (alt_correction + lla.2) * f64::cos(lat) * f64::sin(lon);
    let z = (alt_correction
        * f64::powf(
            wgs84_const::EARTH_SEMI_MINOR_AXIS / wgs84_const::EARTH_SEMI_MAJOR_AXIS,
            2.0,
        )
        + alt)
        * f64::sin(lat);
    return glam::DVec3::new(x, y, z);
}

/// Generates a uniform random ECEF point on the surface of a spherical Earth.
///
/// # Returns
///
/// * `ecef` - Random ECEF location [[meters]].
pub fn rand_ecef() -> glam::DVec3 {
    return util::rand_point_on_sphere(wgs84_const::EARTH_SEMI_MAJOR_AXIS);
}

/// Generates a uniform random LLA point. Altitude is generated in the domain [[0.0, 10000.0]].
///
/// # Returns
///
/// * `lla` - Random LLA location [[degrees-degrees-meters]].
pub fn rand_lla() -> glam::DVec3 {
    return glam::DVec3::new(
        util::lerp(-90.0, 90.0, rand::random()),
        util::lerp(-180.0, 180.0, rand::random()),
        util::lerp(0.0, 10000.0, rand::random()),
    );
}
/// Generates a uniform random LLA point in a given lat/lon/alt bounding box
///
/// # Returns
///
/// * `lla` - Random LLA location [[degrees-degrees-meters]].
pub fn rand_lla_in_range(
    min_lat_d: f64,
    max_lat_d: f64,
    min_lon_d: f64,
    max_lon_d: f64,
    min_alt_m: f64,
    max_alt_m: f64,
) -> glam::DVec3 {
    return glam::DVec3::new(
        util::lerp(min_lat_d, max_lat_d, rand::random()),
        util::lerp(min_lon_d, max_lon_d, rand::random()),
        util::lerp(min_alt_m, max_alt_m, rand::random()),
    );
}

#[derive(Debug, Clone)]
pub struct IllFormedDMSError {
    pub bad_dms: String,
}

impl fmt::Display for IllFormedDMSError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Got ill-formed degrees minutes seconds: {}. Should be format DD:MM:SS.SSSC where C is a cardinal direction [N, S, E, W]", 
            self.bad_dms
        )
    }
}

/// Converts degrees-minutes-seconds (DMS) to decimal degrees (DD).
///
/// # Arguments
///
/// * `dms` - Format "{Degrees}:{Minutes}:{Seconds}.{DecSeconds}{Direction}" where Direction is one of N, E, S, or W.
///
/// # Returns
///
/// * `dd` - Decimal degrees [degrees].
///
/// # Examples
/// ```
/// use map3d::dms2dd;
/// use approx::relative_eq;
///
/// let dd = dms2dd("25:22:44.738N").unwrap();
/// assert!(relative_eq!(dd, 25.37909389, max_relative = 1e-8, epsilon = 1e-9));
/// ```
pub fn dms2dd(dms: &str) -> Result<f64, IllFormedDMSError> {
    let parts = dms.split(":").collect::<Vec<&str>>();

    if parts.len() == 3 {
        let sdeg = parts[0];
        let smin = parts[1];
        let sec_card = parts[2];
        if let Some(cardinal) = sec_card.chars().last() {
            let lower_card = &cardinal.to_lowercase().to_string()[..];
            let ssec = &sec_card[..sec_card.len() - 1];
            if let (Ok(deg), Ok(min), Ok(sec)) = (
                sdeg.parse::<f64>(),
                smin.parse::<f64>(),
                ssec.parse::<f64>(),
            ) {
                if vec!["n", "s", "e", "w"].contains(&lower_card) {
                    let mut dd = deg + min / 60. + sec / 3600.;
                    if lower_card == "s" || lower_card == "w" {
                        dd *= -1.;
                    }
                    return Ok(dd);
                }
            }
        }
    }
    return Err(IllFormedDMSError {
        bad_dms: dms.to_string(),
    });
}

/// Converts decimal degrees (DD) to degrees-minutes-seconds (DMS).
///
/// # Arguments
///
/// * `dd` - Decimal degrees [degrees].
/// * `is_lat` - Flag to denote if this decimal value describes a latitude [bool]
///
/// # Returns
///
/// * `dms` - Format "{Degrees}:{Minutes}:{Seconds}.{DecSeconds}{Direction}" where Direction is one of N, E, S, or W.
///
/// # Examples
/// ```
/// use map3d::dd2dms;
/// use approx::relative_eq;
///
/// let dms = dd2dms(25.37909389, true);
/// assert_eq!(dms, "25:22:44.738N");
/// ```
pub fn dd2dms(dd: f64, is_lat: bool) -> String {
    let dir: &str;

    if is_lat {
        if dd > 0. {
            dir = "N";
        } else {
            dir = "S";
        }
    } else {
        if dd > 0. {
            dir = "E";
        } else {
            dir = "W";
        }
    }

    let deg = f64::abs(dd) * 3600.;
    let (mnt, sec) = (deg / 60., deg % 60.);
    let (deg, min) = (mnt / 60., mnt % 60.);
    return format!("{:.0}:{:.0}:{:.3}{}", deg.floor(), min.floor(), sec, dir);
}

/// Convenience function to convert lat/lon to a tuple of (lat dms, lon dms)
///
/// # Arguments
///
/// * `ll_deg` - Reference latitude-longitude [[degrees-degrees]].
///
/// # Returns
///
/// * `(lat dms, lon dms)` - Tuple of lat/lon as degrees:minutes:seconds [Tuple[String, String]]
/// ```
pub fn ll2dms(ll_deg: impl IntoLatLonTuple) -> (String, String) {
    let ll_deg = ll_deg.into_lat_lon_tuple();
    return (dd2dms(ll_deg.0, true), dd2dms(ll_deg.0, false));
}

#[cfg(test)]
mod test_lla {
    use super::*;
    use crate::util::assert_vecs_close;
    use rstest::*;

    #[fixture]
    fn ecef_fixture() -> (Vec<glam::DVec3>, Vec<glam::DVec3>) {
        let ecefs: Vec<glam::DVec3> = vec![
            glam::DVec3::new(-4395937.137069274, -5292832.080918098, 269071.20488286763),
            glam::DVec3::new(-17425.064377540722, -2006865.517897409, 7930762.472554953),
            glam::DVec3::new(-3895837.31483683, 6644097.950743062, -1824545.3930313005),
            glam::DVec3::new(-6409133.120866153, 4558772.702384297, -1947061.2076449227),
            glam::DVec3::new(851485.3223891761, -2568595.6166770314, -8943557.577897426),
            glam::DVec3::new(5691809.8303289935, -7030732.321824082, -8330125.529865682),
            glam::DVec3::new(2516433.37963281, -1241849.192556627, -6760285.815142313),
            glam::DVec3::new(-3482763.8631693274, 6769587.309460899, -944640.6623275336),
            glam::DVec3::new(-370523.61786744185, 6024008.984975778, 8509531.670449838),
            glam::DVec3::new(5313476.383658681, -9825896.959403213, -1741391.5667025768),
            glam::DVec3::new(-8908541.706026632, -7037043.26505645, 6552883.707257189),
            glam::DVec3::new(-617141.4009794481, 2060754.5193012804, 8394970.341717768),
            glam::DVec3::new(8538245.25323937, 4747903.057412151, -9958019.246258922),
            glam::DVec3::new(-2108304.0336288405, -1654639.056214511, -9162254.95404531),
            glam::DVec3::new(-9223445.909886952, -400765.09915581346, -9900123.17700839),
            glam::DVec3::new(-7688902.383860592, -4471753.673588827, -5826850.3124411125),
            glam::DVec3::new(-3018693.051038252, -5388736.217988148, -5258460.323543303),
            glam::DVec3::new(-9288375.018009178, -2127487.6902046744, 6460020.61877924),
            glam::DVec3::new(-144971.8057209216, 114606.55915352888, 7971798.167930942),
            glam::DVec3::new(6828101.960543122, 693885.4050173741, -1196749.1985590179),
            glam::DVec3::new(-8578157.642460234, -1842531.7364011942, 6845094.434824787),
            glam::DVec3::new(5787354.780059811, 1210116.2836728748, -4726770.355619501),
            glam::DVec3::new(4193285.538259528, -1970166.3550874572, 7412301.116472796),
            glam::DVec3::new(3890132.340585826, 1161874.8197520673, -7909529.461251948),
            glam::DVec3::new(-7403258.820000752, -2903792.6257274374, -8826380.396521661),
            glam::DVec3::new(1857337.7916966896, 368102.4709657077, -6155560.052247517),
            glam::DVec3::new(-7588845.593718505, -8209442.364279032, 8265779.358689017),
            glam::DVec3::new(-5108159.12198603, -8476495.093040336, 6352145.996764505),
            glam::DVec3::new(-186001.42373996228, -7630898.02119923, -9721624.592627238),
            glam::DVec3::new(2369638.1324996054, 4073593.1322049536, 452913.52990143374),
            glam::DVec3::new(449824.9360387679, 4166398.4307631515, 2736272.9522982505),
            glam::DVec3::new(9965957.90734066, 2221724.6208532117, -6853904.619168903),
            glam::DVec3::new(-4478176.754602823, 1036869.5552951898, -9423720.838865533),
            glam::DVec3::new(2097024.4875351437, -9881162.142519716, 4949580.930534031),
            glam::DVec3::new(201204.94145997614, 9498512.914456967, 3515229.3174460693),
            glam::DVec3::new(-2060584.097762437, 3861553.560423618, -7614373.714063268),
            glam::DVec3::new(1757425.2753162049, 3398544.52313062, 8492215.282477114),
            glam::DVec3::new(-9047342.859610673, -8441792.192733321, -4469448.824678039),
            glam::DVec3::new(-487921.69639001414, -6030301.279856028, -9260258.22696507),
            glam::DVec3::new(-2744053.779757822, 2376587.0533640455, -3680945.978733376),
            glam::DVec3::new(-1815727.9597172216, 1953269.055585172, 6538261.825529313),
            glam::DVec3::new(8720682.736849941, -3489723.916849705, -5774441.696210405),
            glam::DVec3::new(-2933009.790774938, -5440069.166919639, -8623652.84907917),
            glam::DVec3::new(549449.791821098, -2920680.440199934, -3534971.5976333166),
            glam::DVec3::new(-5140412.492418254, -7429509.788924876, -1444380.0341157485),
            glam::DVec3::new(9626007.606757686, 8719399.041604526, 1697464.7512210696),
            glam::DVec3::new(-9127014.835284937, -6298505.842698975, -773826.4704206288),
            glam::DVec3::new(1842921.888856599, 8121796.140805811, 6227066.092612989),
            glam::DVec3::new(5138399.743888708, 9322686.007429656, -2555719.5912182704),
            glam::DVec3::new(9598896.75147951, 5044493.199291278, -6248509.982967714),
        ];
        let llas: Vec<glam::DVec3> = vec![
            glam::DVec3::new(2.253516806471046, -129.71116955184112, 507441.10318602197),
            glam::DVec3::new(75.87000157402547, -90.49747108091655, 1822723.22233205),
            glam::DVec3::new(-13.396850991057667, 120.38568018022004, 1538213.4642196773),
            glam::DVec3::new(-13.975248661411722, 144.5760168892664, 1725598.1936111697),
            glam::DVec3::new(-73.23828849316851, -71.65972404561161, 2985437.311158756),
            glam::DVec3::new(-42.74052199178427, -51.00770324601096, 5928794.414992734),
            glam::DVec3::new(-67.57498967210243, -26.26614362057283, 959684.6950130499),
            glam::DVec3::new(-7.112487646164081, 117.22455673918091, 1293518.4109493706),
            glam::DVec3::new(54.76454334396985, 93.51970396395149, 4068652.2450541486),
            glam::DVec3::new(-8.893647137325992, -61.59714074842926, 4927844.7237863885),
            glam::DVec3::new(30.07510912049895, -141.69405513524464, 6735317.147520137),
            glam::DVec3::new(75.69522425715692, 106.67156561450872, 2308138.501369687),
            glam::DVec3::new(-45.63516031581972, 29.077333155761565, 7582905.476120061),
            glam::DVec3::new(-73.76417124368244, -141.8744698912028, 3187753.7972916476),
            glam::DVec3::new(-47.08986703197784, -177.51202373059226, 7170110.832990964),
            glam::DVec3::new(-33.334219053050205, -149.81834070560083, 4261638.047698805),
            glam::DVec3::new(-40.55844825566602, -119.256926093939, 1742735.252764878),
            glam::DVec3::new(34.23382312320651, -167.09902807775964, 5140859.543690856),
            glam::DVec3::new(88.67910855596926, 141.67208580697599, 1617176.124676821),
            glam::DVec3::new(-9.951000598269978, 5.80259213124441, 589322.5756239324),
            glam::DVec3::new(38.067270677905, -167.87743403521839, 4758084.056677728),
            glam::DVec3::new(-38.79866773869741, 11.810196086116186, 1199912.009335559),
            glam::DVec3::new(58.11840077026344, -25.165937921189123, 2378386.964989038),
            glam::DVec3::new(-62.94048624737819, 16.62942891748623, 2529453.921751855),
            glam::DVec3::new(-48.084398711074584, -158.58331540347453, 5514132.784860136),
            glam::DVec3::new(-73.00848065406132, 11.210083031167493, 81603.365385446),
            glam::DVec3::new(36.56190250218874, -132.75043328084922, 7532975.424592685),
            glam::DVec3::new(32.78902393868271, -121.07427361795573, 5387958.701666567),
            glam::DVec3::new(-51.95818468723161, -91.39629526176816, 5995326.962900662),
            glam::DVec3::new(5.539216363414069, 59.81313129551833, -1643546.9126202406),
            glam::DVec3::new(33.367417431775394, 83.83793347365611, -1366875.6175801605),
            glam::DVec3::new(-33.963908882404155, 12.56752085132872, 5926171.518468513),
            glam::DVec3::new(-64.09003114845517, 166.96355233242235, 4124167.2847124636),
            glam::DVec3::new(26.190941042810294, -78.01820568227308, 4874709.392305468),
            glam::DVec3::new(20.38330525588493, 88.78649739621953, 3754550.8161831154),
            glam::DVec3::new(-60.228949370259244, 118.08506369211479, 2420676.065792122),
            glam::DVec3::new(65.84506256608404, 62.65601109433141, 2953957.9933096142),
            glam::DVec3::new(-19.918996193014078, -136.9830393254037, 6780862.795777614),
            glam::DVec3::new(-56.943542406173144, -94.62581967163953, 4698278.924516642),
            glam::DVec3::new(-45.63501439769118, 139.10461559970705, -1197409.76205113),
            glam::DVec3::new(67.93118253875696, 132.9100464016069, 701440.670716858),
            glam::DVec3::new(-31.68079101252089, -21.809626299383666, 4653736.780262814),
            glam::DVec3::new(-54.48102155235577, -118.3312980633975, 4245634.217446316),
            glam::DVec3::new(-50.206653953803524, -79.34581589819817, -1747312.1358781399),
            glam::DVec3::new(-9.125140350907303, -124.67901908506032, 2771588.2018704447),
            glam::DVec3::new(7.470140696990517, 42.17081118035693, 6720670.269056683),
            glam::DVec3::new(-4.007031763998265, -145.39064333427916, 4738278.426321056),
            glam::DVec3::new(36.898669089623674, 77.21546306700436, 4028403.093408107),
            glam::DVec3::new(-13.551349707628185, 61.137694236757405, 4570511.41330536),
            glam::DVec3::new(-30.036801286738232, 27.723202807325016, 6142375.721604907),
        ];
        return (ecefs, llas);
    }
    #[fixture]
    fn lla_fixture() -> (Vec<glam::DVec3>, Vec<glam::DVec3>) {
        let llas: Vec<glam::DVec3> = vec![
            glam::DVec3::new(-56.66172812377343, 155.68130312212668, 718875820.2943813),
            glam::DVec3::new(-33.26507252871781, -31.964572932142886, 7065343507.721147),
            glam::DVec3::new(6.371772880304988, -59.23405806377991, 8352922369.965223),
            glam::DVec3::new(-70.74335211522242, 120.07092031064208, 4526779918.14558),
            glam::DVec3::new(-24.461510525146508, -59.62407504106517, 2420545872.95108),
            glam::DVec3::new(-72.3152549895963, 101.64767964829839, 3795852142.154489),
            glam::DVec3::new(50.1705938605771, 105.40775429487171, 7390834021.2357435),
            glam::DVec3::new(-32.30257694254681, -135.9790191808388, 2106745558.3293736),
            glam::DVec3::new(-19.934988367833313, 65.33934397460283, 6927358798.01723),
            glam::DVec3::new(23.702048312802475, 150.7801192499238, 4151592272.73058),
            glam::DVec3::new(-44.139744893078216, -10.394871262469564, 1308050706.656194),
            glam::DVec3::new(80.33315640988386, -167.00766083959087, 4853517330.590896),
            glam::DVec3::new(89.5832033021365, -146.3731250375061, 5541861449.666565),
            glam::DVec3::new(6.002327772952086, 178.1723518113481, 8243112160.578481),
            glam::DVec3::new(-30.240836896769675, 5.48046771739655, 774183075.5164747),
            glam::DVec3::new(-66.50252282873791, -130.9736602675406, 6467951118.810617),
            glam::DVec3::new(32.244715738302816, 134.35132917429144, 8312562383.178297),
            glam::DVec3::new(-40.929722118941775, -166.0099481725839, 6216809556.871332),
            glam::DVec3::new(-3.161999721921518, -121.6362613721991, 1686482297.1376824),
            glam::DVec3::new(70.69015001986463, -18.77422907679727, 4347545178.616857),
            glam::DVec3::new(-77.89686967839751, 76.8590940393218, 275547474.5753206),
            glam::DVec3::new(56.70256197263234, 31.487362633295106, 3351651550.650465),
            glam::DVec3::new(-8.537166774081555, 124.9300397881874, 6964157346.925901),
            glam::DVec3::new(-3.3883037953841466, 159.9876304913834, 5824233554.097087),
            glam::DVec3::new(3.3173719339601604, 19.53837842565602, 7727952079.516271),
            glam::DVec3::new(-29.656043118692313, 50.423126817903324, 3109749957.585276),
            glam::DVec3::new(-3.317182434873658, 115.18183094259633, 6783718307.968782),
            glam::DVec3::new(87.99594613540816, 113.8812711654927, 6548397030.25793),
            glam::DVec3::new(42.155897640035505, -7.385067279532194, 4223681045.2455497),
            glam::DVec3::new(-45.66421911004391, 165.75779873561737, 9231088702.94254),
            glam::DVec3::new(74.98669309517001, -139.02652259218758, 7752650661.051985),
            glam::DVec3::new(0.04388642130096798, 139.12251298740722, 9856972131.89139),
            glam::DVec3::new(18.913840578810465, -152.22766889844678, 8674973608.740688),
            glam::DVec3::new(-70.71679219133352, 168.7938671723137, 2559962939.2985635),
            glam::DVec3::new(-17.840267641554703, 145.15346668909052, 9666183830.211079),
            glam::DVec3::new(7.81829152155791, 178.60319210639625, 2434544733.5319457),
            glam::DVec3::new(-86.60171868991569, -144.4301374069363, 8315174907.876453),
            glam::DVec3::new(-64.583522424254, 117.42569962019991, 2137778660.492624),
            glam::DVec3::new(-53.497097618994985, 66.73409062380492, 6968275498.757018),
            glam::DVec3::new(39.85170162247488, 82.86455367169816, 5083534728.543176),
            glam::DVec3::new(-89.14033256764529, 3.817737674123464, 8488834909.559485),
            glam::DVec3::new(-8.097151555351061, -60.04675219817315, 1794800252.0766892),
            glam::DVec3::new(60.35525557045381, 137.40735372191648, 2643745363.528964),
            glam::DVec3::new(-33.80752224309167, 151.33853754701425, 4511336080.320615),
            glam::DVec3::new(44.49769217327577, 35.65986371715448, 7747124179.421991),
            glam::DVec3::new(78.6881809933453, 153.6339574621485, 1262173554.861432),
            glam::DVec3::new(50.934753774135714, 48.68011259868561, 3940700238.395185),
            glam::DVec3::new(23.139059023558076, -30.9794825232527, 3732066418.3197985),
            glam::DVec3::new(-60.66725009582464, -27.905389704786387, 5654168566.610009),
            glam::DVec3::new(-89.52247206976813, 49.3889940119827, 2084170969.4723942),
        ];
        let ecefs: Vec<glam::DVec3> = vec![
            glam::DVec3::new(-363226337.8378485, 164145694.11941433, -605883225.5202017),
            glam::DVec3::new(5016417802.618961, -3130294546.8914347, -3878912812.734696),
            glam::DVec3::new(4249637013.1107903, -7138476691.558969, 927704883.3233159),
            glam::DVec3::new(-749123383.4568931, 1293818655.7673783, -4279508924.526832),
            glam::DVec3::new(1117071280.1290438, -1905836230.5036347, -1004929049.7808872),
            glam::DVec3::new(-233195557.85231522, 1131259987.718429, -3622523730.6242113),
            glam::DVec3::new(-1258810196.8083477, 4567666948.353717, 5680702139.465385),
            glam::DVec3::new(-1284356369.4257667, -1241197752.6029196, -1129213367.408031),
            glam::DVec3::new(2719703477.4990993, 5923770937.923891, -2364069442.8993587),
            glam::DVec3::new(-3322780364.3305306, 1858552546.759458, 1671407285.5992749),
            glam::DVec3::new(927816843.0914981, -170200408.00971267, -915359858.8829931),
            glam::DVec3::new(-795180643.7592813, -183469932.25196317, 4790867197.061249),
            glam::DVec3::new(-33606494.82719547, -22350830.551816523, 5548071401.239205),
            glam::DVec3::new(-8200090300.39463, 261659157.03025025, 862635420.7610741),
            glam::DVec3::new(671261500.4702597, 64404182.064525165, -393099795.1348588),
            glam::DVec3::new(-1692640892.2722268, -1948969436.132962, -5937439805.014645),
            glam::DVec3::new(-4918541001.068794, 5031191125.999227, 4438438970.340508),
            glam::DVec3::new(-4562247156.197555, -1136654630.154329, -4076992529.3376484),
            glam::DVec3::new(-886595545.8633595, -1439098934.8425846, -93374639.72836758),
            glam::DVec3::new(1363144037.4345145, -463368006.7734003, 4108967125.721511),
            glam::DVec3::new(13439868.383440318, 57568179.5971645, -275637077.97071016),
            glam::DVec3::new(1572069597.6960115, 962888601.5025085, 2806725105.8847656),
            glam::DVec3::new(-3946937207.6022162, 5651485039.359078, -1034776257.2819849),
            glam::DVec3::new(-5468975219.234577, 1991881397.6015773, -344601758.5886673),
            glam::DVec3::new(7276755652.2802105, 2582321037.9901214, 447557846.6049511),
            glam::DVec3::new(1725273328.2316494, 2087211973.2923207, -1541817056.8399067),
            glam::DVec3::new(-2884293340.9078116, 6134483975.098778, -392895716.5994358),
            glam::DVec3::new(-92799096.46533564, 209598007.20691597, 6550744575.743122),
            glam::DVec3::new(3109827878.675527, -403071979.06005263, 2838982707.422298),
            glam::DVec3::new(-6257302596.723574, 1588245806.4929829, -6607134865.92882),
            glam::DVec3::new(-1517523803.7477174, -1317930511.8018465, 7494157658.802485),
            glam::DVec3::new(-7457782245.210336, 6455006230.264203, 7554923.466331962),
            glam::DVec3::new(-7266578347.596589, -3826748677.7182827, 2814011879.0678635),
            glam::DVec3::new(-831350760.9757568, 164704360.3558542, -2422341250.456174),
            glam::DVec3::new(-7556421765.651863, 5260956590.476351, -2963316063.6143136),
            glam::DVec3::new(-2417514993.952572, 58948027.124627456, 332037521.0480978),
            glam::DVec3::new(-401231813.01364976, -286934665.0235125, -8306899057.677289),
            glam::DVec3::new(-423873876.7793667, 816838646.0451242, -1936605057.1374884),
            glam::DVec3::new(1638840880.6982732, 3811586986.433357, -5606389740.5359535),
            glam::DVec3::new(485379156.6587149, 3877298386.9417615, 3261608187.700436),
            glam::DVec3::new(127175105.33258279, 8486506.652578589, -8494235452.686916),
            glam::DVec3::new(890350698.4192543, -1545042792.5320225, -253693647.0788635),
            glam::DVec3::new(-965000269.7722342, 887134681.2380865, 2303222856.352874),
            glam::DVec3::new(-3293865707.0306754, 1800460317.3511906, -2513657365.399923),
            glam::DVec3::new(4493418846.659723, 3224078407.769552, 5434256241.694285),
            glam::DVec3::new(-222944108.97574744, 110505773.3995093, 1243887305.7373564),
            glam::DVec3::new(1642387205.8161027, 1868181939.908915, 3064602177.6773486),
            glam::DVec3::new(2947321316.0621667, -1769493145.9165955, 1469058917.9567044),
            glam::DVec3::new(2450561222.4285192, -1297800536.1928244, -4934781507.158352),
            glam::DVec3::new(11341319.896049185, 13227002.021572359, -2090455113.763534),
        ];
        return (llas, ecefs);
    }

    #[rstest]
    fn test_ecef2lla_ferarri(ecef_fixture: (Vec<glam::DVec3>, Vec<glam::DVec3>)) {
        let ecefs = ecef_fixture.0;
        let llas = ecef_fixture.1;
        for (i, ecef) in ecefs.iter().enumerate() {
            let actual_lla: glam::DVec3 = ecef2lla_ferarri(ecef);
            let expected_lla = llas.get(i).unwrap();
            assert_vecs_close(&actual_lla, expected_lla, 1e-6);
        }
    }
    #[rstest]
    fn test_ecef2lla_map3d(ecef_fixture: (Vec<glam::DVec3>, Vec<glam::DVec3>)) {
        let ecefs = ecef_fixture.0;
        let llas = ecef_fixture.1;
        for (i, ecef) in ecefs.iter().enumerate() {
            let actual_lla: glam::DVec3 = ecef2lla_map3d(ecef);
            let expected_lla = llas.get(i).unwrap();
            assert_vecs_close(&actual_lla, expected_lla, 1e-6);
        }
    }

    #[test]
    fn test_ecef2lla_zero_map3() {
        let zero = glam::DVec3::new(0., 0., 0.);
        let actual = ecef2lla_map3d(&zero);
        assert!(almost::zero(actual.x));
        assert!(almost::zero(actual.y));
        assert!(almost::equal_with(actual.z, -6378137.0, 1e-6));
    }
    #[test]
    fn test_ecef2lla_zero_ferrari() {
        let zero = glam::DVec3::new(0., 0., 0.);
        let actual = ecef2lla_ferarri(&zero);
        assert!(almost::zero(actual.x));
        assert!(almost::zero(actual.y));
        assert!(almost::equal_with(actual.z, -6292741.654585736, 1e-6));
    }

    #[rstest]
    fn test_lla2ecef(lla_fixture: (Vec<glam::DVec3>, Vec<glam::DVec3>)) {
        let llas = lla_fixture.0;
        let ecefs = lla_fixture.1;
        for (i, lla) in llas.iter().enumerate() {
            let actual_ecef = lla2ecef(lla);
            let expected_ecef = ecefs.get(i).unwrap();
            assert_vecs_close(&actual_ecef, expected_ecef, 1e-6);
        }
    }
    #[test]
    fn test_dms_lower2dd() {
        let lat_dms = "25:22:44.738n";
        let lon_dms = "74:59:55.525w";
        let lat = dms2dd(&lat_dms);
        let lon = dms2dd(&lon_dms);

        assert!(lat.is_ok());
        assert!(lon.is_ok());
        assert!(almost::equal_with(lat.unwrap(), 25.379094, 1e-6));
        assert!(almost::equal_with(lon.unwrap(), -74.998757, 1e-6));
    }
    #[test]
    fn test_dms_upper2dd() {
        let lat_dms = "25:22:44.738N";
        let lon_dms = "74:59:55.525W";
        let lat = dms2dd(&lat_dms);
        let lon = dms2dd(&lon_dms);

        assert!(lat.is_ok());
        assert!(lon.is_ok());
        assert!(almost::equal_with(lat.unwrap(), 25.379094, 1e-6));
        assert!(almost::equal_with(lon.unwrap(), -74.998757, 1e-6));
    }
    #[test]
    fn test_dms2dd_throws_err_on_bad_cardinal() {
        let lat_dms = "25:22:44.738R";
        let lat = dms2dd(&lat_dms);

        assert!(lat.is_err());
    }
    #[test]
    fn test_dms2dd_int_sec() {
        let lat_dms = "25:22:44N";
        let lat = dms2dd(&lat_dms);

        assert!(lat.is_ok());
    }
    #[test]
    fn test_dms2dd_throws_err_on_bad_min() {
        let lat_dms = "25:2244N";
        let lat = dms2dd(&lat_dms);

        assert!(lat.is_err());
    }

    #[test]
    fn test_dd2dms() {
        let lat_dd = 25.379094;
        let lon_dd = -74.998757;
        let lat_dms = dd2dms(lat_dd, true);
        let lon_dms = dd2dms(lon_dd, false);
        assert!(lat_dms == "25:22:44.738N");
        assert!(lon_dms == "74:59:55.525W");
    }
}

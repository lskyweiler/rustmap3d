extern crate nalgebra_glm as glm;
use almost;

static EARTH_SEMI_MAJOR_AXIS: f64 = 6378137.0;
static EARTH_SEMI_MAJOR_AXIS_2: f64 = EARTH_SEMI_MAJOR_AXIS * EARTH_SEMI_MAJOR_AXIS;
static EARTH_SEMI_MINOR_AXIS: f64 = 6356752.314245;
static EARTH_SEMI_MINOR_AXIS_2: f64 = EARTH_SEMI_MINOR_AXIS * EARTH_SEMI_MINOR_AXIS;
static EARTH_FLATTENING_FACTOR: f64 = 0.003352810664740;
static EARTH_ANGULAR_VEL_RADPS: f64 = 7.292115900000000e-05;
static EARTH_E: f64 = 521854.0084255785;
static EARTH_E_2: f64 = EARTH_E * EARTH_E;

static _ECEF2LLA_A: f64 = EARTH_SEMI_MAJOR_AXIS;
static _ECEF2LLA_B: f64 = _ECEF2LLA_A * (1.0 - EARTH_FLATTENING_FACTOR);
static _ECEF2LLA_A2: f64 = _ECEF2LLA_A * _ECEF2LLA_A;
static _ECEF2LLA_B2: f64 = _ECEF2LLA_B * _ECEF2LLA_B;
static _ECEF2LLA_E2: f64 = 1.0 - _ECEF2LLA_B2 / _ECEF2LLA_A2;
static _ECEF2LLA_EP2: f64 = _ECEF2LLA_A2 / _ECEF2LLA_B2 - 1.0;
static _ECEF2LLA_E2EP2: f64 = _ECEF2LLA_E2 * _ECEF2LLA_EP2;
static _ECEF2LLA_EP22: f64 = _ECEF2LLA_EP2 * _ECEF2LLA_EP2;
static _ECEF2LLA_EP24: f64 = _ECEF2LLA_EP22 * _ECEF2LLA_EP22;
static _ECEF2LLA_TWO_THRIDS: f64 = 2.0 / 3.0;

pub fn ecef2lla_ferarri(x: f64, y: f64, z: f64) -> (f64, f64, f64) {
    let z_ecef_squared: f64 = f64::powf(z, 2.);
    let range_squared: f64 = f64::powf(x, 2.) + f64::powf(y, 2.);
    let range_: f64 = f64::sqrt(range_squared);
    let f_term: f64 = 54. * f64::powf(EARTH_SEMI_MINOR_AXIS, 2.) * z_ecef_squared;
    let g_term: f64 = range_squared + (1. - _ECEF2LLA_E2) * z_ecef_squared
        - _ECEF2LLA_E2
            * (f64::powf(EARTH_SEMI_MAJOR_AXIS, 2.) - f64::powf(EARTH_SEMI_MINOR_AXIS, 2.));

    let c_term: f64 = _ECEF2LLA_EP22 * f_term * range_squared / f64::powf(g_term, 3.);
    let c_term_sqrt_mod: f64 = f64::sqrt(f64::powf(c_term, 2.) + 2. * c_term);
    let s_term: f64 = f64::powf(1. + c_term + c_term_sqrt_mod, 1. / 3.);
    let p_term: f64 =
        f_term / (3. * f64::powf(s_term + 1. / s_term + 1., 2.) * f64::powf(g_term, 2.));
    let q_term: f64 = f64::sqrt(1. + 2. * _ECEF2LLA_EP22 * p_term);
    let sqrt_mod2: f64 = f64::sqrt(
        0.5 * f64::powf(EARTH_SEMI_MAJOR_AXIS, 2.) * (1. + 1. / q_term)
            - p_term * (1. - _ECEF2LLA_E2) * z_ecef_squared / (q_term * (1. + q_term))
            - 0.5 * p_term * range_squared,
    );
    let r0_term: f64 = -(p_term * _ECEF2LLA_E2 * range_) / (1. + q_term) + sqrt_mod2;
    let uv_subterm: f64 = f64::powf(range_ - _ECEF2LLA_E2 * r0_term, 2.);
    let u_term: f64 = f64::sqrt(uv_subterm + z_ecef_squared);
    let v_term: f64 = f64::sqrt(uv_subterm + (1. - _ECEF2LLA_E2) * z_ecef_squared);
    let z0_term: f64 = f64::powf(EARTH_SEMI_MINOR_AXIS, 2.) * z / (EARTH_SEMI_MAJOR_AXIS * v_term);
    let lat_rad: f64 = if range_ != 0. {
        f64::atan2(z + _ECEF2LLA_EP2 * z0_term, range_)
    } else {
        0.0
    };
    let lon_rad: f64 = f64::atan2(y, x);
    let alt: f64 =
        u_term * (1. - f64::powf(EARTH_SEMI_MINOR_AXIS, 2.) / (EARTH_SEMI_MAJOR_AXIS * v_term));

    let lat: f64 = f64::to_degrees(lat_rad);
    let lon: f64 = f64::to_degrees(lon_rad);
    return (lat, lon, alt);
}

pub fn ecef2lla_map3d(x: f64, y: f64, z: f64) -> (f64, f64, f64) {
    let r = f64::sqrt(x * x + y * y + z * z);
    let r2 = r * r;
    let u = f64::sqrt(0.5 * (r2 - EARTH_E_2) + 0.5 * f64::hypot(r2 - EARTH_E_2, 2.0 * EARTH_E * z));
    let hxy = f64::hypot(x, y);
    let hue = f64::hypot(u, EARTH_E);

    let mut beta = std::f64::consts::FRAC_PI_2 * f64::signum(z);

    if !almost::zero(u) && !almost::zero(hxy) {
        beta = f64::atan(hue / u * z / hxy);
        beta += ((EARTH_SEMI_MINOR_AXIS * u - EARTH_SEMI_MAJOR_AXIS * hue + EARTH_E_2)
            * f64::sin(beta))
            / (EARTH_SEMI_MAJOR_AXIS * hue * 1. / f64::cos(beta) - EARTH_E_2 * f64::cos(beta))
    }

    let lat = f64::atan(EARTH_SEMI_MAJOR_AXIS / EARTH_SEMI_MINOR_AXIS * f64::tan(beta));
    let lon = f64::atan2(y, x);

    let mut alt = f64::hypot(
        z - EARTH_SEMI_MINOR_AXIS * f64::sin(beta),
        hxy - EARTH_SEMI_MAJOR_AXIS * f64::cos(beta),
    );
    let inside = x * x / EARTH_SEMI_MAJOR_AXIS_2
        + y * y / EARTH_SEMI_MAJOR_AXIS_2
        + z * z / EARTH_SEMI_MINOR_AXIS_2
        < 1.;
    alt *= if inside { -1. } else { 1.0 };
    return (f64::to_degrees(lat), f64::to_degrees(lon), alt);
}

pub fn ecef2lla(x: f64, y: f64, z: f64, use_ferarri: Option<bool>) -> (f64, f64, f64) {
    if use_ferarri.unwrap_or(false) {
        return ecef2lla_ferarri(x, y, z);
    } else {
        return ecef2lla_map3d(x, y, z);
    }
}

pub fn lla2ecef(lat: f64, lon: f64, alt: f64) -> (f64, f64, f64) {
    let lat = f64::to_radians(lat);
    let lon = f64::to_radians(lon);

    let alt_correction = EARTH_SEMI_MAJOR_AXIS_2
        / f64::hypot(
            EARTH_SEMI_MAJOR_AXIS * f64::cos(lat),
            EARTH_SEMI_MINOR_AXIS * f64::sin(lat),
        );
    let x = (alt_correction + alt) * f64::cos(lat) * f64::cos(lon);
    let y = (alt_correction + alt) * f64::cos(lat) * f64::sin(lon);
    let z = (alt_correction * f64::powf(EARTH_SEMI_MINOR_AXIS / EARTH_SEMI_MAJOR_AXIS, 2.0) + alt)
        * f64::sin(lat);
    return (x, y, z);
}

pub fn ecef2eci(x: f64, y: f64, z: f64, time_since_eci_lock: f64) -> (f64, f64, f64) {
    let earth_rotation_angle: f64 = time_since_eci_lock * EARTH_ANGULAR_VEL_RADPS;
    let mut rot: glm::DMat4 = glm::DMat4::identity();
    rot = glm::rotate_z(&rot, earth_rotation_angle);
    let eci: glm::DVec4 = rot * glm::DVec4::new(x, y, z, 1.);
    return (eci.x, eci.y, eci.z);
}
//
// fn enu2ecef_quat(lat: f64, lon: f64) -> glm::DQuat {
//     let lat_rad: f64 = f64::to_radians(lat);
//     let lon_rad: f64 = f64::to_radians(lon);
//
//     let mut dcm: glm::DQuat = glm::DQuat::identity();
//     dcm = glm::quat_rotate(&dcm, std::f64::consts::FRAC_PI_2 + lon_rad, &glm::DVec3::new(0., 0., 1.));
//     dcm = glm::quat_rotate(&dcm, std::f64::consts::FRAC_PI_2 - lat_rad, &glm::DVec3::new(1., 0., 0.));
//     return dcm;
// }
//
// pub fn enu2ecef_dcm(lat: f64, lon: f64) -> (f64, f64, f64, f64) {
//     let q = enu2ecef_quat(lat, lon);
//     return (q.i, q.j, q.k, q.w);
// }
//
// fn ecef2enu_quat(lat: f64, lon: f64) -> glm::DQuat {
//     let dcm: glm::DQuat = enu2ecef_quat(lat, lon);
//     let dcm: glm::DQuat = glm::quat_conjugate(&dcm);
//     return dcm;
// }
// pub fn ecef2enu_dcm(lat: f64, lon: f64) -> (f64, f64, f64, f64) {
//     let q = ecef2enu_quat(lat, lon);
//     return (q.i, q.j, q.k, q.w);
// }

#[cfg(test)]
mod tests {
    // Note this useful idiom: importing names from outer (for mod tests) scope.
    use super::*;

    #[test]
    fn test_ecef2lla_ferarri() {
        let actual = ecef2lla_ferarri(1., 2., 3.);
    }
    #[test]
    fn test_lla2ecef(){
        let llas: Vec<(f64, f64, f64)> = vec![
            (-56.66172812377343, 155.68130312212668, 718875820.2943813),
            (-33.26507252871781, -31.964572932142886, 7065343507.721147),
            (6.371772880304988, -59.23405806377991, 8352922369.965223),
            (-70.74335211522242, 120.07092031064208, 4526779918.14558),
            (-24.461510525146508, -59.62407504106517, 2420545872.95108),
            (-72.3152549895963, 101.64767964829839, 3795852142.154489),
            (50.1705938605771, 105.40775429487171, 7390834021.2357435),
            (-32.30257694254681, -135.9790191808388, 2106745558.3293736),
            (-19.934988367833313, 65.33934397460283, 6927358798.01723),
            (23.702048312802475, 150.7801192499238, 4151592272.73058),
            (-44.139744893078216, -10.394871262469564, 1308050706.656194),
            (80.33315640988386, -167.00766083959087, 4853517330.590896),
            (89.5832033021365, -146.3731250375061, 5541861449.666565),
            (6.002327772952086, 178.1723518113481, 8243112160.578481),
            (-30.240836896769675, 5.48046771739655, 774183075.5164747),
            (-66.50252282873791, -130.9736602675406, 6467951118.810617),
            (32.244715738302816, 134.35132917429144, 8312562383.178297),
            (-40.929722118941775, -166.0099481725839, 6216809556.871332),
            (-3.161999721921518, -121.6362613721991, 1686482297.1376824),
            (70.69015001986463, -18.77422907679727, 4347545178.616857),
            (-77.89686967839751, 76.8590940393218, 275547474.5753206),
            (56.70256197263234, 31.487362633295106, 3351651550.650465),
            (-8.537166774081555, 124.9300397881874, 6964157346.925901),
            (-3.3883037953841466, 159.9876304913834, 5824233554.097087),
            (3.3173719339601604, 19.53837842565602, 7727952079.516271),
            (-29.656043118692313, 50.423126817903324, 3109749957.585276),
            (-3.317182434873658, 115.18183094259633, 6783718307.968782),
            (87.99594613540816, 113.8812711654927, 6548397030.25793),
            (42.155897640035505, -7.385067279532194, 4223681045.2455497),
            (-45.66421911004391, 165.75779873561737, 9231088702.94254),
            (74.98669309517001, -139.02652259218758, 7752650661.051985),
            (0.04388642130096798, 139.12251298740722, 9856972131.89139),
            (18.913840578810465, -152.22766889844678, 8674973608.740688),
            (-70.71679219133352, 168.7938671723137, 2559962939.2985635),
            (-17.840267641554703, 145.15346668909052, 9666183830.211079),
            (7.81829152155791, 178.60319210639625, 2434544733.5319457),
            (-86.60171868991569, -144.4301374069363, 8315174907.876453),
            (-64.583522424254, 117.42569962019991, 2137778660.492624),
            (-53.497097618994985, 66.73409062380492, 6968275498.757018),
            (39.85170162247488, 82.86455367169816, 5083534728.543176),
            (-89.14033256764529, 3.817737674123464, 8488834909.559485),
            (-8.097151555351061, -60.04675219817315, 1794800252.0766892),
            (60.35525557045381, 137.40735372191648, 2643745363.528964),
            (-33.80752224309167, 151.33853754701425, 4511336080.320615),
            (44.49769217327577, 35.65986371715448, 7747124179.421991),
            (78.6881809933453, 153.6339574621485, 1262173554.861432),
            (50.934753774135714, 48.68011259868561, 3940700238.395185),
            (23.139059023558076, -30.9794825232527, 3732066418.3197985),
            (-60.66725009582464, -27.905389704786387, 5654168566.610009),
            (-89.52247206976813, 49.3889940119827, 2084170969.4723942),
        ];
        let ecefs: Vec<(f64, f64, f64)> = vec![
            (-363226337.8378485, 164145694.11941433, -605883225.5202017),
            (5016417802.618961, -3130294546.8914347, -3878912812.734696),
            (4249637013.1107903, -7138476691.558969, 927704883.3233159),
            (-749123383.4568931, 1293818655.7673783, -4279508924.526832),
            (1117071280.1290438, -1905836230.5036347, -1004929049.7808872),
            (-233195557.85231522, 1131259987.718429, -3622523730.6242113),
            (-1258810196.8083477, 4567666948.353717, 5680702139.465385),
            (-1284356369.4257667, -1241197752.6029196, -1129213367.408031),
            (2719703477.4990993, 5923770937.923891, -2364069442.8993587),
            (-3322780364.3305306, 1858552546.759458, 1671407285.5992749),
            (927816843.0914981, -170200408.00971267, -915359858.8829931),
            (-795180643.7592813, -183469932.25196317, 4790867197.061249),
            (-33606494.82719547, -22350830.551816523, 5548071401.239205),
            (-8200090300.39463, 261659157.03025025, 862635420.7610741),
            (671261500.4702597, 64404182.064525165, -393099795.1348588),
            (-1692640892.2722268, -1948969436.132962, -5937439805.014645),
            (-4918541001.068794, 5031191125.999227, 4438438970.340508),
            (-4562247156.197555, -1136654630.154329, -4076992529.3376484),
            (-886595545.8633595, -1439098934.8425846, -93374639.72836758),
            (1363144037.4345145, -463368006.7734003, 4108967125.721511),
            (13439868.383440318, 57568179.5971645, -275637077.97071016),
            (1572069597.6960115, 962888601.5025085, 2806725105.8847656),
            (-3946937207.6022162, 5651485039.359078, -1034776257.2819849),
            (-5468975219.234577, 1991881397.6015773, -344601758.5886673),
            (7276755652.2802105, 2582321037.9901214, 447557846.6049511),
            (1725273328.2316494, 2087211973.2923207, -1541817056.8399067),
            (-2884293340.9078116, 6134483975.098778, -392895716.5994358),
            (-92799096.46533564, 209598007.20691597, 6550744575.743122),
            (3109827878.675527, -403071979.06005263, 2838982707.422298),
            (-6257302596.723574, 1588245806.4929829, -6607134865.92882),
            (-1517523803.7477174, -1317930511.8018465, 7494157658.802485),
            (-7457782245.210336, 6455006230.264203, 7554923.466331962),
            (-7266578347.596589, -3826748677.7182827, 2814011879.0678635),
            (-831350760.9757568, 164704360.3558542, -2422341250.456174),
            (-7556421765.651863, 5260956590.476351, -2963316063.6143136),
            (-2417514993.952572, 58948027.124627456, 332037521.0480978),
            (-401231813.01364976, -286934665.0235125, -8306899057.677289),
            (-423873876.7793667, 816838646.0451242, -1936605057.1374884),
            (1638840880.6982732, 3811586986.433357, -5606389740.5359535),
            (485379156.6587149, 3877298386.9417615, 3261608187.700436),
            (127175105.33258279, 8486506.652578589, -8494235452.686916),
            (890350698.4192543, -1545042792.5320225, -253693647.0788635),
            (-965000269.7722342, 887134681.2380865, 2303222856.352874),
            (-3293865707.0306754, 1800460317.3511906, -2513657365.399923),
            (4493418846.659723, 3224078407.769552, 5434256241.694285),
            (-222944108.97574744, 110505773.3995093, 1243887305.7373564),
            (1642387205.8161027, 1868181939.908915, 3064602177.6773486),
            (2947321316.0621667, -1769493145.9165955, 1469058917.9567044),
            (2450561222.4285192, -1297800536.1928244, -4934781507.158352),
            (11341319.896049185, 13227002.021572359, -2090455113.763534),
        ];

        for (i, lla) in llas.iter().enumerate(){
            let actual_ecef = lla2ecef(lla.0, lla.1, lla.2);
            assert!(almost::equal_with(actual_ecef.0, ecefs.get(i).unwrap().0, 1e-6));
            assert!(almost::equal_with(actual_ecef.1, ecefs.get(i).unwrap().1, 1e-6));
            assert!(almost::equal_with(actual_ecef.2, ecefs.get(i).unwrap().2, 1e-6));
        }

    }
}

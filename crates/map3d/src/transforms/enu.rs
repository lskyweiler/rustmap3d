use crate::{
    traits::{IntoDVec3, IntoLatLonTriple, IntoLatLonTuple},
    transforms::lla,
};
use glam;

/// Calculates the quaternion that yields an ENU to ECEF transformation at this LLA.
///
/// # Arguments
///
/// * `ll_deg` - Lat lon tuple [degrees-degrees]
///
/// # Returns
///
/// * `quat` - Normalized ENU to ECEF quaternion.
pub fn enu2ecef_quat(ll_deg: impl IntoLatLonTuple) -> glam::DQuat {
    let ll_deg = ll_deg.into_lat_lon_tuple();

    let lat_rad: f64 = f64::to_radians(ll_deg.0);
    let lon_rad: f64 = f64::to_radians(ll_deg.1);

    let yaw = glam::DQuat::from_rotation_z(std::f64::consts::FRAC_PI_2 + lon_rad);
    let roll = glam::DQuat::from_rotation_x(std::f64::consts::FRAC_PI_2 - lat_rad);
    return yaw * roll;
}

/// Calculates the quaternion that yields an ECEF to ENU transformation at this LLA.
///
/// # Arguments
///
/// * `ll_deg` - Lat lon tuple [degrees-degrees]
///
/// # Returns
///
/// * `quat` - Normalized ECEF to ENU quaternion.
pub fn ecef2enu_quat(ll_deg: impl IntoLatLonTuple) -> glam::DQuat {
    let enu2ecef_rot = enu2ecef_quat(ll_deg);
    return enu2ecef_rot.conjugate();
}

/// Calculates the direction cosine matrix that yields an ENU to ECEF transformation at this LLA.
///
/// # Arguments
///
/// * `ll_deg` - Lat lon tuple [degrees-degrees]
///
/// # Returns
///
/// * `dcm` - ENU to ECEF direction cosine matrix.
pub fn enu2ecef_dcm(ll_deg: impl IntoLatLonTuple) -> glam::DMat3 {
    let q = enu2ecef_quat(ll_deg);
    return glam::DMat3::from_quat(q);
}

/// Calculates the direction cosine matrix that yields an ECEF to ENU transformation at this LLA.
///
/// # Arguments
///
/// * `ll_deg` - Lat lon tuple [degrees-degrees]
///
/// # Returns
///
/// * `dcm` - ECEF to ENU direction cosine matrix.
pub fn ecef2enu_dcm(ll_deg: impl IntoLatLonTuple) -> glam::DMat3 {
    let q = ecef2enu_quat(ll_deg);
    return glam::DMat3::from_quat(q);
}

/// Converts ECEF uvw to ENU
///
/// # Arguments
///
/// * `ecef_uvw` - Vector represented in ECEF frame [[meters]].
/// * `ll_ref` - Reference latitude-longitude [[degrees-degrees]].
///
/// # Returns
///
/// * `enu` - Vector represented in ENU coordinates [[meters]].
pub fn ecef_uvw2enu(ecef_uvw: impl IntoDVec3, ll_ref: impl IntoLatLonTuple) -> glam::DVec3 {
    let ecef_uvw = ecef_uvw.into_dvec3();

    let rot = ecef2enu_quat(ll_ref);
    return rot * ecef_uvw;
}
/// Converts Absolute ECEF to ENU
///
/// # Arguments
///
/// * `ecef` - Absolute ECEF location [[meters]].
/// * `lla_ref` - Reference latitude-longitude-altitude [[degrees-degrees-meters]].
///
/// # Returns
///
/// * `enu` - Vector represented in ENU coordinates [[meters]].
pub fn ecef2enu(ecef: impl IntoDVec3, lla_ref: impl IntoLatLonTriple) -> glam::DVec3 {
    let ecef = ecef.into_dvec3();
    let lla_ref = lla_ref.into_lat_lon_triple();

    let ecef_uvw = ecef - lla::lla2ecef(lla_ref);
    let rot = ecef2enu_quat(lla_ref);
    return rot * ecef_uvw;
}

/// Converts ENU to ECEF uvw.
/// This is a vector that is not in relation to an ECEF location
///
/// # Arguments
///
/// * `enu` - Vector represented in ENU coordinates [[meters]].
/// * * `ll_ref` - Reference latitude-longitude-altitude [[degrees-degrees-meters]].
///
/// # Returns
///
/// * `ecef_uvw` - Vector represented in ECEF frame. Not an absolute position [[meters]].
pub fn enu2ecef_uvw(enu: impl IntoDVec3, ll_ref: impl IntoLatLonTuple) -> glam::DVec3 {
    let enu = enu.into_dvec3();

    let rot = enu2ecef_quat(ll_ref);
    return rot * enu;
}
/// Converts ENU to Absolute ECEF.
/// This is a vector that is not in relation to an ECEF location
///
/// # Arguments
///
/// * `enu` - Vector represented in ENU coordinates [[meters]].
/// * `lla_ref` - Reference latitude-longitude-altitude [[degrees-degrees-meters]].
///
/// # Returns
///
/// * `ecef` - Vector represented in ECEF frame. Not an absolute position [[meters]].
pub fn enu2ecef(enu: impl IntoDVec3, lla_ref: impl IntoLatLonTriple) -> glam::DVec3 {
    let enu = enu.into_dvec3();
    let lla_ref = lla_ref.into_lat_lon_triple();

    let rot = enu2ecef_quat(lla_ref);
    let ecef_uvw = rot * enu;
    return ecef_uvw + lla::lla2ecef(lla_ref);
}

/// Calculates heading angle from ENU.
///
/// # Arguments
///
/// * `enu` - Vector represented in ENU coordinates [[meters]].
///
/// # Returns
///
/// * `heading_deg` - Heading angle relative to true north [[degrees]].
pub fn enu2heading(enu: impl IntoDVec3) -> f64 {
    let enu = enu.into_dvec3();

    return f64::atan2(enu.x, enu.y).to_degrees();
}

#[cfg(test)]
mod test_enu {
    use super::*;
    use crate::utils::assert_vecs_close;
    use rstest::*;

    #[fixture]
    fn ecef2enu_fixture() -> (Vec<(glam::DVec3, glam::DVec3)>, Vec<glam::DVec3>) {
        let ecef_ref_tuple: Vec<(glam::DVec3, glam::DVec3)> = vec![
            (
                glam::DVec3::new(91619.6484735217, 6925836.249227896, 5957639.1833251435),
                glam::DVec3::new(-50.173150092148774, -112.00193826311323, 0.0),
            ),
            (
                glam::DVec3::new(247424.65845609456, 6213457.926448222, -4561090.24503099),
                glam::DVec3::new(82.57782955532258, 179.8856930744933, 0.0),
            ),
            (
                glam::DVec3::new(4192418.204552155, 7351021.414750714, 1979901.0055067968),
                glam::DVec3::new(62.457227071538824, 55.92173622713477, 0.0),
            ),
            (
                glam::DVec3::new(-8588031.646098651, -7486734.193169072, -9230929.386199847),
                glam::DVec3::new(28.342454025165626, -92.0830991512501, 0.0),
            ),
            (
                glam::DVec3::new(1427216.04466255, 9403604.322131746, -963231.1375281066),
                glam::DVec3::new(30.516610115227095, -89.90471953518953, 0.0),
            ),
            (
                glam::DVec3::new(6642925.238247002, 5903029.863253074, -2232073.9836572986),
                glam::DVec3::new(45.400629146794955, -67.73807362936134, 0.0),
            ),
            (
                glam::DVec3::new(2591477.518994638, -5409635.6294355355, -8285118.102197416),
                glam::DVec3::new(19.970663213164315, -93.2950875704390, 0.0),
            ),
            (
                glam::DVec3::new(2528320.789589662, -3495684.1279134876, 8360309.7196244),
                glam::DVec3::new(66.11400734475339, -95.08388981259206, 0.0),
            ),
            (
                glam::DVec3::new(-9735848.884452593, -8556065.641175192, 8333421.846976527),
                glam::DVec3::new(-14.691539720606187, -2.33006016905326, 0.0),
            ),
            (
                glam::DVec3::new(7426961.41892605, 1655943.456426185, 6591322.351593178),
                glam::DVec3::new(-15.910515256034472, 165.4827913906837, 0.0),
            ),
            (
                glam::DVec3::new(-3196405.2009189655, -9604287.523781441, -1060713.7649627458),
                glam::DVec3::new(-27.470280603087133, 102.2454141837, 0.0),
            ),
            (
                glam::DVec3::new(7335138.4135223925, 2344465.657677945, 8588377.836181395),
                glam::DVec3::new(-10.5382655567836, -167.02758187958835, 0.0),
            ),
            (
                glam::DVec3::new(7310959.32431601, 8350601.114886332, 6569499.540019801),
                glam::DVec3::new(63.317884611786326, 23.40930303068805, 0.0),
            ),
            (
                glam::DVec3::new(-8640546.064591233, -2663666.0467856065, 1614399.1752727698),
                glam::DVec3::new(72.66364577543774, 163.0609809895735, 0.0),
            ),
            (
                glam::DVec3::new(-8068883.573257133, -1893830.0686977436, 8266480.72264607),
                glam::DVec3::new(-25.23522156265527, 125.3010823588916, 0.0),
            ),
            (
                glam::DVec3::new(-8234811.869950183, -3807444.077073443, 462885.92168885097),
                glam::DVec3::new(-64.46261342414945, 130.7529101785710, 0.0),
            ),
            (
                glam::DVec3::new(5745089.68100643, 8186227.487702135, 1050884.6330531444),
                glam::DVec3::new(-64.03982768384915, 2.6480032202244956, 0.0),
            ),
            (
                glam::DVec3::new(2393154.1929320674, 2067816.501552973, 3732474.8825010224),
                glam::DVec3::new(-60.74611032083097, 30.92056758445699, 0.0),
            ),
            (
                glam::DVec3::new(1951197.9768817485, -7575128.269719007, 9939982.202217888),
                glam::DVec3::new(42.50213359486636, 113.81499658197713, 0.0),
            ),
            (
                glam::DVec3::new(6216696.544619927, -2057542.5103991907, 1143021.5607241336),
                glam::DVec3::new(-30.03750128679215, -110.032118791896, 0.0),
            ),
            (
                glam::DVec3::new(8262888.4250550615, 3106595.2889045794, 9812866.097362377),
                glam::DVec3::new(15.123933131017566, 164.52989386282826, 0.0),
            ),
            (
                glam::DVec3::new(7218924.780765143, -447912.04118075967, 5171595.8773390055),
                glam::DVec3::new(81.52927937810878, -105.2611885755099, 0.0),
            ),
            (
                glam::DVec3::new(-912999.7792285755, 2374762.7068727836, -4955536.282540757),
                glam::DVec3::new(10.831088217119586, 151.2254067023637, 0.0),
            ),
            (
                glam::DVec3::new(3623182.097599946, -124808.5784964189, 6830241.830351334),
                glam::DVec3::new(71.11579335672388, -144.99222808538826, 0.0),
            ),
            (
                glam::DVec3::new(2208932.904275298, -3444783.5638758554, -7103490.483277793),
                glam::DVec3::new(22.462213216531538, -26.6324481797208, 0.0),
            ),
            (
                glam::DVec3::new(698713.3718823884, -8409581.988125127, -3903578.000227963),
                glam::DVec3::new(-87.47177257040715, -13.04761510545472, 0.0),
            ),
            (
                glam::DVec3::new(-9556270.567275014, -7440669.805674921, 2815381.5153284855),
                glam::DVec3::new(7.966991562503196, 100.79184677036972, 0.0),
            ),
            (
                glam::DVec3::new(499026.44389439, -1925421.3250966482, 7925563.273168109),
                glam::DVec3::new(-71.1002119000218, -138.06002686196018, 0.0),
            ),
            (
                glam::DVec3::new(7238450.744717773, -4383306.046812624, -8262036.304761575),
                glam::DVec3::new(-13.118964393782036, 74.96561856267363, 0.0),
            ),
            (
                glam::DVec3::new(-326376.61228840984, 7739298.466333255, -5871984.449330825),
                glam::DVec3::new(-72.49265123799492, 91.02269534514028, 0.0),
            ),
            (
                glam::DVec3::new(-1226800.3167807497, 7604616.1196159385, 244802.38239366747),
                glam::DVec3::new(45.976409115548705, -86.172644141553, 0.0),
            ),
            (
                glam::DVec3::new(-6183006.182703774, 1260934.9898499493, 5405250.482727632),
                glam::DVec3::new(-46.5078875763827, 96.5794699892873, 0.0),
            ),
            (
                glam::DVec3::new(4892851.839336393, -4366090.555875951, -284396.9303730335),
                glam::DVec3::new(-81.70294712853254, -160.1769692625534, 0.0),
            ),
            (
                glam::DVec3::new(9237967.029159822, 5403631.481081279, -3005393.284370361),
                glam::DVec3::new(-79.40102319570062, 38.14134828047784, 0.0),
            ),
            (
                glam::DVec3::new(-6538766.213631488, -4940418.412632655, 2501053.290733097),
                glam::DVec3::new(13.527045111462556, 102.20616898781418, 0.0),
            ),
            (
                glam::DVec3::new(-8784252.36675533, -8232153.597217414, 4884503.641264852),
                glam::DVec3::new(30.28635667737656, -169.57611510228907, 0.0),
            ),
            (
                glam::DVec3::new(-4943652.169548029, -3855871.065791603, -4748617.896841602),
                glam::DVec3::new(74.73754290271745, 152.61923201200796, 0.0),
            ),
            (
                glam::DVec3::new(2469337.744048135, -9190045.568734694, 4837736.320298016),
                glam::DVec3::new(-12.948807601309923, 46.97162855809245, 0.0),
            ),
            (
                glam::DVec3::new(-1433624.4808392152, -182826.48301463015, -5022036.343881993),
                glam::DVec3::new(-45.74760712894212, 152.37356328703, 0.0),
            ),
            (
                glam::DVec3::new(5344321.103997834, 2878251.6468978976, -2881088.089473922),
                glam::DVec3::new(30.33833424064295, -69.68258737927425, 0.0),
            ),
            (
                glam::DVec3::new(6606169.345653623, 322403.8278391063, 1042680.5917547438),
                glam::DVec3::new(62.631186380765996, -139.94615572408208, 0.0),
            ),
            (
                glam::DVec3::new(667958.4714587647, 8478848.064964425, 1694820.8740469385),
                glam::DVec3::new(-78.2754170386466, -114.12583810194599, 0.0),
            ),
            (
                glam::DVec3::new(8174259.422747154, 2536147.3532497776, -9862214.372994114),
                glam::DVec3::new(71.51999174030132, -146.96589979434754, 0.0),
            ),
            (
                glam::DVec3::new(-3472815.7693284587, 4576504.523164628, 5061670.1162975095),
                glam::DVec3::new(-2.043219531494401, 8.809342235384065, 0.0),
            ),
            (
                glam::DVec3::new(4164265.524093671, -8876967.849787056, 5611143.587449348),
                glam::DVec3::new(-77.72886207380276, -58.502589304409916, 0.0),
            ),
            (
                glam::DVec3::new(9676345.130538844, -6880885.856225148, 2825292.9134078994),
                glam::DVec3::new(18.694870013985096, 74.11457134859293, 0.0),
            ),
            (
                glam::DVec3::new(8345975.870587062, 5602669.236328628, 6440366.360817695),
                glam::DVec3::new(-20.537448114795794, 80.45056404450543, 0.0),
            ),
            (
                glam::DVec3::new(76565.8634340372, -5176255.943787364, 8534384.933999725),
                glam::DVec3::new(4.390777736040391, -163.5117863846071, 0.0),
            ),
            (
                glam::DVec3::new(-8621776.079462763, 8582232.035568487, -7881480.052577838),
                glam::DVec3::new(70.39862590743721, -50.56334026059545, 0.0),
            ),
            (
                glam::DVec3::new(-4399316.195656607, -6898512.909927797, 4952200.285973493),
                glam::DVec3::new(13.452784771177136, -80.37984986897065, 0.0),
            ),
        ];
        let enu: Vec<glam::DVec3> = vec![
            glam::DVec3::new(-2509734.058866111, -1142229.8051993023, -8710086.829399489),
            glam::DVec3::new(-6213939.181029341, -356139.6129727494, -4553234.7863042625),
            glam::DVec3::new(646493.0548110288, -6565953.849297021, 5657224.674040573),
            glam::DVec3::new(-8310221.5498587, -11824484.096495166, 2477362.99531654),
            glam::DVec3::new(1442851.8614025046, 3944020.582333587, -8588097.243385006),
            glam::DVec3::new(8384083.9597701, 530705.873201302, -3658122.3308675354),
            glam::DVec3::new(2898130.522006808, -9580587.360908013, 2106248.00047252),
            glam::DVec3::new(2828141.7883684454, 406381.76914518373, 8963451.345762655),
            glam::DVec3::new(-8944812.308699876, 5682066.860649477, -11186750.260459974),
            glam::DVec3::new(-3464795.3083847975, 4481617.34349062, -8322123.193586318),
            glam::DVec3::new(5160745.024628614, -4957933.438608909, -7236716.489417127),
            glam::DVec3::new(-638024.4383018238, 7039962.2571193, -9115528.775503935),
            glam::DVec3::new(4758642.742106677, -6009149.6767303115, 10372387.808200765),
            glam::DVec3::new(5065559.227112598, -6668312.075854795, 3772818.7838625945),
            glam::DVec3::new(7679624.4540795535, 8806536.758325476, -704596.7317034043),
            glam::DVec3::new(8723624.845966201, 2447560.2355613476, 656383.0818598685),
            glam::DVec3::new(7912063.637985037, 5959948.937075286, 1732914.3346207177),
            glam::DVec3::new(544218.9974970771, 4542220.806800395, -1733919.4470265843),
            glam::DVec3::new(1273660.08066382, 12542690.992105417, 1025514.8480509501),
            glam::DVec3::new(6545395.75730307, 891176.5148664078, -742222.4719711064),
            glam::DVec3::new(-5198048.819521416, 11334531.338719131, -4327503.698459759),
            glam::DVec3::new(7082255.427083047, 2213827.8816959755, 4898931.340738286),
            glam::DVec3::new(-1642041.3457062577, -5232446.364934714, 977553.893898297),
            glam::DVec3::new(2180801.9347610455, 4950825.7295749895, 5525274.352594351),
            glam::DVec3::new(-2089105.6099524165, -7908981.413461881, 537723.1402286584),
            glam::DVec3::new(-8034727.96923753, 2404524.236063574, 4013552.0992562943),
            glam::DVec3::new(10780459.878230758, 3553258.2532735206, -5076247.431745518),
            glam::DVec3::new(1765741.0366824528, 3433493.892317093, -7201673.843672094),
            glam::DVec3::new(-8127704.602222981, -8581068.9141233, -418878.4728269605),
            glam::DVec3::new(188190.11501304168, 5618723.277314343, 7929564.744095197),
            glam::DVec3::new(-716453.8452096153, 5684941.331412287, -5153958.6238624565),
            glam::DVec3::new(5997804.751024207, 5142896.894642161, -2571615.2481815317),
            glam::DVec3::new(5766620.612909903, -3130671.465188509, -169145.9772780796),
            glam::DVec3::new(-1455502.2937485129, 9869173.857722817, 4904342.692741647),
            glam::DVec3::new(7435498.038448695, 3237763.4321128717, -2765632.3199936696),
            glam::DVec3::new(6506963.899765746, -890288.4638884775, 11209666.612828782),
            glam::DVec3::new(5697491.384008396, -3774239.402854111, -3892366.7913987376),
            glam::DVec3::new(-8076048.557717319, 3586898.887116836, -5989144.782130713),
            glam::DVec3::new(826761.0616518205, -2655410.2107246453, 4424343.6206899965),
            glam::DVec3::new(6011204.366063954, -2060480.221598107, -2183257.3166331653),
            glam::DVec3::new(4004336.8956559882, 5154200.514421189, -1494022.607362235),
            glam::DVec3::new(-2856049.652164667, -7499693.903157335, -3287403.620908837),
            glam::DVec3::new(2329930.4291646713, 4684682.772947516, -11964071.395589357),
            glam::DVec3::new(5054368.641846909, 4961083.740458888, -2909699.8362255576),
            glam::DVec3::new(-1087142.537981118, 10714673.301033907, -3411816.9755286165),
            glam::DVec3::new(-11190213.783338673, 3948581.5711958185, -2854530.496779622),
            glam::DVec3::new(-7300849.649118861, 8455065.135673758, 4211057.18533386),
            glam::DVec3::new(4985129.447896278, 8402484.675946508, 2044981.6104112952),
            glam::DVec3::new(-1207186.325554382, 8759529.854828134, -11485677.829091577),
            glam::DVec3::new(-5490298.462615342, 3405031.6482858593, 7051959.535886337),
        ];
        return (ecef_ref_tuple, enu);
    }

    #[rstest]
    fn test_ecef2enu(ecef2enu_fixture: (Vec<(glam::DVec3, glam::DVec3)>, Vec<glam::DVec3>)) {
        let ecef_ref_tuples = ecef2enu_fixture.0;
        let enu = ecef2enu_fixture.1;
        for (i, ecef_ref_tuple) in ecef_ref_tuples.iter().enumerate() {
            let actual_enu = ecef_uvw2enu(&ecef_ref_tuple.0, &ecef_ref_tuple.1);
            let expected_enu = enu.get(i).unwrap();
            assert_vecs_close(&actual_enu, expected_enu, 1e-6);
        }
    }
    #[rstest]
    fn test_enu2ecef(ecef2enu_fixture: (Vec<(glam::DVec3, glam::DVec3)>, Vec<glam::DVec3>)) {
        let ecef_ref_tuples = ecef2enu_fixture.0;
        let enu = ecef2enu_fixture.1;
        for (i, ecef_ref_tuple) in ecef_ref_tuples.iter().enumerate() {
            let actual_ecef = enu2ecef_uvw(enu.get(i).unwrap(), &ecef_ref_tuple.1);
            let expected_ecef = ecef_ref_tuple.0;
            assert_vecs_close(&actual_ecef, &expected_ecef, 1e-6);
        }
    }

    #[rstest]
    fn test_enu2heading() {
        {
            let actual_heading = enu2heading(&glam::DVec3::new(1000., 1000., 0.));
            let expected_heading = 45.;
            assert!(almost::equal_with(actual_heading, expected_heading, 1e-10));
        }
        {
            let actual_heading = enu2heading(&glam::DVec3::new(1000., -1000., 0.));
            let expected_heading = 135.;
            assert!(almost::equal_with(actual_heading, expected_heading, 1e-10));
        }
    }
}

use crate::{
    traits::{IntoDVec3, IntoLatLonTriple, IntoLatLonTuple},
    transforms::{enu, lla},
};
use glam;

/// Converts ECEF uvw Vector to NED.
/// This is a vector that is not in relation to an ECEF location
///
/// # Arguments
///
/// * `ecef_uvw` - Vector represented in ECEF coordinates [[meters]].
/// * `ll_ref` - Reference latitude-longitude [[degrees-degrees]].
///
/// # Returns
///
/// * `ned` - Vector represented in NED coordinates [[meters]].
pub fn ecef_uvw2ned(ecef_uvw: impl IntoDVec3, lla_ref: impl IntoLatLonTuple) -> glam::DVec3 {
    let ecef_uvw = ecef_uvw.into_dvec3();
    let lla_ref = lla_ref.into_lat_lon_tuple();

    let rot = ecef2ned_quat(lla_ref);
    return rot * ecef_uvw;
}
/// Converts Absolute ECEF to NED.
///
/// # Arguments
///
/// * `ecef` - Vector represented in ECEF coordinates [[meters]].
/// * `lla_ref` - Reference latitude-longitude-altitude [[degrees-degrees-meters]].
///
/// # Returns
///
/// * `ned` - Vector represented in NED coordinates [[meters]].
pub fn ecef2ned(ecef: impl IntoDVec3, lla_ref: impl IntoLatLonTriple) -> glam::DVec3 {
    let ecef = ecef.into_dvec3();
    let lla_ref = lla_ref.into_lat_lon_triple();

    let ecef_uvw = ecef - lla::lla2ecef(lla_ref);
    let rot = ecef2ned_quat(lla_ref);
    return rot * ecef_uvw;
}

/// Converts NED to ECEF uvw
///
/// # Arguments
///
/// * `ned` - Vector represented in NED coordinates [[meters]].
/// * `ll_ref` - Reference latitude-longitude [[degrees-degrees]].
///
/// # Returns
///
/// * `ecef_uvw` - Vector represented in ECEF frame. Not an absolute position [[meters]].
pub fn ned2ecef_uvw(ned: impl IntoDVec3, ll_ref: impl IntoLatLonTuple) -> glam::DVec3 {
    let ned = ned.into_dvec3();

    let rot = ned2ecef_quat(ll_ref);
    return rot * ned;
}
/// Converts NED to Absolute ECEF
///
/// # Arguments
///
/// * `ned` - Vector represented in NED coordinates [[meters]].
/// * `lla_ref` - Reference latitude-longitude-altitude [[degrees-degrees-meters]].
///
/// # Returns
///
/// * `ecef` - Absolute ECEF vector
pub fn ned2ecef(ned: impl IntoDVec3, lla_ref: impl IntoLatLonTriple) -> glam::DVec3 {
    let ned = ned.into_dvec3();
    let lla_ref = lla_ref.into_lat_lon_triple();

    let rot = ned2ecef_quat(lla_ref);
    let ecef_uvw = rot * ned;
    return ecef_uvw + lla::lla2ecef(lla_ref);
}

/// Calculates the direction cosine matrix that yields an NED to ECEF transformation at this LLA.
///
/// # Arguments
///
/// * `ll_deg` - Lat lon tuple [degrees-degrees]
///
/// # Returns
///
/// * `dcm` - NED to ECEF direction cosine matrix.
pub fn ned2ecef_dcm(ll_deg: impl IntoLatLonTuple) -> glam::DMat3 {
    let q = ned2ecef_quat(ll_deg);
    return glam::DMat3::from_quat(q);
}

/// Calculates the direction cosine matrix that yields an ECEF to NED transformation at this LLA.
///
/// # Arguments
///
/// * `ll_deg` - Lat lon tuple [degrees-degrees]
///
/// # Returns
///
/// * `dcm` - ECEF to NED direction cosine matrix.
pub fn ecef2ned_dcm(ll_deg: impl IntoLatLonTuple) -> glam::DMat3 {
    let q = ecef2ned_quat(ll_deg);
    return glam::DMat3::from_quat(q);
}
/// Calculates the quaternion that yields an NED to ECEF transformation at this LLA.
///
/// # Arguments
///
/// * `ll_deg` - Lat lon tuple [degrees-degrees]
///
/// # Returns
///
/// * `quat` - Normalized NED to ECEF quaternion.
pub fn ned2ecef_quat(ll_deg: impl IntoLatLonTuple) -> glam::DQuat {
    let enu2ecef_q = enu::enu2ecef_quat(ll_deg);
    let enu2ecef_mat = glam::DMat3::from_quat(enu2ecef_q);

    let east = enu2ecef_mat.x_axis;
    let north = enu2ecef_mat.y_axis;
    let up = enu2ecef_mat.z_axis;
    let down = -up;

    #[rustfmt::skip]
    let ned_mat = glam::DMat3::from_cols(north, east, down);
    return glam::DQuat::from_mat3(&ned_mat);
}

/// Calculates the quaternion that yields an ECEF to NED transformation at this LLA.
///
/// # Arguments
///
/// * `ll_deg` - Lat lon tuple [degrees-degrees]
///
/// # Returns
///
/// * `quat` - Normalized ECEF to NED quaternion.
pub fn ecef2ned_quat(ll_deg: impl IntoLatLonTuple) -> glam::DQuat {
    let ned2ecef_rot = ned2ecef_quat(ll_deg);
    return ned2ecef_rot.conjugate();
}

#[cfg(test)]
mod test_ned {
    use super::*;
    use rstest::*;

    #[fixture]
    fn ecef2ned_fixture() -> (Vec<(glam::DVec3, glam::DVec3)>, Vec<glam::DVec3>) {
        let ecef_ref_tuple: Vec<(glam::DVec3, glam::DVec3)> = vec![
            (
                glam::DVec3::new(6888437.030500963, 5159088.058806049, -1588568.383383099),
                glam::DVec3::new(-43.39498494726661, 4.058899692699072, 0.0),
            ),
            (
                glam::DVec3::new(5675971.780695453, -3933745.478421451, -468060.9169528838),
                glam::DVec3::new(15.008767101905619, 146.92063867032067, 0.0),
            ),
            (
                glam::DVec3::new(-4363243.112005923, 5116084.0831444785, 2367379.933506632),
                glam::DVec3::new(-44.90885855476071, 147.50865214856645, 0.0),
            ),
            (
                glam::DVec3::new(6204344.719931791, 8043319.008791654, -3797048.613613347),
                glam::DVec3::new(41.36971468682316, 143.58178366847767, 0.0),
            ),
            (
                glam::DVec3::new(-557145.6909457333, -7985975.838632684, -1316563.2909243256),
                glam::DVec3::new(19.959655219884283, 148.68397916564334, 0.0),
            ),
            (
                glam::DVec3::new(-459804.4689456597, 7306198.5554328, -4790153.792160811),
                glam::DVec3::new(54.905008862344026, 17.53174938081216, 0.0),
            ),
            (
                glam::DVec3::new(4394093.728079082, -2023529.1555146254, 6496899.54296466),
                glam::DVec3::new(30.26757622173315, -179.5885850468058, 0.0),
            ),
            (
                glam::DVec3::new(7352055.509855617, -5121782.462257361, -3495912.7450521984),
                glam::DVec3::new(66.68482177955784, -111.21584705913939, 0.0),
            ),
            (
                glam::DVec3::new(-5227681.427695597, 9350805.005802866, 6063589.3855974),
                glam::DVec3::new(-9.365477141597339, -151.03950532108723, 0.0),
            ),
            (
                glam::DVec3::new(158812.85041147843, 8656676.484538134, -7818843.081377927),
                glam::DVec3::new(9.228104296299222, 74.36210755208026, 0.0),
            ),
            (
                glam::DVec3::new(6289337.265826721, 805672.1394064799, 9276770.919476017),
                glam::DVec3::new(18.573413033048936, 31.542143103157088, 0.0),
            ),
            (
                glam::DVec3::new(1925737.2316621258, -2301977.0805467907, 1513020.2832977697),
                glam::DVec3::new(-37.74068956750355, -111.81912172043178, 0.0),
            ),
            (
                glam::DVec3::new(2255463.5973721338, 3133187.779792577, -469380.1598123852),
                glam::DVec3::new(-73.83161498479313, 92.73741190791725, 0.0),
            ),
            (
                glam::DVec3::new(8467620.318925612, 6849204.462803648, 7963462.42715758),
                glam::DVec3::new(76.15483916763182, 14.615972981299592, 0.0),
            ),
            (
                glam::DVec3::new(4105667.997088125, -4487317.573757457, 6232574.17015757),
                glam::DVec3::new(62.907473733546084, 142.21402827360305, 0.0),
            ),
            (
                glam::DVec3::new(8995297.46464241, 1593900.2149121184, -988737.8673768956),
                glam::DVec3::new(28.84416815203001, 178.65282216728616, 0.0),
            ),
            (
                glam::DVec3::new(5866501.682604484, -8352540.236067053, 2255662.100814244),
                glam::DVec3::new(-2.440043645549977, 46.85304254813022, 0.0),
            ),
            (
                glam::DVec3::new(-5139287.558762874, 4629784.415816955, -7657314.13582964),
                glam::DVec3::new(-50.317103363790864, 106.04986981580731, 0.0),
            ),
            (
                glam::DVec3::new(6318261.930673189, -7987849.595678076, -7072830.221753922),
                glam::DVec3::new(35.58071523442298, -163.71573556837956, 0.0),
            ),
            (
                glam::DVec3::new(8200320.293980793, 683959.3652144801, 3611782.65124513),
                glam::DVec3::new(-85.19457696080306, 48.59996756812498, 0.0),
            ),
            (
                glam::DVec3::new(1519058.9606308155, -2175811.8135434627, -2597201.19329625),
                glam::DVec3::new(86.49299711650835, -166.89886645986513, 0.0),
            ),
            (
                glam::DVec3::new(9220625.604792222, -6300561.172051234, -7522096.711511366),
                glam::DVec3::new(-52.096228220403646, 108.26877252750512, 0.0),
            ),
            (
                glam::DVec3::new(-9544348.486626834, -1487623.3606636561, -7969995.612516604),
                glam::DVec3::new(-43.21441983729025, -100.50146232612576, 0.0),
            ),
            (
                glam::DVec3::new(-2994120.6520693544, -6393641.969406243, 72730.10419774428),
                glam::DVec3::new(-82.91183272475539, -143.668353171972, 0.0),
            ),
            (
                glam::DVec3::new(-6012884.190658741, -2828893.973767963, 4631966.124507211),
                glam::DVec3::new(60.89878173481494, 150.6535423183193, 0.0),
            ),
            (
                glam::DVec3::new(3452811.2714610514, 9330978.060863663, -8838981.123470027),
                glam::DVec3::new(31.71632117388809, 124.35285373258189, 0.0),
            ),
            (
                glam::DVec3::new(-4986253.214297766, 1935827.8693882204, -1153719.326018421),
                glam::DVec3::new(-58.5324927987406, -10.21485056533632, 0.0),
            ),
            (
                glam::DVec3::new(1382254.7904856037, 172002.60125266388, -3771079.9799958635),
                glam::DVec3::new(-25.712697133752684, 121.55802277283243, 0.0),
            ),
            (
                glam::DVec3::new(1212004.377070479, -9751273.623413712, 4831487.548213271),
                glam::DVec3::new(-29.535020194777093, -163.54926231537002, 0.0),
            ),
            (
                glam::DVec3::new(-5197391.84347292, 9062586.796555977, -2955488.769689851),
                glam::DVec3::new(-38.181975325847986, -50.68756898865132, 0.0),
            ),
            (
                glam::DVec3::new(2674957.0449850517, 2421536.9123733453, 4312387.006029125),
                glam::DVec3::new(-20.156897643748977, -30.80952422019098, 0.0),
            ),
            (
                glam::DVec3::new(-9969515.562865596, -6153809.175106484, -3311966.1867499687),
                glam::DVec3::new(-46.90512716652745, 49.463784406548086, 0.0),
            ),
            (
                glam::DVec3::new(7508467.8342603445, 1363028.4182038382, -1711872.0663271137),
                glam::DVec3::new(-17.591926478565682, 72.65866461612316, 0.0),
            ),
            (
                glam::DVec3::new(3243917.7794763483, -9064406.280864034, -1092956.2056234032),
                glam::DVec3::new(-43.339153779499895, -123.2328340359681, 0.0),
            ),
            (
                glam::DVec3::new(-254687.97861935943, 1228098.5122885387, 5109695.345173651),
                glam::DVec3::new(69.09752776476617, -1.9502386648967445, 0.0),
            ),
            (
                glam::DVec3::new(-662155.2929495294, 6180917.147207247, 7500326.629605424),
                glam::DVec3::new(56.234687825476634, -112.31953414170192, 0.0),
            ),
            (
                glam::DVec3::new(2661775.1983660073, -8330658.996485413, 4511087.109226249),
                glam::DVec3::new(87.62786643692306, -35.345944003484306, 0.0),
            ),
            (
                glam::DVec3::new(-3676457.2555731535, -5729506.758706078, 4346482.866220744),
                glam::DVec3::new(-89.5756383505163, 116.18330779130967, 0.0),
            ),
            (
                glam::DVec3::new(-8044313.163986813, -7621922.104305084, 2985308.497923072),
                glam::DVec3::new(67.25768830206161, -79.20621240232589, 0.0),
            ),
            (
                glam::DVec3::new(-7996386.218725819, 7078762.191946764, -2066076.453381911),
                glam::DVec3::new(-75.35782498171784, -81.10301636906564, 0.0),
            ),
            (
                glam::DVec3::new(5846830.623713044, 7227198.072744723, -7331588.91594902),
                glam::DVec3::new(3.755795114555795, 54.281965733905416, 0.0),
            ),
            (
                glam::DVec3::new(7437276.714211722, -4431803.69567279, -9628513.449088097),
                glam::DVec3::new(-82.68061073845303, 65.15883724004757, 0.0),
            ),
            (
                glam::DVec3::new(8930051.083399918, 8768775.994698372, 8197023.54810205),
                glam::DVec3::new(-82.43918424587858, 89.6885364207107, 0.0),
            ),
            (
                glam::DVec3::new(3107237.2934945915, 4247153.050324835, 8054203.0123866135),
                glam::DVec3::new(25.225415962780346, -45.91826532998783, 0.0),
            ),
            (
                glam::DVec3::new(-5843117.926183505, 1742510.0939028692, -9822058.359018423),
                glam::DVec3::new(-62.81582870448219, -59.9729803092481, 0.0),
            ),
            (
                glam::DVec3::new(4369988.455430791, -3234880.5994664277, 2410762.1663310328),
                glam::DVec3::new(-82.58346908888234, -121.01020355679265, 0.0),
            ),
            (
                glam::DVec3::new(-4209382.927282661, -2104160.3402341865, 969685.9314502683),
                glam::DVec3::new(-37.18673973767942, -7.896719105632457, 0.0),
            ),
            (
                glam::DVec3::new(-9034872.754234113, -6408263.019168887, 461004.63400196284),
                glam::DVec3::new(-77.24468086301746, -34.859107279766334, 0.0),
            ),
            (
                glam::DVec3::new(-1705567.8205711525, -8011993.235225978, 8173151.087935612),
                glam::DVec3::new(-4.6791627952866435, 122.70539974596176, 0.0),
            ),
            (
                glam::DVec3::new(-3126968.1268446445, -418269.6169602778, 3991905.823012369),
                glam::DVec3::new(-13.22364176207492, -71.31487816103186, 0.0),
            ),
        ];
        let ned: Vec<glam::DVec3> = vec![
            glam::DVec3::new(3817222.48054082, 4658571.265151352, -6349553.538000638),
            glam::DVec3::new(1335560.479667212, 198199.46328847948, 6788746.964214704),
            glam::DVec3::new(6215046.4463947285, -1971463.5162006821, -2881531.5065581324),
            glam::DVec3::new(-2705751.452139706, -10155862.596903786, 2672791.8450811454),
            glam::DVec3::new(16941.438273651525, 7112108.151736968, 3903484.4614574388),
            glam::DVec3::new(-4196041.589283414, 7105335.751442247, 2906022.9995483346),
            glam::DVec3::new(7818661.838767107, 2055028.707839415, 507757.48807762563),
            glam::DVec3::new(-3325079.223590582, 8707243.769614134, 2373717.722279952),
            glam::DVec3::new(5990291.986467549, -10712799.23411021, 941094.0918214218),
            glam::DVec3::new(-9061359.615151051, 2180531.241850624, -7016738.689651038),
            glam::DVec3::new(6952053.224228817, -2603474.54049187, -8435296.427302884),
            glam::DVec3::new(2066450.2991287007, 2643374.4640652123, -197856.88363498566),
            glam::DVec3::new(2771666.85982101, -2402526.7977076243, -1292295.3383048982),
            glam::DVec3::new(-7728001.629512573, 4490844.479939028, -10106395.803587643),
            glam::DVec3::new(8174937.185333731, 1030751.23863034, -2818769.837445282),
            glam::DVec3::new(3454244.953396713, -1804943.7863353621, 8321289.964481875),
            glam::DVec3::new(2164974.005012828, -9992279.672711134, 2176239.0493952204),
            glam::DVec3::new(-371837.78150431626, 3658950.7480725506, -9641340.996500337),
            glam::DVec3::new(-3526752.99639549, 9439055.971421905, 7226156.516731704),
            glam::DVec3::new(6217719.1737010805, -5698837.223374292, 3101813.958870949),
            glam::DVec3::new(825608.3386346335, 2463504.6292084446, 2652671.676882206),
            glam::DVec3::new(-11622623.652663339, -6780805.413641624, -483987.5357060544),
            glam::DVec3::new(-3615814.874120512, -9113347.937575273, -7791098.351688512),
            glam::DVec3::new(6161623.468428223, 3376833.985513638, -692888.337145604),
            glam::DVec3::new(-1115448.4565121694, 5412718.962804998, -5922058.40516899),
            glam::DVec3::new(-10544501.6812377, -8115923.044843912, -248841.26316541713),
            glam::DVec3::new(-5080617.0732847275, 1020883.0769152861, 1756800.1702353738),
            glam::DVec3::new(-3647944.2084103096, -1267853.540811822, -1116381.595853447),
            glam::DVec3::new(4991934.067786062, 9695319.888885854, 990419.2709722328),
            glam::DVec3::new(-8693038.257175997, 1720354.0762511175, 6272937.350163244),
            glam::DVec3::new(4412558.882047785, 3449871.5464713047, 493587.2015961539),
            glam::DVec3::new(-10409383.191861024, 3577248.750890754, 5203629.587439036),
            glam::DVec3::new(-562179.2525078077, -6760908.88330678, -3890951.968104692),
            glam::DVec3::new(3188546.913593634, 7681057.538208658, -4971472.691921508),
            glam::DVec3::new(2099860.9212028678, 1218719.7314506723, -4667695.873833862),
            glam::DVec3::new(8712939.198567996, -2959883.4468468903, -3197008.9875583653),
            glam::DVec3::new(-6797822.957480535, -5255233.032677077, -4796557.332128665),
            glam::DVec3::new(-3487079.84736866, 5827317.746543763, 4372429.642127713),
            glam::DVec3::new(-4361519.839891313, -9329383.645262405, -5065222.912952845),
            glam::DVec3::new(-8485272.391840788, -6805383.920264025, 81490.94709643815),
            glam::DVec3::new(-7923794.00309282, -527825.6411444983, -8780939.872624164),
            glam::DVec3::new(-2116687.4840525235, -8610968.474886889, -9435735.063012568),
            glam::DVec3::new(9819083.582509633, -8882251.737492235, 6965604.174583564),
            glam::DVec3::new(7665132.586816506, 5186753.740633406, -2628069.0356712104),
            glam::DVec3::new(-8430208.39139448, -4186943.6469871406, -6712100.1414773455),
            glam::DVec3::new(827984.89422318, 5411990.781832159, 2323321.460391936),
            glam::DVec3::new(-1572839.8441506187, -2662525.2591259694, 3677474.7097341423),
            glam::DVec3::new(-3556595.327789179, -10422339.65689887, 1277791.7010225418),
            glam::DVec3::new(7671120.493029718, 5764204.045919783, 6467554.251532741),
            glam::DVec3::new(3747536.9393992727, -3096156.3708752114, 1502655.6004291987),
        ];
        return (ecef_ref_tuple, ned);
    }

    #[rstest]
    fn test_ecef2ned_dcm() {
        let actual = ecef2ned_dcm((0., 0.));
        let ned = actual * glam::DVec3::new(0.5, -1., 1.);
        assert!(ned.abs_diff_eq(glam::DVec3::new(1., -1., -0.5), 1e-6));
    }
    #[rstest]
    fn test_ecef2ned(ecef2ned_fixture: (Vec<(glam::DVec3, glam::DVec3)>, Vec<glam::DVec3>)) {
        let ecef_ref_tuples = ecef2ned_fixture.0;
        let neds = ecef2ned_fixture.1;
        for (i, ecef_ref_tuple) in ecef_ref_tuples.iter().enumerate() {
            let actual_ned = ecef_uvw2ned(&ecef_ref_tuple.0, &ecef_ref_tuple.1);
            let expected_ned = neds.get(i).unwrap();
            assert!(actual_ned.abs_diff_eq(*expected_ned, 1e-6));
        }
    }
    #[rstest]
    fn test_ned2ecef(ecef2ned_fixture: (Vec<(glam::DVec3, glam::DVec3)>, Vec<glam::DVec3>)) {
        let ecef_ref_tuples = ecef2ned_fixture.0;
        let neds = ecef2ned_fixture.1;
        for (i, ecef_ref_tuple) in ecef_ref_tuples.iter().enumerate() {
            let actual_ecef = ned2ecef_uvw(neds.get(i).unwrap(), &ecef_ref_tuple.1);
            let expected_ecef = ecef_ref_tuple.0;
            assert!(actual_ecef.abs_diff_eq(expected_ecef, 1e-6));
        }
    }
}

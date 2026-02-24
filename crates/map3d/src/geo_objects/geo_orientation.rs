use crate::{
    geo_objects::{
        geo_position::{EitherGeoPosOrLLATup, GeoPosition},
        geo_vector::GeoVector,
    },
    traits::IntoEitherLLATupOrGeoPos,
    transforms::*,
    DQuat, DVec3,
};
#[allow(unused_imports)]
#[cfg(feature = "bevy")]
use bevy::prelude::*;
#[cfg(feature = "pyo3")]
use either::Either;
#[cfg(not(feature = "pyo3"))]
use map3d_derive::*;
#[cfg(feature = "pyo3")]
use pyo3::{prelude::*, exceptions::PyValueError};
#[cfg(feature = "pyo3")]
use pyo3_stub_gen::derive::*;
#[cfg(feature = "py-bevy")]
use simple_py_bevy::*;
use std::ops::Mul;

#[derive(Clone, Copy, Default, PartialEq)]
#[cfg_attr(feature = "pyo3", gen_stub_pyclass, pyclass)]
#[cfg_attr(
    all(feature = "py-bevy", feature = "pyo3"),
    derive(PyBevyCompRef, PyStructRef)
)]
#[cfg_attr(feature ="serde", derive(serde::Deserialize, serde::Serialize))]
#[cfg_attr(
    feature = "bevy",
    derive(Component, Reflect),
    reflect(Component)
)]
#[cfg_attr(not(feature = "pyo3"), derive(DummyPyO3))]
pub struct GeoOrientation {
    #[
        cfg_attr(
            all(feature = "py-bevy", feature = "pyo3"), 
            py_bevy(
                get_ref = pyglam::DQuatRef, 
                other_set_type = pyglam::DQuatRef
            )
        )
    ]
    ecef_rot: DQuat,
}

/// Euler rotation sequences.
///
/// The three elemental rotations may be extrinsic (rotations about the axes xyz of the original
/// coordinate system, which is assumed to remain motionless), or intrinsic (rotations about the
/// axes of the rotating coordinate system XYZ, solidary with the moving body, which changes its
/// orientation after each elemental rotation).
///
/// Follows glam convention: https://docs.rs/glam/latest/glam/enum.EulerRot.html
///
/// euler_rot = i * j * k  #> EulerRot.XYZ given (i, j, k) euler angles
/// euler_Rot = k * j * i  #> EulerRot::XYZEx given (i, j, k) euler angles
///
#[cfg_attr(feature = "pyo3", gen_stub_pyclass_enum, pyclass(eq, eq_int))]
#[derive(PartialEq, Clone, Debug)]
pub enum EulerRot {
    /// Intrinsic three-axis rotation ZYX
    ZYX,
    /// Intrinsic three-axis rotation ZXY
    ZXY,
    /// Intrinsic three-axis rotation YXZ
    YXZ,
    /// Intrinsic three-axis rotation YZX
    YZX,
    /// Intrinsic three-axis rotation XYZ
    XYZ,
    /// Intrinsic three-axis rotation XZY
    XZY,

    /// Intrinsic two-axis rotation ZYZ
    ZYZ,
    /// Intrinsic two-axis rotation ZXZ
    ZXZ,
    /// Intrinsic two-axis rotation YXY
    YXY,
    /// Intrinsic two-axis rotation YZY
    YZY,
    /// Intrinsic two-axis rotation XYX
    XYX,
    /// Intrinsic two-axis rotation XZX
    XZX,

    /// Extrinsic three-axis rotation ZYX
    ZYXEx,
    /// Extrinsic three-axis rotation ZXY
    ZXYEx,
    /// Extrinsic three-axis rotation YXZ
    YXZEx,
    /// Extrinsic three-axis rotation YZX
    YZXEx,
    /// Extrinsic three-axis rotation XYZ
    XYZEx,
    /// Extrinsic three-axis rotation XZY
    XZYEx,

    /// Extrinsic two-axis rotation ZYZ
    ZYZEx,
    /// Extrinsic two-axis rotation ZXZ
    ZXZEx,
    /// Extrinsic two-axis rotation YXY
    YXYEx,
    /// Extrinsic two-axis rotation YZY
    YZYEx,
    /// Extrinsic two-axis rotation XYX
    XYXEx,
    /// Extrinsic two-axis rotation XZX
    XZXEx,
}
impl Into<glam::EulerRot> for EulerRot {
    fn into(self) -> glam::EulerRot {
        match self {
            EulerRot::XYZ => glam::EulerRot::XYZ,
            EulerRot::XZX => glam::EulerRot::XZX,
            EulerRot::XYX => glam::EulerRot::XYX,
            EulerRot::XZY => glam::EulerRot::XZY,
            EulerRot::XYXEx => glam::EulerRot::XYXEx,
            EulerRot::XYZEx => glam::EulerRot::XYZEx,
            EulerRot::XZXEx => glam::EulerRot::XZXEx,
            EulerRot::XZYEx => glam::EulerRot::XZYEx,
            EulerRot::YXY => glam::EulerRot::YXY,
            EulerRot::YXZ => glam::EulerRot::YXZ,
            EulerRot::YZX => glam::EulerRot::YZX,
            EulerRot::YZY => glam::EulerRot::YZY,
            EulerRot::YXYEx => glam::EulerRot::YXYEx,
            EulerRot::YXZEx => glam::EulerRot::YXZEx,
            EulerRot::YZXEx => glam::EulerRot::YZXEx,
            EulerRot::YZYEx => glam::EulerRot::YZYEx,
            EulerRot::ZXY => glam::EulerRot::ZXY,
            EulerRot::ZXZ => glam::EulerRot::ZXZ,
            EulerRot::ZXYEx => glam::EulerRot::ZXYEx,
            EulerRot::ZXZEx => glam::EulerRot::ZXZEx,
            EulerRot::ZYX => glam::EulerRot::ZYX,
            EulerRot::ZYXEx => glam::EulerRot::ZYXEx,
            EulerRot::ZYZ => glam::EulerRot::ZYZ,
            EulerRot::ZYZEx => glam::EulerRot::ZYZEx,
        }
    }
}

impl GeoOrientation {
    pub fn ecef(&self) -> &DQuat {
        return &self.ecef_rot;
    }
    pub fn ecef_mut(&mut self) -> &mut DQuat {
        return &mut self.ecef_rot;
    }

    /// Gets the directional cosine matrix of the body in the ecef frame
    pub fn dcm(&self) -> glam::DMat3 {
        glam::DMat3::from_quat(self.ecef_rot.into())
    }

    pub fn set_ecef(&mut self, body2ecef: &DQuat) {
        self.ecef_rot = body2ecef.clone()
    }
}

#[cfg_attr(
    all(feature = "py-bevy", feature = "pyo3"),
    py_bevy_methods,
    py_ref_methods
)]
#[cfg_attr(feature = "pyo3", gen_stub_pymethods, pymethods)]
impl GeoOrientation {

    /// Load from a json string
    /// ```
    /// {
    ///     "ecef_rot": [
    ///         1., 0., 0., 0.
    ///     ]
    /// }
    /// ```
    #[cfg(feature = "serde")]
    #[staticmethod]
    pub fn model_validate_json(json_str: &str) -> PyResult<Self> {
        match serde_json::from_str(json_str) {
            Ok(loaded) => Ok(loaded),
            Err(what) => Err(PyValueError::new_err(format!("{}", what)))
        }
    }

    /// Dump to a json string
    /// # Examples
    /// 
    /// ```
    /// {
    ///     "ecef_rot": [
    ///         1., 0., 0., 0.
    ///     ]
    /// }
    /// ```
    #[cfg(feature = "serde")]
    pub fn model_dump_json(&self) -> PyResult<String> {
        match serde_json::to_string(&self) {
            Ok(s) => Ok(s),
            Err(what) => Err(PyValueError::new_err(format!("{}", what)))
        }
    }

    /// Create an identity ecef orientation
    #[staticmethod]
    pub fn from_identity() -> Self {
        return Self {
            ecef_rot: DQuat::new(glam::DQuat::IDENTITY),
        };
    }
    /// Construct an orientation from a body2ecef quaternion
    ///
    /// This does not check that the input quaternion is normalized
    ///
    /// # Arguments
    ///
    /// - `body2ecef` (`&DQuat`) - Quaternion rotating a body coordinate frame into the ecef frame
    ///
    #[staticmethod]
    pub fn from_ecef(body2ecef: &DQuat) -> Self {
        Self {
            ecef_rot: body2ecef.clone(),
        }
    }
    /// Create a minimum rotation from one geo position to another
    ///
    /// # Arguments
    ///
    /// - `from_` (`&GeoPosition`) - Starting geo position
    /// - `to` (`&GeoPosition`) - Ending geo position
    ///
    #[staticmethod]
    pub fn from_geo_rotation_arc(from_: &GeoPosition, to: &GeoPosition) -> Self {
        // pyglam::dquat is currently feature gated behind python. it would be nice to call that directly
        let ecef_rot = glam::DQuat::from_rotation_arc(from_.ecef().into(), to.ecef().into());
        Self::from_ecef(&ecef_rot.into())
    }
    /// Create an orientation from an ECEF Axis and angle in radians
    ///
    /// # Arguments
    ///
    /// - `ecef_axis` (`&DVec3`) - ECEF unit vector
    /// - `angle` (`f64`) - Angle in radians
    ///
    #[staticmethod]
    pub fn from_axis_angle(ecef_axis: &DVec3, angle: f64) -> Self {
        let body2ecef = glam::DQuat::from_axis_angle(ecef_axis.into(), angle);
        Self::from_ecef(&body2ecef.into())
    }
    /// Construct an orientation from ecef euler angles from a given euler rotation sequence
    ///
    /// # Arguments
    ///
    /// - `ecef_rad` (`&DVec3`) - Euler angles in radians in ecef frame
    /// - `order` (`EulerRot`) - Euler angle rotation sequence. Defaults to a 3-2-1 sequence
    ///
    #[staticmethod]
    #[pyo3(signature = (ecef_rad, order = EulerRot::XYZEx))]
    pub fn from_ecef_euler(ecef_rad: &DVec3, order: EulerRot) -> Self {
        let ecef_rot = glam::DQuat::from_euler(order.into(), ecef_rad.x, ecef_rad.y, ecef_rad.z);
        Self::from_ecef(&ecef_rot.into())
    }
    /// Construct a GeoOrientation from a local ned coordinate frame
    /// ned is the euler radians around north, east, down in an euler sequence
    ///
    /// # Arguments
    ///
    /// - `ned_rad` (`&DVec3`) - NED euler angles in radians
    /// - `reference` (`tuple[float, float, float] | GeoPosition`) - Reference location
    /// - `order` (`EulerRot`) - Euler angle rotation sequence. Defaults to a 3-2-1 sequence
    ///
    #[staticmethod]
    #[pyo3(signature = (ned_rad, reference, order = EulerRot::XYZEx))]
    pub fn from_ned_euler(
        ned_rad: &DVec3,
        reference: EitherGeoPosOrLLATup,
        order: EulerRot,
    ) -> Self {
        let ned_quat = glam::DQuat::from_euler(order.into(), ned_rad.x, ned_rad.y, ned_rad.z);
        let body2ecef = ned2ecef_quat(reference) * ned_quat;
        Self::from_ecef(&body2ecef.into())
    }
    /// Construct a GeoOrientation from a local enu coordinate frame
    /// enu is the euler radians around east, north, up in an euler sequence
    ///
    /// # Arguments
    ///
    /// - `enu_rad` (`&DVec3`) - ENU euler angles in radians
    /// - `reference` (`tuple[float, float, float] | GeoPosition`) - Reference location euler angles are in relation to
    /// - `order` (`EulerRot`) - Euler angle rotation sequence. Defaults to a 3-2-1 sequence
    ///
    #[staticmethod]
    #[pyo3(signature = (enu_rad, reference, order = EulerRot::XYZEx))]
    pub fn from_enu_euler(
        enu_rad: &DVec3,
        reference: EitherGeoPosOrLLATup,
        order: EulerRot,
    ) -> Self {
        let enu_quat = glam::DQuat::from_euler(order.into(), enu_rad.x, enu_rad.y, enu_rad.z);
        let body2ecef = enu2ecef_quat(reference) * enu_quat;
        Self::from_ecef(&body2ecef.into())
    }
    /// Construct an orientation aligned with the ENU frame at the given reference location
    ///
    /// # Arguments
    ///
    /// - `reference` (`tuple[float, float, float] | GeoPosition`) - Reference geo position
    ///
    #[staticmethod]
    pub fn from_enu_frame(reference: EitherGeoPosOrLLATup) -> Self {
        let body2ecef = enu2ecef_quat(reference);
        Self::from_ecef(&body2ecef.into())
    }
    /// Construct an orientation aligned with the NED frame at the given reference location
    ///
    /// # Arguments
    ///
    /// - `reference` (`tuple[float, float, float] | GeoPosition`) - Reference geo position
    ///
    #[staticmethod]
    pub fn from_ned_frame(reference: EitherGeoPosOrLLATup) -> Self {
        let body2ecef = ned2ecef_quat(reference);
        Self::from_ecef(&body2ecef.into())
    }

    /// Convert this quaternion into euler angles around the ecef axis using the given rotation sequence
    ///
    /// # Arguments
    ///
    /// - `order` (`EulerRot`) - Rotation sequence for the resulting euler angles
    ///
    /// # Returns
    ///
    /// - `euler_angles` (f64, f64, f64) - x, y, z euler angle rotations in radians
    ///
    pub fn to_ecef_euler(&self, order: EulerRot) -> (f64, f64, f64) {
        self.dcm().to_euler(order.into())
    }

    /// Gets the heading direction (clockwise off north) in degrees for the body's forward vector
    ///
    /// # Arguments
    ///
    /// - `reference` (`tuple[float, float, float] | GeoPosition`) - Reference location to compute heading in relation to
    ///
    /// # Returns
    ///
    /// - `f64` - Heading angle in degrees
    ///
    pub fn heading(&self, reference: EitherGeoPosOrLLATup) -> f64 {
        return ecef_quat2heading(&self.ecef_rot.into(), reference);
    }

    /// Express this bodies orientation in a local enu frame
    ///
    /// # Arguments
    ///
    /// - `reference` (`tuple[float, float, float] | GeoPosition`) - Reference location
    ///
    /// # Returns
    ///
    /// - `DQuat` - body 2 local enu rotation
    ///
    pub fn as_enu(&self, reference: EitherGeoPosOrLLATup) -> DQuat {
        let ecef2enu = ecef2enu_quat(reference);
        return ecef2enu * self.ecef_rot;
    }

    /// Get the positive x axis for this orientation in the ecef frame
    pub fn x_axis(&self) -> DVec3 {
        self.dcm().col(0).into()
    }
    /// Get the positive y axis for this orientation in the ecef frame
    pub fn y_axis(&self) -> DVec3 {
        self.dcm().col(1).into()
    }
    /// Get the positive z axis for this orientation in the ecef frame
    pub fn z_axis(&self) -> DVec3 {
        self.dcm().col(2).into()
    }

    /// Multiply this orientation with either a GeoPosition or a GeoOrientation
    ///
    /// Multiplying two orientations together results in a combined rotation
    /// Multiplying a position and an orientation results in the transformation of that vector in the orientation's frame
    ///
    /// # Arguments
    ///
    /// - `rhs` (`Either<GeoVector, GeoOrientation>`) - Either a GeoVector to transform or a GeoOrientation
    ///
    /// # Returns
    ///
    /// - `PyResult<Either<GeoVector, GeoOrientation>>` - Either a transformed GeoVector or a combined GeoOrientation
    ///
    #[cfg(feature = "pyo3")]
    fn __mul__(
        &self,
        rhs: Either<GeoVector, GeoOrientation>,
    ) -> PyResult<Either<GeoVector, GeoOrientation>> {
        match rhs {
            Either::Left(vec) => Ok(Either::Left(self * vec)),
            Either::Right(rot) => Ok(Either::Right(self * rot)),
        }
    }
}

macro_rules! ops_with_geo_pos {
    ($a:ty, $b:ty) => {
        impl Mul<$a> for $b {
            type Output = GeoVector;
            fn mul(self, rhs: $a) -> Self::Output {
                let new_vel = self.ecef() * rhs.ecef_uvw();
                GeoVector::from_ecef(&new_vel, (0., 0., 0.).into_either())
            }
        }
    };
}
ops_with_geo_pos!(GeoVector, GeoOrientation);
ops_with_geo_pos!(&GeoVector, GeoOrientation);
ops_with_geo_pos!(GeoVector, &GeoOrientation);
ops_with_geo_pos!(&GeoVector, &GeoOrientation);

macro_rules! ops_with_self {
    ($a:ty, $b:ty) => {
        impl Mul<$a> for $b {
            type Output = GeoOrientation;
            fn mul(self, rhs: $a) -> Self::Output {
                let new_vel = self.ecef() * rhs.ecef();
                GeoOrientation::from_ecef(&new_vel)
            }
        }
    };
}
ops_with_self!(GeoOrientation, GeoOrientation);
ops_with_self!(&GeoOrientation, GeoOrientation);
ops_with_self!(GeoOrientation, &GeoOrientation);
ops_with_self!(&GeoOrientation, &GeoOrientation);

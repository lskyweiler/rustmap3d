use crate::{geo_objects::geo_position::GeoPosition, transforms::*};
use either::Either;
use pyglam;
use pyo3::prelude::*;
use pyo3_stub_gen::derive::*;
use std::ops::Mul;

#[derive(Clone)]
#[gen_stub_pyclass]
#[pyclass]
pub struct GeoOrientation {
    ecef_rot: pyglam::DQuat,
}

impl GeoOrientation {
    pub fn ecef(&self) -> &pyglam::DQuat {
        return &self.ecef_rot;
    }
    pub fn ecef_mut(&mut self) -> &mut pyglam::DQuat {
        return &mut self.ecef_rot;
    }

    /// Gets the directional cosine matrix of the body in the ecef frame
    pub fn dcm(&self) -> glam::DMat3 {
        glam::DMat3::from_quat(self.ecef_rot.into())
    }

    pub fn set_ecef(&mut self, body2ecef: &pyglam::DQuat) {
        self.ecef_rot = body2ecef.clone()
    }
}

#[gen_stub_pymethods]
#[pymethods]
impl GeoOrientation {
    /// Create an identity ecef orientation
    #[staticmethod]
    pub fn from_identity() -> Self {
        return Self {
            ecef_rot: pyglam::DQuat::new(glam::DQuat::IDENTITY),
        };
    }
    /// Construct an orientation from a body2ecef quaternion
    ///
    /// # Arguments
    ///
    /// - `body2ecef` (`&pyglam::DQuat`) - Quaternion rotating a body coordinate frame into the ecef frame
    ///
    #[staticmethod]
    pub fn from_ecef(body2ecef: &pyglam::DQuat) -> Self {
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
        let ecef_rot = pyglam::DQuat::from_rotation_arc(from_.ecef().into(), to.ecef().into());
        Self::from_ecef(&ecef_rot)
    }
    /// Construct an orientation from ecef euler angles
    ///
    /// # Arguments
    ///
    /// - `ecef_321` (`&pyglam`) - Euler angles in radians in ecef frame
    #[staticmethod]
    pub fn from_ecef_euler(ecef_321: &pyglam::DVec3) -> Self {
        let ecef_rot =
            glam::DQuat::from_euler(glam::EulerRot::XYZ, ecef_321.x, ecef_321.y, ecef_321.z);
        Self::from_ecef(&ecef_rot.into())
    }
    /// Construct a GeoOrientation from a local ned coordinate frame
    /// enu is the euler radians around north, east, down in a 3-2-1 sequence
    ///
    /// # Arguments
    ///
    /// - `ned_321` (`&pyglam`) - NED euler angles in radians
    /// - `reference_pos` (`&GeoPosition`) - Reference location euler angles are in relation to
    ///
    #[staticmethod]
    pub fn from_ned_euler(ned_321: &pyglam::DVec3, reference_pos: &GeoPosition) -> Self {
        let ref_lla = reference_pos.lla();
        let ned_quat =
            glam::DQuat::from_euler(glam::EulerRot::XYZ, ned_321.x, ned_321.y, ned_321.z);
        let body2ecef = ned2ecef_quat(ref_lla) * ned_quat;
        Self::from_ecef(&body2ecef.into())
    }
    /// Construct a GeoOrientation from a local enu coordinate frame
    /// enu is the euler radians around east, north, up in a 3-2-1 sequence
    ///
    /// # Arguments
    ///
    /// - `enu_321` (`&pyglam`) - ENU euler angles in radians
    /// - `reference_pos` (`&GeoPosition`) - Reference location euler angles are in relation to
    ///
    #[staticmethod]
    pub fn from_enu_euler(enu_321: &pyglam::DVec3, reference_pos: &GeoPosition) -> Self {
        let ref_lla = reference_pos.lla();
        let enu_quat =
            glam::DQuat::from_euler(glam::EulerRot::XYZ, enu_321.x, enu_321.y, enu_321.z);
        let body2ecef = enu2ecef_quat(ref_lla) * enu_quat;
        Self::from_ecef(&body2ecef.into())
    }

    /// Gets the heading direction (clockwise off north) in degrees for the body's forward vector
    ///
    /// # Arguments
    ///
    /// - `reference` (`&GeoPosition`) - Reference location to compute heading in relation to
    ///
    /// # Returns
    ///
    /// - `f64` - Heading angle in degrees
    ///
    pub fn heading(&self, reference: &GeoPosition) -> f64 {
        let lla = reference.lla();
        return ecef_quat2heading(&self.ecef_rot.into(), &glam::dvec2(lla.0, lla.1));
    }

    /// Express this bodies orientation in a local enu frame
    ///
    /// # Arguments
    ///
    /// - `reference` (`&GeoPosition`) - Refernce location
    ///
    /// # Returns
    ///
    /// - `pyglam::DQuat` - body 2 local enu rotation
    ///
    pub fn as_enu(&self, reference: &GeoPosition) -> pyglam::DQuat {
        let lla = reference.lla();
        let ecef2enu = ecef2enu_quat(lla);
        return ecef2enu * self.ecef_rot;
    }

    /// Get the forward (positive x axis) for this orientation in the ecef frame
    pub fn forward(&self) -> pyglam::DVec3 {
        self.dcm().col(0).into()
    }
    /// Get the left (positive y axis) for this orientation in the ecef frame
    pub fn left(&self) -> pyglam::DVec3 {
        self.dcm().col(1).into()
    }
    /// Get the up (positive z axis) for this orientation in the ecef frame
    pub fn up(&self) -> pyglam::DVec3 {
        self.dcm().col(2).into()
    }
    /// Get the right (negative y axis) for this orientation in the ecef frame
    pub fn right(&self) -> pyglam::DVec3 {
        (-self.dcm().col(1)).into()
    }
    /// Get the down (negative z axis) for this orientation in the ecef frame
    pub fn down(&self) -> pyglam::DVec3 {
        (-self.dcm().col(2)).into()
    }
    /// Get the back (negative x axis) for this orientation in the ecef frame
    pub fn back(&self) -> pyglam::DVec3 {
        (-self.dcm().col(0)).into()
    }

    fn __mul__(
        &self,
        rhs: Either<GeoPosition, GeoOrientation>,
    ) -> PyResult<Either<GeoPosition, GeoOrientation>> {
        match rhs {
            Either::Left(pos) => Ok(Either::Left(self * pos)),
            Either::Right(rot) => Ok(Either::Right(self * rot)),
        }
    }
}

macro_rules! ops_with_geo_pos {
    ($a:ty, $b:ty) => {
        impl Mul<$a> for $b {
            type Output = GeoPosition;
            fn mul(self, rhs: $a) -> Self::Output {
                let new_vel = self.ecef() * rhs.ecef();
                GeoPosition::from_ecef(&new_vel)
            }
        }
    };
}
ops_with_geo_pos!(GeoPosition, GeoOrientation);
ops_with_geo_pos!(&GeoPosition, GeoOrientation);
ops_with_geo_pos!(GeoPosition, &GeoOrientation);
ops_with_geo_pos!(&GeoPosition, &GeoOrientation);

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

use glam;
#[cfg(feature = "pyo3")]
use pyo3::{prelude::*, types::PyDict};
use rand_distr::{Distribution, Normal};

/// Linear interpolation between two values.
///
/// # Arguments
///
/// * `x` - Start value.
/// * `y` - End value.
/// * `a` - Amount / factor between `x` and `y`, usually between 0 and 1.
///
/// # Returns
///
/// - `value` - Value between `x` and `y` based on a factor `a`.
pub fn lerp(x: f64, y: f64, a: f64) -> f64 {
    return (y - x) * a + x;
}

pub fn quat_forward(q: &glam::DQuat) -> glam::DVec3 {
    return glam::DMat3::from_quat(*q).x_axis;
}
pub fn quat_left(q: &glam::DQuat) -> glam::DVec3 {
    return glam::DMat3::from_quat(*q).y_axis;
}
pub fn quat_up(q: &glam::DQuat) -> glam::DVec3 {
    return glam::DMat3::from_quat(*q).z_axis;
}

/// Generates a random normalized quaternion.
///
/// # Returns
///
/// * `quat` - Random normalized quaternion.
pub fn rand_orientation() -> glam::DQuat {
    return glam::DQuat::from_xyzw(
        rand::random(),
        rand::random(),
        rand::random(),
        rand::random(),
    )
    .normalize();
}

/// Wraps an angle to the domains [[-pi, pi]].
///
/// # Arguments
///
/// * `angle` - Angle [[radians]].
///
/// # Returns
///
/// - `angle` - Angle guaranteed to be in domain [[-pi, pi]] [[radians]].
pub fn wrap_to_pi(angle: f64) -> f64 {
    let mut x = angle % std::f64::consts::TAU;

    if x <= -std::f64::consts::PI {
        x += std::f64::consts::TAU;
    } else if x > std::f64::consts::PI {
        x -= std::f64::consts::TAU;
    }

    x
}

/// Wraps an angle in degrees to be between [0, 360]
///
/// # Arguments
///
/// - `angle_d` (`f64`) - angle in degrees
///
/// # Returns
///
/// - `f64` - 0. <= angle <= 360.
pub fn wrap_to_0_360(angle_d: f64) -> f64 {
    let mut wrapped = angle_d % 360.0;
    if wrapped < 0.0 {
        wrapped += 360.0;
    }
    wrapped
}

/// Generates a uniform random point on the surface of a sphere.
///
/// # Arguments
///
/// * `radius` - Radius of sphere.
///
/// # Returns
///
/// * `vector` - Random point in R3.
pub fn rand_point_on_sphere(radius: f64) -> glam::DVec3 {
    let mut rng = rand::rng();
    let normal = Normal::new(0.0, 1.0).unwrap();

    return radius
        * (glam::DVec3::new(
            normal.sample(&mut rng),
            normal.sample(&mut rng),
            normal.sample(&mut rng),
        )
        .normalize());
}

/// Convert a json string to a python dict
///
/// Simply calls json.loads(json_str)
///
#[cfg(feature = "pyo3")]
pub fn dump_to_pydict<'py>(py: Python<'py>, json_str: &str) -> PyResult<Py<PyAny>> {
    let json_mod = py.import("json")?;
    let get_module_callable = json_mod.getattr("loads")?;
    let loaded = get_module_callable.call1((json_str,))?;
    Ok(loaded.unbind())
}
/// Convert a json dict to a string
///
/// Simply calls json.dumps(json_dict)
///
#[cfg(feature = "pyo3")]
pub fn pydict_to_dump<'py>(py: Python<'py>, json_dict: Py<PyDict>) -> PyResult<String> {
    let json_mod = py.import("json")?;
    let get_module_callable = json_mod.getattr("dumps")?;
    let dumped = get_module_callable.call1((json_dict,))?;
    let json_str: String = dumped.extract()?;
    Ok(json_str)
}

#[cfg(all(feature = "pydantic-serde", feature = "pyo3"))]
pub fn create_pydantic_core_schema<'py>(
    py: Python,
    validator_py_fn: Py<PyAny>,
    serializer_py_fn: Py<PyAny>,
    json_schema_input_schema: Py<PyAny>,
) -> PyResult<Py<PyAny>> {
    use pyo3::types::IntoPyDict;
    use std::collections::HashMap;

    /*
    @classmethod
    def __get_pydantic_core_schema__(
        cls, _source: type[typing.Any], _handler: pydantic.GetCoreSchemaHandler
    ) -> CoreSchema:
        """
        Generates the pydantic-core schema for this class.
        """

        # Define the schema: use a plain validator function
        schema = core_schema.no_info_plain_validator_function(
            GeoPosition.model_validate,
            json_schema_input_schema=core_schema.model_schema(
                GeoPosition,
                core_schema.model_fields_schema(
                    fields={
                        "ecef": core_schema.model_field(
                            core_schema.tuple_schema(
                                [
                                    core_schema.model_field(
                                        schema=core_schema.float_schema()
                                    ),
                                    core_schema.model_field(
                                        schema=core_schema.float_schema()
                                    ),
                                    core_schema.model_field(
                                        schema=core_schema.float_schema()
                                    ),
                                ],
                                ref="DVec3"
                            ),
                        )
                    },
                ),
            ),
            serialization=core_schema.plain_serializer_function_ser_schema(
                lambda v: {"ecef": [v.ecef.x, v.ecef.y, v.ecef.z]},
                when_used="always",
            ),
        )
        return schema
    */

    let pydantic_core_mod = py.import("pydantic_core")?;
    let core_schema_mod = pydantic_core_mod.getattr("core_schema")?;
    let schema_fn = core_schema_mod.getattr("no_info_plain_validator_function")?;
    let serialization_fn = core_schema_mod.getattr("plain_serializer_function_ser_schema")?;

    let ser_kwargs = HashMap::from([("when_used", "always")]).into_py_dict(py)?;
    let serialization = serialization_fn.call((serializer_py_fn,), Some(&ser_kwargs))?;

    let schema_kwargs = HashMap::from([
        ("serialization", serialization),
        (
            "json_schema_input_schema",
            json_schema_input_schema.bind(py).to_owned(),
        ),
    ])
    .into_py_dict(py)?;
    let schema = schema_fn.call((validator_py_fn,), Some(&schema_kwargs))?;

    Ok(schema.unbind())
}

#[cfg(all(feature = "pydantic-serde", feature = "pyo3"))]
pub fn create_pydantic_model_config<'py>(py: Python<'py>) -> PyResult<Py<PyAny>> {
    let pydantic_mod = py.import("pydantic")?;
    let model_config_type = pydantic_mod.getattr("ConfigDict")?;

    let out = model_config_type.call0()?;
    Ok(out.unbind())
}

pub struct Pydantic<'py> {
    pub pydantic_core_mod: Bound<'py, PyModule>,
    pub core_schema_mod: Bound<'py, PyAny>,
    pub tuple_schema_fn: Bound<'py, PyAny>,
    pub model_schema_fn: Bound<'py, PyAny>,
    pub model_field_fn: Bound<'py, PyAny>,
    pub model_fields_schema_fn: Bound<'py, PyAny>,
    pub float_schema_fn: Bound<'py, PyAny>,
}
impl<'py> Pydantic<'py> {
    pub fn new(py: Python<'py>) -> PyResult<Self> {
        let pydantic_core_mod = py.import("pydantic_core")?;
        let core_schema_mod = pydantic_core_mod.getattr("core_schema")?;

        Ok(Self {
            pydantic_core_mod: pydantic_core_mod.clone(),
            core_schema_mod: core_schema_mod.clone(),
            tuple_schema_fn: core_schema_mod.getattr("tuple_schema")?,
            model_schema_fn: core_schema_mod.getattr("model_schema")?,
            model_field_fn: core_schema_mod.getattr("model_field")?,
            model_fields_schema_fn: core_schema_mod.getattr("model_fields_schema")?,
            float_schema_fn: core_schema_mod.getattr("float_schema")?,
        })
    }
}

#[cfg(all(feature = "pydantic-serde", feature = "pyo3"))]
pub fn create_pydantic_schema_description<'py>(
    py: Python,
    description: &str,
) -> PyResult<(String, Py<PyAny>)> {
    use pyo3::IntoPyObjectExt;
    use std::collections::HashMap;
    let description = HashMap::from([("description", description)]).into_py_any(py)?;
    let js_updates = HashMap::from([(("pydantic_js_updates", description))]).into_py_any(py)?;
    Ok(("metadata".to_string(), js_updates))
}

#[cfg(all(feature = "pydantic-serde", feature = "pyo3"))]
pub fn create_dvec3_schema<'py>(py: Python<'py>) -> PyResult<Py<PyAny>> {
    /*
    core_schema.tuple_schema(
        [
            core_schema.model_field(
                schema=core_schema.float_schema()
            ),
            core_schema.model_field(
                schema=core_schema.float_schema()
            ),
            core_schema.model_field(
                schema=core_schema.float_schema()
            ),
        ],
        ref="DVec3",
        metadata={"pydantic_js_updates": {"description": "fjdkl"}}
    ),
    */

    use pyo3::types::{IntoPyDict, PyString};
    use std::collections::HashMap;

    let pydantic = Pydantic::new(py)?;

    let float_schema = pydantic.float_schema_fn.call0()?;
    let float_field = pydantic.model_field_fn.call1((float_schema,))?.unbind();
    let fields = vec![
        float_field.clone_ref(py),
        float_field.clone_ref(py),
        float_field.clone_ref(py),
    ];

    let description = create_pydantic_schema_description(py, "3 component Vector of floats XYZ")?;
    let ref_ = PyString::new(py, "DVec3").unbind().as_any().clone_ref(py);

    let kwargs = HashMap::from([("ref".to_string(), ref_), description]).into_py_dict(py)?;
    let schema = pydantic.tuple_schema_fn.call((fields,), Some(&kwargs))?;

    Ok(schema.unbind())
}
#[cfg(all(feature = "pydantic-serde", feature = "pyo3"))]
pub fn create_lat_lon_tuple_schema<'py>(py: Python<'py>) -> PyResult<Py<PyAny>> {
    use pyo3::types::{IntoPyDict, PyString};
    use std::collections::HashMap;

    let pydantic = Pydantic::new(py)?;

    let float_schema = pydantic.float_schema_fn.call0()?;
    let float_field = pydantic.model_field_fn.call1((float_schema,))?.unbind();
    let fields = vec![
        float_field.clone_ref(py),
        float_field.clone_ref(py),
        float_field.clone_ref(py),
    ];

    let ref_ = PyString::new(py, "LLA").unbind().as_any().clone_ref(py);

    let desc = create_pydantic_schema_description(
        py,
        "3 component Tuple of Latitude, Longitude, Altitude in Meters/Degrees",
    )?;
    let kwargs = HashMap::from([("ref".to_string(), ref_), desc]).into_py_dict(py)?;
    let schema = pydantic.tuple_schema_fn.call((fields,), Some(&kwargs))?;

    Ok(schema.unbind())
}
#[cfg(all(feature = "pydantic-serde", feature = "pyo3"))]
pub fn create_dquat_schema<'py>(py: Python<'py>) -> PyResult<Py<PyAny>> {
    use pyo3::{
        types::{IntoPyDict, PyString},
        IntoPyObjectExt,
    };
    use std::collections::HashMap;
    let pydantic = Pydantic::new(py)?;

    let float_schema = pydantic.float_schema_fn.call0()?;
    let float_field = pydantic.model_field_fn.call1((float_schema,))?.unbind();
    let fields = vec![
        float_field.clone_ref(py),
        float_field.clone_ref(py),
        float_field.clone_ref(py),
        float_field.clone_ref(py),
    ];

    let description = HashMap::from([("description", "4 component Quaternion of floats WXYZ")])
        .into_py_any(py)?;
    let ref_ = PyString::new(py, "DQuat").unbind().as_any().clone_ref(py);

    let js_updates = HashMap::from([(("pydantic_js_updates", description))]).into_py_any(py)?;
    let kwargs = HashMap::from([("ref", ref_), ("metadata", js_updates)]).into_py_dict(py)?;
    let schema = pydantic.tuple_schema_fn.call((fields,), Some(&kwargs))?;

    Ok(schema.unbind())
}

/// Generate a wrapper function around the given function that allows pydantic to call as a non-class method validator
#[cfg(all(feature = "pydantic-serde", feature = "pyo3"))]
#[macro_export]
macro_rules! validator_wrapper_fn {
    ($Obj:ident, $Name:literal) => {
        #[cfg(all(feature = "pydantic-serde", feature = "pyo3"))]
        #[pyfunction]
        fn validate_obj<'py>(py: Python<'py>, obj: Py<PyAny>) -> PyResult<Py<PyAny>> {
            use pyo3::IntoPyObjectExt;

            if let Ok(dict) = obj.extract::<Py<PyDict>>(py) {
                let validated = $Obj::model_validate(py, dict)?;
                let validated_any = validated.into_py_any(py)?;
                return Ok(validated_any);
            } else if let Ok(validated) = obj.extract::<$Obj>(py) {
                let validated_any = validated.into_py_any(py)?;
                return Ok(validated_any);
            }
            let obj_type: Py<PyString> = obj
                .getattr(py, "__class__")?
                .getattr(py, "__name__")?
                .extract(py)?;
            Err(PyValueError::new_err(format!(
                "Must either be a $Name object or dict. Got {}",
                obj_type
            )))
        }
    };
}

/// Test utility to assert that two vector's components are close to equal
/// Panics if not close to equal.
///
/// # Arguments
///
/// * `a` - Vector A
/// * `b` - Vector B
/// * `tol` - Absolute tolerance
#[deprecated(
    since = "0.2.0",
    note = "assert!(glam::DVec3::abs_diff_eq(...)) instead"
)]
pub fn assert_vecs_close(a: &glam::DVec3, b: &glam::DVec3, tol: f64) {
    assert!(almost::equal_with(a.x, b.x, tol));
    assert!(almost::equal_with(a.y, b.y, tol));
    assert!(almost::equal_with(a.z, b.z, tol));
}

#[cfg(test)]
mod util_tests {
    use super::*;
    use approx::relative_eq;
    use rstest::*;
    use std::f64::consts::PI;

    #[rstest]
    #[case(-4.0*PI, 0.0)]
    #[case(-1.2*PI, 0.8*PI)]
    #[case(-0.5*PI, -0.5*PI)]
    #[case(0.0, 0.0)]
    #[case(0.6*PI, 0.6*PI)]
    #[case(1.6*PI, -0.4*PI)]
    fn test_wrap_to_pi(#[case] angle: f64, #[case] truth: f64) {
        let test = wrap_to_pi(angle);
        assert!(relative_eq!(
            test,
            truth,
            max_relative = 1e-13,
            epsilon = 1e-14
        ));
    }

    #[rstest]
    #[case(90., 90.)]
    #[case(720., 0.)]
    #[case(450., 90.)]
    #[case(-90., 270.)]
    fn test_wrap_to_0_360(#[case] angle: f64, #[case] truth: f64) {
        let test = wrap_to_0_360(angle);
        assert!(relative_eq!(
            test,
            truth,
            max_relative = 1e-13,
            epsilon = 1e-14
        ));
    }
}

//! Shared driver for the batched transforms.
//!
//! Every `*_batch` / `*_batch_into` function does its per-batch setup once (rotations, reference
//! ECEF position, parsed time), then applies the same per-point kernel as its single-point
//! counterpart to each input, so the results match that function bit for bit.

/// Applies `kernel` to each input, writing the results to `out`.
///
/// This is the only place the batched transforms iterate, so it is where parallel iteration would
/// go.
///
/// # Panics
///
/// If `inputs` and `out` differ in length.
pub(crate) fn map_into(
    inputs: &[glam::DVec3],
    out: &mut [glam::DVec3],
    kernel: impl Fn(glam::DVec3) -> glam::DVec3,
) {
    assert_eq!(
        inputs.len(),
        out.len(),
        "batch output length must match input length"
    );
    for (o, &v) in out.iter_mut().zip(inputs) {
        *o = kernel(v);
    }
}

/// Applies `kernel` to each input, returning the results in a new `Vec`.
pub(crate) fn map(
    inputs: &[glam::DVec3],
    kernel: impl Fn(glam::DVec3) -> glam::DVec3,
) -> Vec<glam::DVec3> {
    let mut out = vec![glam::DVec3::ZERO; inputs.len()];
    map_into(inputs, &mut out, kernel);
    return out;
}

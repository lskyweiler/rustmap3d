# map3d_derive

This is here to allow us to use pyo3 attributes either when compiling with pyo3 or not

```rust
// this doens't work
#[cfg_attr(feature = "pyo3", setter)]
fn foo(): ....

// so mock the pyo3 to make it work with feature gating
#[setter]
fn foo(): ....
```

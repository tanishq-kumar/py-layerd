use pyo3::prelude::*;

mod layout_ext;

#[pymodule]
fn _core(m: &Bound<'_, PyModule>) -> PyResult<()> {
    layout_ext::register(m)?;
    Ok(())
}

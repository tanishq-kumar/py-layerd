mod force;
mod layout_ext;
mod options;

use pyo3::prelude::*;

#[pymodule]
fn _core(m: &Bound<'_, PyModule>) -> PyResult<()> {
    layout_ext::register(m)?;
    options::register(m)?;
    Ok(())
}

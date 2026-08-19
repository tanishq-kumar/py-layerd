use pyo3::prelude::*;
use pyo3::exceptions::PyValueError;

use ffi_types::{layout_flat, FfiError, MAX_EDGE_COUNT, MAX_NODE_COUNT};

#[pyclass]
#[derive(Clone, Debug)]
pub struct NodeSpec {
    #[pyo3(get, set)]
    pub id: u32,
    #[pyo3(get, set)]
    pub width: f64,
    #[pyo3(get, set)]
    pub height: f64,
}

#[pymethods]
impl NodeSpec {
    #[new]
    fn new(id: u32, width: f64, height: f64) -> Self {
        Self { id, width, height }
    }
}

#[pyclass]
#[derive(Clone, Debug)]
pub struct EdgeSpec {
    #[pyo3(get, set)]
    pub id: u32,
    #[pyo3(get, set)]
    pub source: u32,
    #[pyo3(get, set)]
    pub target: u32,
}

#[pymethods]
impl EdgeSpec {
    #[new]
    fn new(id: u32, source: u32, target: u32) -> Self {
        Self { id, source, target }
    }
}

#[pyclass]
#[derive(Clone, Debug)]
pub struct LayoutResult {
    #[pyo3(get)]
    pub width: f64,
    #[pyo3(get)]
    pub height: f64,
    #[pyo3(get)]
    pub node_ids: Vec<u32>,
    #[pyo3(get)]
    pub node_x: Vec<f64>,
    #[pyo3(get)]
    pub node_y: Vec<f64>,
    #[pyo3(get)]
    pub node_width: Vec<f64>,
    #[pyo3(get)]
    pub node_height: Vec<f64>,
    #[pyo3(get)]
    pub edge_ids: Vec<u32>,
    #[pyo3(get)]
    pub edge_source: Vec<u32>,
    #[pyo3(get)]
    pub edge_target: Vec<u32>,
    #[pyo3(get)]
    pub edge_bend_start: Vec<u32>,
    #[pyo3(get)]
    pub edge_bend_length: Vec<u32>,
    #[pyo3(get)]
    pub bend_x: Vec<f64>,
    #[pyo3(get)]
    pub bend_y: Vec<f64>,
}

fn ffi_err_to_py(err: FfiError) -> PyErr {
    PyValueError::new_err(err.to_string())
}

fn build_lrd1(nodes: Vec<NodeSpec>, edges: Vec<EdgeSpec>) -> Result<Vec<u8>, PyErr> {
    if nodes.len() > MAX_NODE_COUNT as usize {
        return Err(PyValueError::new_err(format!(
            "node count {} exceeds MAX_NODE_COUNT {}",
            nodes.len(),
            MAX_NODE_COUNT
        )));
    }
    if edges.len() > MAX_EDGE_COUNT as usize {
        return Err(PyValueError::new_err(format!(
            "edge count {} exceeds MAX_EDGE_COUNT {}",
            edges.len(),
            MAX_EDGE_COUNT
        )));
    }
    for n in &nodes {
        if !n.width.is_finite() || !n.height.is_finite() {
            return Err(PyValueError::new_err(format!(
                "node {} has non-finite width/height",
                n.id
            )));
        }
        if n.width <= 0.0 || n.height <= 0.0 {
            return Err(PyValueError::new_err(format!(
                "node {} has non-positive size {}x{}",
                n.id, n.width, n.height
            )));
        }
    }

    let mut buf = Vec::with_capacity(
        ffi_types::HEADER_SIZE
            + nodes.len() * ffi_types::NODE_RECORD_SIZE
            + edges.len() * ffi_types::EDGE_RECORD_SIZE,
    );

    buf.extend_from_slice(ffi_types::MAGIC);
    buf.extend_from_slice(&ffi_types::VERSION.to_le_bytes());
    buf.extend_from_slice(&0u32.to_le_bytes()); // flags
    buf.extend_from_slice(&(nodes.len() as u32).to_le_bytes());
    buf.extend_from_slice(&(edges.len() as u32).to_le_bytes());
    buf.extend_from_slice(&0u32.to_le_bytes()); // bend_count (input)
    buf.extend_from_slice(&0f64.to_le_bytes()); // graph_width
    buf.extend_from_slice(&0f64.to_le_bytes()); // graph_height

    for n in &nodes {
        buf.extend_from_slice(&n.id.to_le_bytes());
        buf.extend_from_slice(&0u32.to_le_bytes()); // reserved
        buf.extend_from_slice(&n.width.to_le_bytes());
        buf.extend_from_slice(&n.height.to_le_bytes());
        buf.extend_from_slice(&0f64.to_le_bytes()); // x
        buf.extend_from_slice(&0f64.to_le_bytes()); // y
    }

    for e in &edges {
        buf.extend_from_slice(&e.id.to_le_bytes());
        buf.extend_from_slice(&e.source.to_le_bytes());
        buf.extend_from_slice(&e.target.to_le_bytes());
        buf.extend_from_slice(&0u32.to_le_bytes()); // bend_start
        buf.extend_from_slice(&0u32.to_le_bytes()); // bend_length
        buf.extend_from_slice(&0u32.to_le_bytes()); // reserved
    }

    Ok(buf)
}

#[pyfunction]
fn layout_flat_py(nodes: Vec<NodeSpec>, edges: Vec<EdgeSpec>) -> PyResult<LayoutResult> {
    let buf = build_lrd1(nodes, edges)?;
    let out = layout_flat(&buf).map_err(ffi_err_to_py)?;
    Ok(LayoutResult {
        width: out.width,
        height: out.height,
        node_ids: out.node_wire_ids,
        node_x: out.node_x,
        node_y: out.node_y,
        node_width: out.node_width,
        node_height: out.node_height,
        edge_ids: out.edge_wire_ids,
        edge_source: out.edge_source_wire_ids,
        edge_target: out.edge_target_wire_ids,
        edge_bend_start: out.edge_bend_start,
        edge_bend_length: out.edge_bend_length,
        bend_x: out.bend_x,
        bend_y: out.bend_y,
    })
}

#[pyfunction]
fn layout_bytes_py(nodes: Vec<NodeSpec>, edges: Vec<EdgeSpec>) -> PyResult<Vec<u8>> {
    let buf = build_lrd1(nodes, edges)?;
    ffi_types::layout_bytes(&buf).map_err(ffi_err_to_py)
}

pub fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<NodeSpec>()?;
    m.add_class::<EdgeSpec>()?;
    m.add_class::<LayoutResult>()?;
    m.add_function(wrap_pyfunction!(layout_flat_py, m)?)?;
    m.add_function(wrap_pyfunction!(layout_bytes_py, m)?)?;
    Ok(())
}

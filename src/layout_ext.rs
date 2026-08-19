use layerd::graph::LGraph;
use layerd::math::Vec2;
use layerd::options::LayoutOptions;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

use crate::force::force_layout;
use crate::options::PyLayoutOptions;
use ffi_types::{FfiError, MAX_EDGE_COUNT, MAX_NODE_COUNT};

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

fn build_graph(
    nodes: &[NodeSpec],
    edges: &[EdgeSpec],
    options: Option<LayoutOptions>,
) -> Result<(LGraph, Vec<u32>, Vec<u32>, Vec<u32>, Vec<u32>, Vec<u32>), PyErr> {
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
    for n in nodes {
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

    let mut graph = LGraph::new();
    if let Some(opts) = options {
        graph.options = opts;
        graph.reseed_from_options();
    }

    let mut node_ids: Vec<u32> = Vec::with_capacity(nodes.len());
    let mut arena_ids = Vec::with_capacity(nodes.len());
    let mut ports: Vec<(layerd::graph::index::PortId, layerd::graph::index::PortId)> =
        Vec::with_capacity(nodes.len());
    let mut caller_to_idx: std::collections::HashMap<u32, usize> = std::collections::HashMap::new();

    for (i, n) in nodes.iter().enumerate() {
        if caller_to_idx.insert(n.id, i).is_some() {
            return Err(PyValueError::new_err(format!("duplicate node id: {}", n.id)));
        }
        let nid = graph.add_node(Vec2::new(n.width, n.height));
        let west = graph.add_port(nid, layerd::graph::port::PortSide::West);
        let east = graph.add_port(nid, layerd::graph::port::PortSide::East);
        node_ids.push(n.id);
        arena_ids.push(nid);
        ports.push((west, east));
    }

    let mut edge_ids: Vec<u32> = Vec::with_capacity(edges.len());
    let mut edge_sources: Vec<u32> = Vec::with_capacity(edges.len());
    let mut edge_targets: Vec<u32> = Vec::with_capacity(edges.len());
    let mut seen: std::collections::HashSet<u32> = std::collections::HashSet::new();
    let mut arena_edge_ids = Vec::with_capacity(edges.len());

    for e in edges {
        if !seen.insert(e.id) {
            return Err(PyValueError::new_err(format!("duplicate edge id: {}", e.id)));
        }
        let si = *caller_to_idx.get(&e.source).ok_or_else(|| {
            PyValueError::new_err(format!("edge {} references unknown node {}", e.id, e.source))
        })?;
        let ti = *caller_to_idx.get(&e.target).ok_or_else(|| {
            PyValueError::new_err(format!("edge {} references unknown node {}", e.id, e.target))
        })?;
        let eid = graph.add_edge(ports[si].1, ports[ti].0);
        edge_ids.push(e.id);
        edge_sources.push(e.source);
        edge_targets.push(e.target);
        arena_edge_ids.push(eid);
    }

    Ok((graph, node_ids, edge_ids, edge_sources, edge_targets, arena_edge_ids.iter().map(|_| 0).collect()))
}

fn run_layout_and_collect(
    mut graph: LGraph,
    node_ids: Vec<u32>,
    edge_ids: Vec<u32>,
    edge_sources: Vec<u32>,
    edge_targets: Vec<u32>,
) -> Result<LayoutResult, PyErr> {
    let node_arena: Vec<layerd::graph::index::NodeId> = graph.nodes_iter().map(|(id, _)| id).collect();
    let edge_arena: Vec<layerd::graph::index::EdgeId> = graph.edges_iter().map(|(id, _)| id).collect();

    if node_arena.is_empty() {
        let n_edges = edge_ids.len();
        return Ok(LayoutResult {
            width: 0.0,
            height: 0.0,
            node_ids,
            node_x: vec![],
            node_y: vec![],
            node_width: vec![],
            node_height: vec![],
            edge_ids,
            edge_source: edge_sources,
            edge_target: edge_targets,
            edge_bend_start: vec![0; n_edges],
            edge_bend_length: vec![0; n_edges],
            bend_x: vec![],
            bend_y: vec![],
        });
    }

    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        layerd::layout(&mut graph);
        graph
    }))
    .map_err(|_| PyValueError::new_err("layerd pipeline panicked"))?;

    let graph = result;

    let mut out_node_x = Vec::with_capacity(node_arena.len());
    let mut out_node_y = Vec::with_capacity(node_arena.len());
    let mut out_node_w = Vec::with_capacity(node_arena.len());
    let mut out_node_h = Vec::with_capacity(node_arena.len());

    for nid in &node_arena {
        if let Some(n) = graph.try_node(*nid) {
            out_node_x.push(n.position.x);
            out_node_y.push(n.position.y);
            out_node_w.push(n.size.x);
            out_node_h.push(n.size.y);
        } else {
            out_node_x.push(0.0);
            out_node_y.push(0.0);
            out_node_w.push(0.0);
            out_node_h.push(0.0);
        }
    }

    let mut bend_x: Vec<f64> = Vec::new();
    let mut bend_y: Vec<f64> = Vec::new();
    let mut bend_start: Vec<u32> = Vec::with_capacity(edge_arena.len());
    let mut bend_len: Vec<u32> = Vec::with_capacity(edge_arena.len());

    for eid in &edge_arena {
        let start = bend_x.len();
        if let Some(e) = graph.try_edge(*eid) {
            for p in &e.bend_points {
                bend_x.push(p.x);
                bend_y.push(p.y);
            }
        }
        bend_start.push(start as u32);
        bend_len.push((bend_x.len() - start) as u32);
    }

    Ok(LayoutResult {
        width: graph.size.x,
        height: graph.size.y,
        node_ids,
        node_x: out_node_x,
        node_y: out_node_y,
        node_width: out_node_w,
        node_height: out_node_h,
        edge_ids,
        edge_source: edge_sources,
        edge_target: edge_targets,
        edge_bend_start: bend_start,
        edge_bend_length: bend_len,
        bend_x,
        bend_y,
    })
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
    buf.extend_from_slice(&0u32.to_le_bytes());
    buf.extend_from_slice(&(nodes.len() as u32).to_le_bytes());
    buf.extend_from_slice(&(edges.len() as u32).to_le_bytes());
    buf.extend_from_slice(&0u32.to_le_bytes());
    buf.extend_from_slice(&0f64.to_le_bytes());
    buf.extend_from_slice(&0f64.to_le_bytes());

    for n in &nodes {
        buf.extend_from_slice(&n.id.to_le_bytes());
        buf.extend_from_slice(&0u32.to_le_bytes());
        buf.extend_from_slice(&n.width.to_le_bytes());
        buf.extend_from_slice(&n.height.to_le_bytes());
        buf.extend_from_slice(&0f64.to_le_bytes());
        buf.extend_from_slice(&0f64.to_le_bytes());
    }

    for e in &edges {
        buf.extend_from_slice(&e.id.to_le_bytes());
        buf.extend_from_slice(&e.source.to_le_bytes());
        buf.extend_from_slice(&e.target.to_le_bytes());
        buf.extend_from_slice(&0u32.to_le_bytes());
        buf.extend_from_slice(&0u32.to_le_bytes());
        buf.extend_from_slice(&0u32.to_le_bytes());
    }

    Ok(buf)
}

#[pyfunction]
fn layout_flat_py(nodes: Vec<NodeSpec>, edges: Vec<EdgeSpec>) -> PyResult<LayoutResult> {
    let buf = build_lrd1(nodes, edges)?;
    let out = ffi_types::layout_flat(&buf).map_err(ffi_err_to_py)?;
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

#[pyfunction]
#[pyo3(signature = (nodes, edges, options=None))]
fn layout_with_options_py(
    nodes: Vec<NodeSpec>,
    edges: Vec<EdgeSpec>,
    options: Option<PyLayoutOptions>,
) -> PyResult<LayoutResult> {
    let opts = options.map(|o| o.inner);
    // If options provided, use direct LGraph path so options are honored
    if opts.is_some() {
        let (graph, node_ids, edge_ids, edge_sources, edge_targets, _) =
            build_graph(&nodes, &edges, opts)?;
        return run_layout_and_collect(graph, node_ids, edge_ids, edge_sources, edge_targets);
    }
    layout_flat_py(nodes, edges)
}


#[pyfunction]
#[pyo3(signature = (nodes, edges, iters=200, area=80000.0))]
fn layout_force_py(nodes: Vec<NodeSpec>, edges: Vec<EdgeSpec>, iters: usize, area: f64) -> PyResult<LayoutResult> {
    let (mut graph, node_ids, edge_ids, edge_sources, edge_targets, _) = build_graph(&nodes, &edges, None)?;
    // init positions via chain default then force
    force_layout(&mut graph, iters, area);
    // encode size from bounds
    let max_x = graph.nodes_iter().map(|(_, n)| n.position.x + n.size.x).fold(0.0, f64::max);
    let max_y = graph.nodes_iter().map(|(_, n)| n.position.y + n.size.y).fold(0.0, f64::max);
    graph.size = layerd::math::Vec2::new(max_x + 12.0, max_y + 12.0);
    run_layout_and_collect_force(graph, node_ids, edge_ids, edge_sources, edge_targets)
}

fn run_layout_and_collect_force(
    graph: LGraph,
    node_ids: Vec<u32>,
    edge_ids: Vec<u32>,
    edge_sources: Vec<u32>,
    edge_targets: Vec<u32>,
) -> PyResult<LayoutResult> {
    let node_arena: Vec<layerd::graph::index::NodeId> = graph.nodes_iter().map(|(id, _)| id).collect();
    let edge_arena: Vec<layerd::graph::index::EdgeId> = graph.edges_iter().map(|(id, _)| id).collect();
    if node_arena.is_empty() {
        let n = edge_ids.len();
        return Ok(LayoutResult { width: 0.0, height: 0.0, node_ids, node_x: vec![], node_y: vec![], node_width: vec![], node_height: vec![], edge_ids, edge_source: edge_sources, edge_target: edge_targets, edge_bend_start: vec![0; n], edge_bend_length: vec![0; n], bend_x: vec![], bend_y: vec![] });
    }
    let mut xs = Vec::with_capacity(node_arena.len());
    let mut ys = Vec::with_capacity(node_arena.len());
    let mut ws = Vec::with_capacity(node_arena.len());
    let mut hs = Vec::with_capacity(node_arena.len());
    for nid in &node_arena {
        let n = graph.node(*nid);
        xs.push(n.position.x); ys.push(n.position.y); ws.push(n.size.x); hs.push(n.size.y);
    }
    Ok(LayoutResult { width: graph.size.x, height: graph.size.y, node_ids, node_x: xs, node_y: ys, node_width: ws, node_height: hs, edge_ids, edge_source: edge_sources, edge_target: edge_targets, edge_bend_start: vec![0; edge_arena.len()], edge_bend_length: vec![0; edge_arena.len()], bend_x: vec![], bend_y: vec![] })
}

pub fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<NodeSpec>()?;
    m.add_class::<EdgeSpec>()?;
    m.add_class::<LayoutResult>()?;
    m.add_function(wrap_pyfunction!(layout_flat_py, m)?)?;
    m.add_function(wrap_pyfunction!(layout_bytes_py, m)?)?;
    m.add_function(wrap_pyfunction!(layout_with_options_py, m)?)?;
    m.add_function(wrap_pyfunction!(layout_force_py, m)?)?;
    Ok(())
}

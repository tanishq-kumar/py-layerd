use layerd::options::{
    CycleBreakingStrategy, EdgeRoutingStrategy, LayeringStrategy, LayoutDirection,
    LayoutOptions, NodePlacementStrategy, SpacingOptions,
};
use pyo3::prelude::*;
use pyo3::exceptions::PyValueError;

#[pyclass]
#[derive(Clone)]
pub struct PyLayoutOptions {
    pub inner: LayoutOptions,
}

#[pymethods]
impl PyLayoutOptions {
    #[new]
    #[pyo3(signature = (
        direction="RIGHT",
        layering="network_simplex",
        node_placement="brandes_koepf",
        edge_routing="orthogonal",
        cycle_breaking="greedy",
        node_node=20.0,
        node_node_between_layers=20.0,
        edge_node=10.0,
        edge_edge=10.0,
        padding=12.0,
        thoroughness=7,
        random_seed=1,
    ))]
    fn new(
        direction: &str,
        layering: &str,
        node_placement: &str,
        edge_routing: &str,
        cycle_breaking: &str,
        node_node: f64,
        node_node_between_layers: f64,
        edge_node: f64,
        edge_edge: f64,
        padding: f64,
        thoroughness: u32,
        random_seed: u64,
    ) -> PyResult<Self> {
        let mut opts = LayoutOptions::default();
        opts.direction = parse_direction(direction)?;
        opts.layering = parse_layering(layering)?;
        opts.node_placement = parse_node_placement(node_placement)?;
        opts.edge_routing = parse_edge_routing(edge_routing)?;
        opts.cycle_breaking = parse_cycle_breaking(cycle_breaking)?;
        opts.spacing = SpacingOptions {
            node_node,
            node_node_between_layers,
            edge_node,
            edge_edge,
            ..SpacingOptions::default()
        };
        opts.padding = layerd::math::Padding::uniform(padding);
        opts.thoroughness = thoroughness;
        opts.random_seed = random_seed;
        Ok(Self { inner: opts })
    }

    fn __repr__(&self) -> String {
        format!(
            "PyLayoutOptions(direction={}, layering={}, node_placement={}, edge_routing={})",
            self.inner.direction as u8, self.inner.layering as u8, self.inner.node_placement as u8, self.inner.edge_routing as u8
        )
    }
}

fn parse_direction(s: &str) -> PyResult<LayoutDirection> {
    match s.to_lowercase().as_str() {
        "right" | "r" | "east" => Ok(LayoutDirection::Right),
        "left" | "l" | "west" => Ok(LayoutDirection::Left),
        "down" | "d" | "south" => Ok(LayoutDirection::Down),
        "up" | "u" | "north" => Ok(LayoutDirection::Up),
        "undefined" | "undef" => Ok(LayoutDirection::Undefined),
        _ => Err(PyValueError::new_err(format!("unknown direction: {s} (use RIGHT/LEFT/DOWN/UP)"))),
    }
}

fn parse_layering(s: &str) -> PyResult<LayeringStrategy> {
    match s.to_lowercase().as_str() {
        "network_simplex" | "networksimplex" | "simplex" => Ok(LayeringStrategy::NetworkSimplex),
        "longest_path" | "longestpath" => Ok(LayeringStrategy::LongestPath),
        "longest_path_source" | "longestpathsource" => Ok(LayeringStrategy::LongestPathSource),
        "coffman_graham" | "coffmangraham" => Ok(LayeringStrategy::CoffmanGraham),
        "min_width" | "minwidth" => Ok(LayeringStrategy::MinWidth),
        "stretch_width" | "stretchwidth" => Ok(LayeringStrategy::StretchWidth),
        _ => Err(PyValueError::new_err(format!(
            "unknown layering: {s} (use network_simplex/longest_path/coffman_graham/min_width/stretch_width)"
        ))),
    }
}

fn parse_node_placement(s: &str) -> PyResult<NodePlacementStrategy> {
    match s.to_lowercase().as_str() {
        "brandes_koepf" | "brandeskoepf" | "bk" => Ok(NodePlacementStrategy::BrandesKoepf),
        "simple" => Ok(NodePlacementStrategy::Simple),
        "linear_segments" | "linearsegments" => Ok(NodePlacementStrategy::LinearSegments),
        "network_simplex" | "networksimplex" => Ok(NodePlacementStrategy::NetworkSimplex),
        _ => Err(PyValueError::new_err(format!(
            "unknown node_placement: {s} (use brandes_koepf/simple/linear_segments/network_simplex)"
        ))),
    }
}

fn parse_edge_routing(s: &str) -> PyResult<EdgeRoutingStrategy> {
    match s.to_lowercase().as_str() {
        "orthogonal" | "orth" => Ok(EdgeRoutingStrategy::Orthogonal),
        "polyline" | "poly" => Ok(EdgeRoutingStrategy::Polyline),
        "splines" | "spline" => Ok(EdgeRoutingStrategy::Splines),
        _ => Err(PyValueError::new_err(format!(
            "unknown edge_routing: {s} (use orthogonal/polyline/splines)"
        ))),
    }
}

fn parse_cycle_breaking(s: &str) -> PyResult<CycleBreakingStrategy> {
    match s.to_lowercase().as_str() {
        "greedy" => Ok(CycleBreakingStrategy::Greedy),
        "depth_first" | "depthfirst" | "dfs" => Ok(CycleBreakingStrategy::DepthFirst),
        "greedy_model_order" | "greedymodelorder" => Ok(CycleBreakingStrategy::GreedyModelOrder),
        "model_order" | "modelorder" => Ok(CycleBreakingStrategy::ModelOrder),
        _ => Err(PyValueError::new_err(format!(
            "unknown cycle_breaking: {s} (use greedy/depth_first/greedy_model_order/model_order)"
        ))),
    }
}

pub fn register(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyLayoutOptions>()?;
    Ok(())
}

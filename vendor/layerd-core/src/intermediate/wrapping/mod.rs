//! Wrapping subsystem: a collection of processors and helpers that split
//! wide-and-narrow layerings into multiple rows and reconnect the
//! back-wrapped edges.

pub mod breaking_point_info;
pub mod breaking_point_inserter;
pub mod breaking_point_processor;
pub mod breaking_point_remover;
pub mod cut_index_calc;
pub mod cutting_utils;
pub mod graph_stats;
pub mod single_edge_graph_wrapper;

pub use breaking_point_inserter::insert as insert_breaking_points;
pub use breaking_point_processor::process as process_breaking_points;
pub use breaking_point_remover::remove as remove_breaking_points;
pub use single_edge_graph_wrapper::wrap as wrap_single_edge_graph;

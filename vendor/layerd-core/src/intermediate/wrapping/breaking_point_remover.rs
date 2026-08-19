//! Removes breaking-point dummies after phase 5 and transfers the routed
//! bend points back onto the original edges. Dispatches on the active
//! `EdgeRoutingStrategy` — POLYLINE, ORTHOGONAL, and SPLINES all have
//! distinct join strategies.

use smallvec::SmallVec;

use super::breaking_point_info::{BPInfo, BREAKING_POINT_INFO, BREAKING_POINT_INFO_STORE, is_end};
use crate::{
    graph::{
        LGraph,
        index::{EdgeId, NodeId},
    },
    math::Vec2,
    options::enums::EdgeRoutingStrategy,
    p5_edge_routing::splines::segment::{
        SPLINE_EDGE_CHAIN, SPLINE_ROUTE_START, SPLINE_SEGMENT_STORE, SegmentId,
    },
    properties::internal::{JUNCTION_POINTS, SPLINE_SURVIVING_EDGE},
};

/// Breaking-point remover entry point. Walks the graph once, picks off
/// every terminal breaking point (i.e. one whose `next` link is `None`),
/// and unwinds the chain via `remove`.
pub fn remove(graph: &mut LGraph) {
    let edge_routing = graph.options.edge_routing;

    // Collect all terminal end-dummies first; removing in place mutates the
    // layer lists so we snapshot up-front.
    let mut terminals: Vec<NodeId> = Vec::new();
    let store: Vec<BPInfo> = graph.properties.get(&BREAKING_POINT_INFO_STORE);
    if store.is_empty() {
        return;
    }
    for layer in &graph.layers {
        for &n in &layer.nodes {
            if is_end(n, &graph.node(n).properties, &store)
                && let Some(id) = graph.node(n).properties.get(&BREAKING_POINT_INFO)
                && store[id.index()].next.is_none()
            {
                terminals.push(n);
            }
        }
    }

    for end_node in terminals {
        let chain_head: Option<_> = graph.node(end_node).properties.get(&BREAKING_POINT_INFO);
        if let Some(id) = chain_head {
            unwind_chain(graph, id, edge_routing);
        }
    }
}

fn unwind_chain(
    graph: &mut LGraph,
    id: super::breaking_point_info::BPInfoId,
    edge_routing: EdgeRoutingStrategy,
) {
    let store: Vec<BPInfo> = graph.properties.get(&BREAKING_POINT_INFO_STORE);
    let mut current = Some(id);
    while let Some(id) = current {
        let bpi = store[id.index()];
        remove_single(graph, bpi, edge_routing);
        current = bpi.prev;
    }
}

fn remove_single(graph: &mut LGraph, bpi: BPInfo, edge_routing: EdgeRoutingStrategy) {
    let mut new_bends: Vec<Vec2> = Vec::new();
    match edge_routing {
        EdgeRoutingStrategy::Splines => {
            join_spline_chains(graph, bpi);
        }
        EdgeRoutingStrategy::Polyline => {
            new_bends.extend(graph.edge(bpi.node_start_edge).bend_points.iter().copied());
            new_bends.push(graph.node(bpi.start).position);
            let mut start_end_bends: Vec<Vec2> =
                graph.edge(bpi.start_end_edge).bend_points.to_vec();
            start_end_bends.reverse();
            new_bends.extend(start_end_bends);
            new_bends.push(graph.node(bpi.end).position);
            new_bends.extend(graph.edge(bpi.original_edge).bend_points.iter().copied());
        }
        EdgeRoutingStrategy::Orthogonal => {
            new_bends.extend(graph.edge(bpi.node_start_edge).bend_points.iter().copied());
            let mut start_end_bends: Vec<Vec2> =
                graph.edge(bpi.start_end_edge).bend_points.to_vec();
            start_end_bends.reverse();
            new_bends.extend(start_end_bends);
            new_bends.extend(graph.edge(bpi.original_edge).bend_points.iter().copied());
        }
    }

    // Restore original edge. For the SPLINES branch, `new_bends` stays empty
    // because the bend points get derived from spline segments during the
    // final bend-point calculator pass.
    graph.edge_mut(bpi.original_edge).bend_points = new_bends;
    let new_src = graph.edge(bpi.node_start_edge).source;
    graph.reroute_edge_source(bpi.original_edge, new_src);

    // Merge junction points.
    let jp_one: SmallVec<Vec2, 4> =
        graph.edge(bpi.node_start_edge).properties.get(&JUNCTION_POINTS);
    let jp_two: SmallVec<Vec2, 4> = graph.edge(bpi.start_end_edge).properties.get(&JUNCTION_POINTS);
    let jp_three: SmallVec<Vec2, 4> =
        graph.edge(bpi.original_edge).properties.get(&JUNCTION_POINTS);
    if !(jp_one.is_empty() && jp_two.is_empty() && jp_three.is_empty()) {
        let mut merged: SmallVec<Vec2, 4> = SmallVec::new();
        for p in jp_three {
            merged.push(p);
        }
        for p in jp_two {
            merged.push(p);
        }
        for p in jp_one {
            merged.push(p);
        }
        graph.edge_mut(bpi.original_edge).properties.set(&JUNCTION_POINTS, merged);
    }

    // Remove the leftover dummies + disconnected edges.
    detach_edge(graph, bpi.start_end_edge);
    detach_edge(graph, bpi.node_start_edge);
    detach_node_from_layer(graph, bpi.end);
    detach_node_from_layer(graph, bpi.start);
}

fn detach_edge(graph: &mut LGraph, edge: EdgeId) {
    let src = graph.edge(edge).source;
    let tgt = graph.edge(edge).target;
    graph.port_mut(src).outgoing_edges.retain(|e| *e != edge);
    graph.port_mut(tgt).incoming_edges.retain(|e| *e != edge);
}

fn detach_node_from_layer(graph: &mut LGraph, node: NodeId) {
    if let Some(l) = graph.node(node).layer.get()
        && l < graph.layers.len()
    {
        graph.layers[l].nodes.retain(|&n| n != node);
    }
    graph.node_mut(node).layer = None.into();
}

/// SPLINES branch of breaking-point removal.
///
/// Join the three spline route chains (`nodeStartEdge` + reversed
/// `startEndEdge` + `originalEdge`) onto the original edge and mark every
/// segment of the middle chain with `inverse_order = true` so the final
/// bend-point calculator knows to walk it backwards.
fn join_spline_chains(graph: &mut LGraph, bpi: BPInfo) {
    let s1: Vec<SegmentId> = graph.edge(bpi.node_start_edge).properties.get(&SPLINE_ROUTE_START);
    let s2: Vec<SegmentId> = graph.edge(bpi.start_end_edge).properties.get(&SPLINE_ROUTE_START);
    let s3: Vec<SegmentId> = graph.edge(bpi.original_edge).properties.get(&SPLINE_ROUTE_START);

    let e1: Vec<EdgeId> = graph.edge(bpi.node_start_edge).properties.get(&SPLINE_EDGE_CHAIN);
    let e2: Vec<EdgeId> = graph.edge(bpi.start_end_edge).properties.get(&SPLINE_EDGE_CHAIN);
    let e3: Vec<EdgeId> = graph.edge(bpi.original_edge).properties.get(&SPLINE_EDGE_CHAIN);

    // Flip inverse_order on every segment that belongs to the middle chain.
    let mut store = graph.properties.get(&SPLINE_SEGMENT_STORE);
    for seg_id in &s2 {
        let idx = seg_id.0 as usize;
        if idx < store.len() {
            store[idx].inverse_order = true;
        }
    }
    graph.properties.set(&SPLINE_SEGMENT_STORE, store);

    let mut joined_segments: Vec<SegmentId> = Vec::with_capacity(s1.len() + s2.len() + s3.len());
    joined_segments.extend(s1);
    joined_segments.extend(s2.into_iter().rev());
    joined_segments.extend(s3);

    let mut joined_edges: Vec<EdgeId> = Vec::with_capacity(e1.len() + e2.len() + e3.len());
    joined_edges.extend(e1);
    joined_edges.extend(e2.into_iter().rev());
    joined_edges.extend(e3);

    graph
        .edge_mut(bpi.original_edge)
        .properties
        .set(&SPLINE_ROUTE_START, joined_segments);
    graph
        .edge_mut(bpi.original_edge)
        .properties
        .set(&SPLINE_EDGE_CHAIN, joined_edges);
    graph
        .edge_mut(bpi.original_edge)
        .properties
        .set(&SPLINE_SURVIVING_EDGE, Some(bpi.original_edge));

    // Clear the middle / start chains — the property map treats default as
    // `Vec::new()`.
    graph
        .edge_mut(bpi.node_start_edge)
        .properties
        .set(&SPLINE_ROUTE_START, Vec::new());
    graph
        .edge_mut(bpi.node_start_edge)
        .properties
        .set(&SPLINE_EDGE_CHAIN, Vec::new());
    graph
        .edge_mut(bpi.start_end_edge)
        .properties
        .set(&SPLINE_ROUTE_START, Vec::new());
    graph
        .edge_mut(bpi.start_end_edge)
        .properties
        .set(&SPLINE_EDGE_CHAIN, Vec::new());
}

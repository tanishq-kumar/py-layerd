use layerd::graph::index::{EdgeId, NodeId};
use layerd::graph::LGraph;
use layerd::math::Vec2;

/// Tiny Fruchterman-Reingold force layout on top of LGraph positions.
///
/// Runs `iters` sweeps over all node pairs (repulsion) + edges (attraction),
/// with linear cooling. Starts from current `node.position` (or random if
/// all at origin), writes back to `node.position`. Single-threaded, keeps
/// the same `LGraph` so `ffi-types` encode path is reused.
pub fn force_layout(graph: &mut LGraph, iters: usize, area: f64) {
    let node_ids: Vec<NodeId> = graph.nodes_iter().map(|(id, _)| id).collect();
    let edge_ids: Vec<EdgeId> = graph.edges_iter().map(|(id, _)| id).collect();
    let n = node_ids.len() as f64;
    if n == 0.0 {
        return;
    }
    let k = (area / n).sqrt().max(8.0);
    let k2 = k * k;

    let mut rng: u64 = 0x9E3779B97F4A7C15;
    let mut disp: Vec<Vec2> = vec![Vec2::ZERO; node_ids.len()];
    let idx_of = |graph: &LGraph, nid: NodeId| -> usize {
        // map NodeId -> index in node_ids vec
        node_ids.iter().position(|&id| id == nid).unwrap_or(0)
    };

    // init: jitter if all at 0
    let all_zero = node_ids.iter().all(|&id| graph.node(id).position == Vec2::ZERO);
    if all_zero {
        for &id in &node_ids {
            let x = (rng as f64 / u64::MAX as f64) * k * n.sqrt();
            let y = (rng as f64 / u64::MAX as f64) * k * n.sqrt();
            rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            graph.node_mut(id).position = Vec2::new(x, y);
        }
    }

    for step in 0..iters {
        let t = k * (1.0 - step as f64 / iters as f64).max(0.02);

        for d in &mut disp {
            *d = Vec2::ZERO;
        }

        // repulsion: O(n^2)
        for i in 0..node_ids.len() {
            for j in (i + 1)..node_ids.len() {
                let a = graph.node(node_ids[i]).position;
                let b = graph.node(node_ids[j]).position;
                let mut dx = a.x - b.x;
                let mut dy = a.y - b.y;
                let mut dist2 = dx * dx + dy * dy + 1e-9;
                let dist = dist2.sqrt();
                let f = k2 / dist;
                let fx = dx / dist * f;
                let fy = dy / dist * f;
                disp[i].x += fx;
                disp[i].y += fy;
                disp[j].x -= fx;
                disp[j].y -= fy;
            }
        }

        // attraction
        for &eid in &edge_ids {
            let e = graph.edge(eid);
            let u = graph.port(e.source).owner;
            let v = graph.port(e.target).owner;
            let i = idx_of(graph, u);
            let j = idx_of(graph, v);
            let a = graph.node(u).position;
            let b = graph.node(v).position;
            let mut dx = a.x - b.x;
            let mut dy = a.y - b.y;
            let dist = (dx * dx + dy * dy).sqrt().max(1e-9);
            let f = dist * dist / k;
            let fx = dx / dist * f;
            let fy = dy / dist * f;
            disp[i].x -= fx;
            disp[i].y -= fy;
            disp[j].x += fx;
            disp[j].y += fy;
        }

        for (idx, &nid) in node_ids.iter().enumerate() {
            let mut d = disp[idx];
            let len = (d.x * d.x + d.y * d.y).sqrt().max(1e-9);
            let clamped = len.min(t);
            d.x = d.x / len * clamped;
            d.y = d.y / len * clamped;
            let p = graph.node(nid).position;
            graph.node_mut(nid).position = Vec2::new(p.x + d.x, p.y + d.y);
        }
    }
}

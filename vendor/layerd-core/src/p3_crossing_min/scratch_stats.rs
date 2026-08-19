use std::sync::{
    Mutex, OnceLock,
    atomic::{AtomicBool, Ordering},
};

static FORCE_ENABLED: AtomicBool = AtomicBool::new(false);
static STATS: OnceLock<Mutex<P3ScratchStats>> = OnceLock::new();

#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct CountingScratchFootprint {
    pub(crate) retained_bytes: usize,
    pub(crate) port_position_slots: usize,
    pub(crate) port_position_capacity: usize,
    pub(crate) node_cardinality_slots: usize,
    pub(crate) node_cardinality_capacity: usize,
    pub(crate) seen_port_slots: usize,
    pub(crate) dense_adjacency_slots: usize,
    pub(crate) bit_capacity: usize,
    pub(crate) ports_capacity: usize,
    pub(crate) relevant_ports_capacity: usize,
    pub(crate) adjacency_capacity: usize,
    pub(crate) hyperedge_capacity: usize,
    pub(crate) ns_stack_capacity: usize,
}

#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct PortDistributionFootprint {
    pub(crate) retained_bytes: usize,
    pub(crate) free_rank_slots: usize,
    pub(crate) free_rank_capacity: usize,
    pub(crate) barycenter_slots: usize,
    pub(crate) barycenter_capacity: usize,
    pub(crate) node_position_slots: usize,
    pub(crate) node_position_capacity: usize,
}

#[derive(Clone, Debug, Default)]
pub struct P3ScratchStats {
    pub counting_instances: usize,
    pub port_distribution_instances: usize,
    pub counting_retained_bytes: usize,
    pub port_distribution_retained_bytes: usize,
    pub max_counting_retained_bytes: usize,
    pub max_port_distribution_retained_bytes: usize,
    pub max_port_position_slots: usize,
    pub max_port_position_capacity: usize,
    pub max_node_cardinality_slots: usize,
    pub max_node_cardinality_capacity: usize,
    pub max_seen_port_slots: usize,
    pub max_dense_adjacency_slots: usize,
    pub max_bit_capacity: usize,
    pub max_ports_capacity: usize,
    pub max_relevant_ports_capacity: usize,
    pub max_adjacency_capacity: usize,
    pub max_hyperedge_capacity: usize,
    pub max_ns_stack_capacity: usize,
    pub max_free_rank_slots: usize,
    pub max_free_rank_capacity: usize,
    pub max_barycenter_slots: usize,
    pub max_barycenter_capacity: usize,
    pub max_node_position_slots: usize,
    pub max_node_position_capacity: usize,
}

#[cfg(feature = "devtools")]
pub fn enable_global_stats() {
    FORCE_ENABLED.store(true, Ordering::Relaxed);
}

#[cfg(feature = "devtools")]
pub fn reset_global_stats() {
    if let Some(stats) = STATS.get() {
        *stats.lock().expect("P3 scratch stats lock poisoned") = P3ScratchStats::default();
    }
}

#[cfg(feature = "devtools")]
pub fn take_global_stats() -> P3ScratchStats {
    if let Some(stats) = STATS.get() {
        std::mem::take(&mut *stats.lock().expect("P3 scratch stats lock poisoned"))
    } else {
        P3ScratchStats::default()
    }
}

pub(crate) fn enabled() -> bool {
    if FORCE_ENABLED.load(Ordering::Relaxed) {
        return true;
    }
    static ENV_ENABLED: OnceLock<bool> = OnceLock::new();
    *ENV_ENABLED.get_or_init(|| std::env::var_os("LAYERD_P3_SCRATCH_STATS").is_some())
}

pub(crate) fn record_counting(footprint: CountingScratchFootprint) {
    if !enabled() {
        return;
    }
    let mut stats = stats().lock().expect("P3 scratch stats lock poisoned");
    stats.counting_instances += 1;
    stats.counting_retained_bytes += footprint.retained_bytes;
    stats.max_counting_retained_bytes =
        stats.max_counting_retained_bytes.max(footprint.retained_bytes);
    stats.max_port_position_slots =
        stats.max_port_position_slots.max(footprint.port_position_slots);
    stats.max_port_position_capacity =
        stats.max_port_position_capacity.max(footprint.port_position_capacity);
    stats.max_node_cardinality_slots =
        stats.max_node_cardinality_slots.max(footprint.node_cardinality_slots);
    stats.max_node_cardinality_capacity =
        stats.max_node_cardinality_capacity.max(footprint.node_cardinality_capacity);
    stats.max_seen_port_slots = stats.max_seen_port_slots.max(footprint.seen_port_slots);
    stats.max_dense_adjacency_slots =
        stats.max_dense_adjacency_slots.max(footprint.dense_adjacency_slots);
    stats.max_bit_capacity = stats.max_bit_capacity.max(footprint.bit_capacity);
    stats.max_ports_capacity = stats.max_ports_capacity.max(footprint.ports_capacity);
    stats.max_relevant_ports_capacity =
        stats.max_relevant_ports_capacity.max(footprint.relevant_ports_capacity);
    stats.max_adjacency_capacity = stats.max_adjacency_capacity.max(footprint.adjacency_capacity);
    stats.max_hyperedge_capacity = stats.max_hyperedge_capacity.max(footprint.hyperedge_capacity);
    stats.max_ns_stack_capacity = stats.max_ns_stack_capacity.max(footprint.ns_stack_capacity);
}

pub(crate) fn record_port_distribution(footprint: PortDistributionFootprint) {
    if !enabled() {
        return;
    }
    let mut stats = stats().lock().expect("P3 scratch stats lock poisoned");
    stats.port_distribution_instances += 1;
    stats.port_distribution_retained_bytes += footprint.retained_bytes;
    stats.max_port_distribution_retained_bytes =
        stats.max_port_distribution_retained_bytes.max(footprint.retained_bytes);
    stats.max_free_rank_slots = stats.max_free_rank_slots.max(footprint.free_rank_slots);
    stats.max_free_rank_capacity = stats.max_free_rank_capacity.max(footprint.free_rank_capacity);
    stats.max_barycenter_slots = stats.max_barycenter_slots.max(footprint.barycenter_slots);
    stats.max_barycenter_capacity =
        stats.max_barycenter_capacity.max(footprint.barycenter_capacity);
    stats.max_node_position_slots =
        stats.max_node_position_slots.max(footprint.node_position_slots);
    stats.max_node_position_capacity =
        stats.max_node_position_capacity.max(footprint.node_position_capacity);
}

fn stats() -> &'static Mutex<P3ScratchStats> {
    STATS.get_or_init(|| Mutex::new(P3ScratchStats::default()))
}

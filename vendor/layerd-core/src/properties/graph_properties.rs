use bitflags::bitflags;

bitflags! {
    /// Flags describing structural properties of a graph.
    #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
    pub struct GraphProperties: u32 {
        const COMMENTS          = 1 << 0;
        const EXTERNAL_PORTS    = 1 << 1;
        const HYPEREDGES        = 1 << 2;
        const HYPERNODES        = 1 << 3;
        const NON_FREE_PORTS    = 1 << 4;
        const NORTH_SOUTH_PORTS = 1 << 5;
        const SELF_LOOPS        = 1 << 6;
        const CENTER_LABELS     = 1 << 7;
        const END_LABELS        = 1 << 8;
        const PARTITIONS        = 1 << 9;
    }
}

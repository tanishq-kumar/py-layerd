/// Spacing configuration for the layered layout algorithm.
///
/// This module provides spacing options and a builder pattern for
/// configuring detailed spacing constraints between graph elements.
use hashbrown::HashMap;

/// Represents a single spacing property with a default value and a factor.
///
/// The effective spacing = factor * base_spacing_default
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SpacingProperty {
    /// The default value for this spacing property.
    pub default: f64,
    /// The factor to apply (for relative spacing).
    pub factor: f64,
}

impl SpacingProperty {
    /// Create a new spacing property with default and factor.
    pub fn new(default: f64, factor: f64) -> Self {
        SpacingProperty { default, factor }
    }

    /// Get the effective spacing value.
    pub fn effective(&self, base_spacing: f64) -> f64 {
        self.default * self.factor / base_spacing
    }
}

/// Base spacing value used for relative calculations.
pub const BASE_SPACING_DEFAULT: f64 = 20.0;

/// Spacing between edges and edges.
pub const SPACING_EDGE_EDGE: SpacingProperty = SpacingProperty { default: 10.0, factor: 1.0 };

/// Spacing between edges and edge labels. Default `2.0`.
pub const SPACING_EDGE_LABEL: SpacingProperty = SpacingProperty { default: 2.0, factor: 1.0 };

/// Spacing between edges and nodes.
pub const SPACING_EDGE_NODE: SpacingProperty = SpacingProperty { default: 10.0, factor: 1.0 };

/// Spacing between labels and labels.
pub const SPACING_LABEL_LABEL: SpacingProperty = SpacingProperty { default: 0.0, factor: 1.0 };

/// Spacing between labels and nodes.
pub const SPACING_LABEL_NODE: SpacingProperty = SpacingProperty { default: 5.0, factor: 1.0 };

/// Horizontal spacing between labels and ports.
pub const SPACING_LABEL_PORT_HORIZONTAL: SpacingProperty =
    SpacingProperty { default: 1.0, factor: 1.0 };

/// Vertical spacing between labels and ports.
pub const SPACING_LABEL_PORT_VERTICAL: SpacingProperty =
    SpacingProperty { default: 1.0, factor: 1.0 };

/// Spacing around self-loop nodes. Default `10.0`.
pub const SPACING_NODE_SELF_LOOP: SpacingProperty = SpacingProperty { default: 10.0, factor: 1.0 };

/// Spacing between ports on the same node.
pub const SPACING_PORT_PORT: SpacingProperty = SpacingProperty { default: 10.0, factor: 1.0 };

/// Spacing between edges in adjacent layers.
pub const SPACING_EDGE_EDGE_BETWEEN_LAYERS: SpacingProperty =
    SpacingProperty { default: 10.0, factor: 1.0 };

/// Spacing between edges and nodes in adjacent layers.
pub const SPACING_EDGE_NODE_BETWEEN_LAYERS: SpacingProperty =
    SpacingProperty { default: 10.0, factor: 1.0 };

/// Spacing between nodes in adjacent layers.
pub const SPACING_NODE_NODE_BETWEEN_LAYERS: SpacingProperty =
    SpacingProperty { default: 20.0, factor: 1.0 };

/// Spacing between comment boxes.
pub const SPACING_COMMENT_COMMENT: SpacingProperty = SpacingProperty { default: 10.0, factor: 1.0 };

/// Spacing between a comment box and its associated node.
pub const SPACING_COMMENT_NODE: SpacingProperty = SpacingProperty { default: 10.0, factor: 1.0 };

/// Spacing options for the layout algorithm.
///
/// Controls the distances between nodes, edges, ports, and labels.
#[derive(Debug, Clone, Copy)]
pub struct SpacingOptions {
    /// Minimum spacing between nodes in the same layer.
    pub node_node: f64,
    /// Minimum spacing between nodes in adjacent layers.
    pub node_node_between_layers: f64,
    /// Minimum spacing between edges and nodes in the same layer.
    pub edge_node: f64,
    /// Minimum spacing between edges and nodes in adjacent layers.
    pub edge_node_between_layers: f64,
    /// Minimum spacing between edges in the same layer.
    pub edge_edge: f64,
    /// Minimum spacing between edges in adjacent layers.
    pub edge_edge_between_layers: f64,
    /// Minimum spacing between ports on the same node.
    pub port_port: f64,
    /// Horizontal spacing between labels and ports.
    pub label_port_horizontal: f64,
    /// Vertical spacing between labels and ports.
    pub label_port_vertical: f64,
    /// Spacing between labels and nodes.
    pub label_node: f64,
    /// Spacing between labels.
    pub label_label: f64,
    /// Spacing between edges and edge labels.
    pub edge_label: f64,
    /// Spacing around self-loop nodes.
    pub node_self_loop: f64,
    /// Spacing between comment boxes.
    pub comment_comment: f64,
    /// Spacing between a comment box and its associated node.
    pub comment_node: f64,
}

impl Default for SpacingOptions {
    fn default() -> Self {
        SpacingOptions {
            node_node: 20.0,
            node_node_between_layers: 20.0,
            edge_node: 10.0,
            edge_node_between_layers: 10.0,
            edge_edge: 10.0,
            edge_edge_between_layers: 10.0,
            port_port: 10.0,
            label_port_horizontal: 1.0,
            label_port_vertical: 1.0,
            label_node: 5.0,
            label_label: 0.0,
            edge_label: SPACING_EDGE_LABEL.default,
            node_self_loop: SPACING_NODE_SELF_LOOP.default,
            comment_comment: SPACING_COMMENT_COMMENT.default,
            comment_node: SPACING_COMMENT_NODE.default,
        }
    }
}

/// Builder for configuring layered spacing options.
///
/// Allows fine-grained control over spacing properties between graph elements.
/// Uses a fluent API for convenient configuration.
///
/// # Example
///
/// ```ignore
/// let builder = LayeredSpacingsBuilder::new()
///     .edge_edge(15.0)
///     .node_node_between_layers(30.0)
///     .build();
/// ```
#[derive(Debug, Clone)]
pub struct LayeredSpacingsBuilder {
    properties: HashMap<&'static str, f64>,
}

impl Default for LayeredSpacingsBuilder {
    fn default() -> Self {
        Self::new()
    }
}

impl LayeredSpacingsBuilder {
    /// Create a new builder with default spacing values.
    pub fn new() -> Self {
        LayeredSpacingsBuilder { properties: HashMap::new() }
    }

    /// Create a builder that computes all spacing values from a base value.
    ///
    /// Each spacing property is set to `base_value * (property.default / BASE_SPACING_DEFAULT)`.
    pub fn with_base_value(base_value: f64) -> Self {
        let mut builder = Self::new();
        let all_properties: &[(&str, SpacingProperty)] = &[
            ("edge_edge", SPACING_EDGE_EDGE),
            ("edge_label", SPACING_EDGE_LABEL),
            ("edge_node", SPACING_EDGE_NODE),
            ("label_label", SPACING_LABEL_LABEL),
            ("label_node", SPACING_LABEL_NODE),
            ("label_port_horizontal", SPACING_LABEL_PORT_HORIZONTAL),
            ("label_port_vertical", SPACING_LABEL_PORT_VERTICAL),
            ("node_self_loop", SPACING_NODE_SELF_LOOP),
            ("port_port", SPACING_PORT_PORT),
            ("edge_edge_between_layers", SPACING_EDGE_EDGE_BETWEEN_LAYERS),
            ("edge_node_between_layers", SPACING_EDGE_NODE_BETWEEN_LAYERS),
            ("node_node_between_layers", SPACING_NODE_NODE_BETWEEN_LAYERS),
        ];
        for &(key, prop) in all_properties {
            let factor = prop.default / BASE_SPACING_DEFAULT;
            builder.properties.insert(key, base_value * factor);
        }
        builder
    }

    /// Apply the configured spacing values to a graph's options.
    pub fn apply(&self, options: &mut super::SpacingOptions) {
        for (&key, &value) in &self.properties {
            match key {
                "edge_edge" => options.edge_edge = value,
                "edge_label" => options.edge_label = value,
                "edge_node" => options.edge_node = value,
                "label_label" => options.label_label = value,
                "label_node" => options.label_node = value,
                "label_port_horizontal" => options.label_port_horizontal = value,
                "label_port_vertical" => options.label_port_vertical = value,
                "node_self_loop" => options.node_self_loop = value,
                "port_port" => options.port_port = value,
                "edge_edge_between_layers" => options.edge_edge_between_layers = value,
                "edge_node_between_layers" => options.edge_node_between_layers = value,
                "node_node_between_layers" => options.node_node_between_layers = value,
                _ => {}
            }
        }
    }

    /// Set spacing between edges and edges.
    pub fn edge_edge(mut self, value: f64) -> Self {
        self.properties.insert("edge_edge", value);
        self
    }

    /// Set spacing between edges and edge labels.
    pub fn edge_label(mut self, value: f64) -> Self {
        self.properties.insert("edge_label", value);
        self
    }

    /// Set spacing between edges and nodes.
    pub fn edge_node(mut self, value: f64) -> Self {
        self.properties.insert("edge_node", value);
        self
    }

    /// Set spacing between labels and labels.
    pub fn label_label(mut self, value: f64) -> Self {
        self.properties.insert("label_label", value);
        self
    }

    /// Set spacing between labels and nodes.
    pub fn label_node(mut self, value: f64) -> Self {
        self.properties.insert("label_node", value);
        self
    }

    /// Set horizontal spacing between labels and ports.
    pub fn label_port_horizontal(mut self, value: f64) -> Self {
        self.properties.insert("label_port_horizontal", value);
        self
    }

    /// Set vertical spacing between labels and ports.
    pub fn label_port_vertical(mut self, value: f64) -> Self {
        self.properties.insert("label_port_vertical", value);
        self
    }

    /// Set spacing around self-loop nodes.
    pub fn node_self_loop(mut self, value: f64) -> Self {
        self.properties.insert("node_self_loop", value);
        self
    }

    /// Set spacing between ports on the same node.
    pub fn port_port(mut self, value: f64) -> Self {
        self.properties.insert("port_port", value);
        self
    }

    /// Set spacing between edges in adjacent layers.
    pub fn edge_edge_between_layers(mut self, value: f64) -> Self {
        self.properties.insert("edge_edge_between_layers", value);
        self
    }

    /// Set spacing between edges and nodes in adjacent layers.
    pub fn edge_node_between_layers(mut self, value: f64) -> Self {
        self.properties.insert("edge_node_between_layers", value);
        self
    }

    /// Set spacing between nodes in adjacent layers.
    pub fn node_node_between_layers(mut self, value: f64) -> Self {
        self.properties.insert("node_node_between_layers", value);
        self
    }

    /// Build the spacing options from the current configuration.
    pub fn build(self) -> SpacingOptions {
        let mut opts = SpacingOptions::default();

        // Apply configured properties
        for (key, value) in self.properties {
            match key {
                "edge_edge" => opts.edge_edge = value,
                "edge_label" => opts.edge_label = value,
                "edge_node" => opts.edge_node = value,
                "label_label" => opts.label_label = value,
                "label_node" => opts.label_node = value,
                "label_port_horizontal" => opts.label_port_horizontal = value,
                "label_port_vertical" => opts.label_port_vertical = value,
                "node_self_loop" => opts.node_self_loop = value,
                "port_port" => opts.port_port = value,
                "edge_edge_between_layers" => opts.edge_edge_between_layers = value,
                "edge_node_between_layers" => opts.edge_node_between_layers = value,
                "node_node_between_layers" => opts.node_node_between_layers = value,
                _ => {}
            }
        }

        opts
    }

    /// Get a configured property value, or its default.
    pub fn get_property(&self, name: &str) -> f64 {
        *self.properties.get(name).unwrap_or(&0.0)
    }

    /// Get the default factor for a spacing property.
    pub fn get_default_factor(property: SpacingProperty) -> f64 {
        property.default / BASE_SPACING_DEFAULT
    }
}

#[cfg(test)]
mod copy_contracts {
    use super::*;

    #[test]
    fn copy_candidates_are_copy() {
        fn assert_copy<T: Copy>() {}

        assert_copy::<SpacingOptions>();
    }
}

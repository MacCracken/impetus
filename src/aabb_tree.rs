//! Dynamic AABB tree (balanced BVH) for broadphase collision detection.
//!
//! Handles heterogeneous object sizes better than spatial hash grids.
//! Uses fattened AABBs to reduce reinsertions when objects move slightly.

use std::collections::BTreeSet;

/// Axis-aligned bounding box used by the tree.
#[derive(Debug, Clone, Copy)]
pub(crate) struct TreeAabb {
    pub min: [f64; 3],
    pub max: [f64; 3],
}

#[allow(dead_code)]
impl TreeAabb {
    /// Create from 2D bounds (z = 0).
    #[inline]
    #[must_use]
    pub fn from_2d(min: [f64; 2], max: [f64; 2]) -> Self {
        Self {
            min: [min[0], min[1], 0.0],
            max: [max[0], max[1], 0.0],
        }
    }

    /// Create from 3D bounds.
    #[inline]
    #[must_use]
    pub fn from_3d(min: [f64; 3], max: [f64; 3]) -> Self {
        Self { min, max }
    }

    /// Fatten the AABB by a margin on all sides.
    #[inline]
    #[must_use]
    fn fattened(&self, margin: f64) -> Self {
        Self {
            min: [
                self.min[0] - margin,
                self.min[1] - margin,
                self.min[2] - margin,
            ],
            max: [
                self.max[0] + margin,
                self.max[1] + margin,
                self.max[2] + margin,
            ],
        }
    }

    /// Surface area heuristic for tree balancing.
    #[inline]
    #[must_use]
    fn surface_area(&self) -> f64 {
        let dx = self.max[0] - self.min[0];
        let dy = self.max[1] - self.min[1];
        let dz = self.max[2] - self.min[2];
        2.0 * (dx * dy + dy * dz + dz * dx)
    }

    /// Merge two AABBs into one that contains both.
    #[inline]
    #[must_use]
    fn merged(&self, other: &Self) -> Self {
        Self {
            min: [
                self.min[0].min(other.min[0]),
                self.min[1].min(other.min[1]),
                self.min[2].min(other.min[2]),
            ],
            max: [
                self.max[0].max(other.max[0]),
                self.max[1].max(other.max[1]),
                self.max[2].max(other.max[2]),
            ],
        }
    }

    /// Check if this AABB overlaps another.
    #[inline]
    #[must_use]
    fn overlaps(&self, other: &Self) -> bool {
        self.min[0] <= other.max[0]
            && self.max[0] >= other.min[0]
            && self.min[1] <= other.max[1]
            && self.max[1] >= other.min[1]
            && self.min[2] <= other.max[2]
            && self.max[2] >= other.min[2]
    }

    /// Check if this AABB fully contains another.
    #[inline]
    #[must_use]
    fn contains(&self, other: &Self) -> bool {
        self.min[0] <= other.min[0]
            && self.min[1] <= other.min[1]
            && self.min[2] <= other.min[2]
            && self.max[0] >= other.max[0]
            && self.max[1] >= other.max[1]
            && self.max[2] >= other.max[2]
    }
}

const NULL_NODE: u32 = u32::MAX;

/// Fattening margin — how much to expand AABBs to reduce reinsertions.
const AABB_MARGIN: f64 = 0.1;

#[derive(Debug, Clone)]
struct Node<K: Copy + Eq + Ord> {
    aabb: TreeAabb,
    parent: u32,
    left: u32,
    right: u32,
    /// Leaf nodes store a key; internal nodes store None.
    key: Option<K>,
    height: i32,
}

impl<K: Copy + Eq + Ord> Node<K> {
    #[inline]
    fn is_leaf(&self) -> bool {
        self.left == NULL_NODE
    }
}

/// Dynamic AABB tree for broadphase collision detection.
pub(crate) struct AabbTree<K: Copy + Eq + Ord> {
    nodes: Vec<Node<K>>,
    root: u32,
    free_list: Vec<u32>,
}

#[allow(dead_code)]
impl<K: Copy + Eq + Ord> AabbTree<K> {
    /// Create an empty tree.
    pub fn new() -> Self {
        Self {
            nodes: Vec::new(),
            root: NULL_NODE,
            free_list: Vec::new(),
        }
    }

    /// Insert a leaf with the given key and tight AABB. Returns the node index.
    pub fn insert(&mut self, key: K, aabb: TreeAabb) -> u32 {
        let fat_aabb = aabb.fattened(AABB_MARGIN);
        let leaf = self.alloc_node(Node {
            aabb: fat_aabb,
            parent: NULL_NODE,
            left: NULL_NODE,
            right: NULL_NODE,
            key: Some(key),
            height: 0,
        });

        if self.root == NULL_NODE {
            self.root = leaf;
            return leaf;
        }

        // Find the best sibling using surface area heuristic
        let mut best = self.root;
        loop {
            let node = &self.nodes[best as usize];
            if node.is_leaf() {
                break;
            }

            let left = node.left;
            let right = node.right;
            let combined = node.aabb.merged(&fat_aabb);
            let combined_sa = combined.surface_area();
            let cost = 2.0 * combined_sa;
            let inherit_cost = 2.0 * (combined_sa - node.aabb.surface_area());

            let cost_left = {
                let merged = self.nodes[left as usize].aabb.merged(&fat_aabb);
                if self.nodes[left as usize].is_leaf() {
                    merged.surface_area() + inherit_cost
                } else {
                    merged.surface_area() - self.nodes[left as usize].aabb.surface_area()
                        + inherit_cost
                }
            };
            let cost_right = {
                let merged = self.nodes[right as usize].aabb.merged(&fat_aabb);
                if self.nodes[right as usize].is_leaf() {
                    merged.surface_area() + inherit_cost
                } else {
                    merged.surface_area() - self.nodes[right as usize].aabb.surface_area()
                        + inherit_cost
                }
            };

            if cost < cost_left && cost < cost_right {
                break;
            }

            best = if cost_left < cost_right { left } else { right };
        }

        // Create a new internal node as parent of the sibling and the new leaf
        let old_parent = self.nodes[best as usize].parent;
        let new_parent = self.alloc_node(Node {
            aabb: self.nodes[best as usize].aabb.merged(&fat_aabb),
            parent: old_parent,
            left: best,
            right: leaf,
            key: None,
            height: self.nodes[best as usize].height + 1,
        });

        self.nodes[best as usize].parent = new_parent;
        self.nodes[leaf as usize].parent = new_parent;

        if old_parent == NULL_NODE {
            self.root = new_parent;
        } else if self.nodes[old_parent as usize].left == best {
            self.nodes[old_parent as usize].left = new_parent;
        } else {
            self.nodes[old_parent as usize].right = new_parent;
        }

        // Walk back up fixing heights and AABBs
        self.fix_upwards(new_parent);

        leaf
    }

    /// Remove a leaf node by index.
    pub fn remove(&mut self, leaf: u32) {
        if leaf == self.root {
            self.root = NULL_NODE;
            self.free_node(leaf);
            return;
        }

        let parent = self.nodes[leaf as usize].parent;
        let grandparent = self.nodes[parent as usize].parent;
        let sibling = if self.nodes[parent as usize].left == leaf {
            self.nodes[parent as usize].right
        } else {
            self.nodes[parent as usize].left
        };

        if grandparent == NULL_NODE {
            self.root = sibling;
            self.nodes[sibling as usize].parent = NULL_NODE;
        } else {
            if self.nodes[grandparent as usize].left == parent {
                self.nodes[grandparent as usize].left = sibling;
            } else {
                self.nodes[grandparent as usize].right = sibling;
            }
            self.nodes[sibling as usize].parent = grandparent;
            self.fix_upwards(grandparent);
        }

        self.free_node(parent);
        self.free_node(leaf);
    }

    /// Update a leaf's AABB. Only reinserts if the tight AABB escapes the fattened one.
    /// Returns true if the node was reinserted.
    pub fn update(&mut self, leaf: u32, tight_aabb: TreeAabb) -> bool {
        if self.nodes[leaf as usize].aabb.contains(&tight_aabb) {
            return false; // Still within fattened AABB
        }

        let key = match self.nodes[leaf as usize].key {
            Some(k) => k,
            None => return false, // Not a leaf node — no-op
        };
        self.remove(leaf);
        self.insert(key, tight_aabb);
        true
    }

    /// Query all unique overlapping leaf pairs.
    pub fn query_pairs(&self) -> BTreeSet<(K, K)> {
        let mut pairs = BTreeSet::new();
        if self.root == NULL_NODE {
            return pairs;
        }

        // Collect all leaves
        let mut leaves = Vec::new();
        self.collect_leaves(self.root, &mut leaves);

        // For each leaf, query the tree for overlaps
        for &leaf in &leaves {
            let aabb = &self.nodes[leaf as usize].aabb;
            let key_a = match self.nodes[leaf as usize].key {
                Some(k) => k,
                None => continue,
            };
            self.query_overlap_recursive(self.root, aabb, key_a, leaf, &mut pairs);
        }

        pairs
    }

    fn query_overlap_recursive(
        &self,
        node: u32,
        query_aabb: &TreeAabb,
        query_key: K,
        query_leaf: u32,
        pairs: &mut BTreeSet<(K, K)>,
    ) {
        if node == NULL_NODE {
            return;
        }

        let n = &self.nodes[node as usize];
        if !n.aabb.overlaps(query_aabb) {
            return;
        }

        if n.is_leaf() {
            if node != query_leaf
                && let Some(key_b) = n.key
            {
                let pair = if query_key < key_b {
                    (query_key, key_b)
                } else {
                    (key_b, query_key)
                };
                pairs.insert(pair);
            }
            return;
        }

        self.query_overlap_recursive(n.left, query_aabb, query_key, query_leaf, pairs);
        self.query_overlap_recursive(n.right, query_aabb, query_key, query_leaf, pairs);
    }

    fn collect_leaves(&self, node: u32, leaves: &mut Vec<u32>) {
        if node == NULL_NODE {
            return;
        }
        let n = &self.nodes[node as usize];
        if n.is_leaf() {
            leaves.push(node);
        } else {
            self.collect_leaves(n.left, leaves);
            self.collect_leaves(n.right, leaves);
        }
    }

    fn alloc_node(&mut self, node: Node<K>) -> u32 {
        if let Some(idx) = self.free_list.pop() {
            self.nodes[idx as usize] = node;
            idx
        } else {
            let idx = self.nodes.len() as u32;
            self.nodes.push(node);
            idx
        }
    }

    fn free_node(&mut self, idx: u32) {
        self.free_list.push(idx);
    }

    fn fix_upwards(&mut self, mut node: u32) {
        while node != NULL_NODE {
            let left = self.nodes[node as usize].left;
            let right = self.nodes[node as usize].right;
            if left == NULL_NODE || right == NULL_NODE {
                node = self.nodes[node as usize].parent;
                continue;
            }
            self.nodes[node as usize].height = 1 + self.nodes[left as usize]
                .height
                .max(self.nodes[right as usize].height);
            self.nodes[node as usize].aabb = self.nodes[left as usize]
                .aabb
                .merged(&self.nodes[right as usize].aabb);
            node = self.nodes[node as usize].parent;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn insert_and_query_2d() {
        let mut tree: AabbTree<u64> = AabbTree::new();
        tree.insert(0, TreeAabb::from_2d([0.0, 0.0], [1.0, 1.0]));
        tree.insert(1, TreeAabb::from_2d([0.5, 0.5], [1.5, 1.5]));
        tree.insert(2, TreeAabb::from_2d([10.0, 10.0], [11.0, 11.0]));

        let pairs = tree.query_pairs();
        assert!(pairs.contains(&(0, 1)));
        assert!(!pairs.contains(&(0, 2)));
        assert!(!pairs.contains(&(1, 2)));
    }

    #[test]
    fn insert_and_query_3d() {
        let mut tree: AabbTree<u64> = AabbTree::new();
        tree.insert(0, TreeAabb::from_3d([0.0, 0.0, 0.0], [1.0, 1.0, 1.0]));
        tree.insert(1, TreeAabb::from_3d([0.5, 0.5, 0.5], [1.5, 1.5, 1.5]));
        let pairs = tree.query_pairs();
        assert!(pairs.contains(&(0, 1)));
    }

    #[test]
    fn no_overlap() {
        let mut tree: AabbTree<u64> = AabbTree::new();
        tree.insert(0, TreeAabb::from_2d([0.0, 0.0], [1.0, 1.0]));
        tree.insert(1, TreeAabb::from_2d([5.0, 5.0], [6.0, 6.0]));
        let pairs = tree.query_pairs();
        assert!(pairs.is_empty());
    }

    #[test]
    fn remove_and_query() {
        let mut tree: AabbTree<u64> = AabbTree::new();
        let a = tree.insert(0, TreeAabb::from_2d([0.0, 0.0], [1.0, 1.0]));
        tree.insert(1, TreeAabb::from_2d([0.5, 0.5], [1.5, 1.5]));
        assert!(!tree.query_pairs().is_empty());

        tree.remove(a);
        assert!(tree.query_pairs().is_empty());
    }

    #[test]
    fn update_no_reinsert() {
        let mut tree: AabbTree<u64> = AabbTree::new();
        let a = tree.insert(0, TreeAabb::from_2d([0.0, 0.0], [1.0, 1.0]));
        // Small move within margin — should not reinsert
        let reinserted = tree.update(a, TreeAabb::from_2d([0.01, 0.01], [1.01, 1.01]));
        assert!(!reinserted);
    }

    #[test]
    fn empty_tree() {
        let tree: AabbTree<u64> = AabbTree::new();
        assert!(tree.query_pairs().is_empty());
    }

    #[test]
    fn single_node() {
        let mut tree: AabbTree<u64> = AabbTree::new();
        tree.insert(0, TreeAabb::from_2d([0.0, 0.0], [1.0, 1.0]));
        assert!(tree.query_pairs().is_empty());
    }

    #[test]
    fn many_overlapping() {
        let mut tree: AabbTree<u64> = AabbTree::new();
        for i in 0..10 {
            tree.insert(i, TreeAabb::from_2d([0.0, 0.0], [1.0, 1.0]));
        }
        let pairs = tree.query_pairs();
        // All 10 nodes overlap, so C(10,2) = 45 pairs
        assert_eq!(pairs.len(), 45);
    }
}

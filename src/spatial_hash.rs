//! Generic spatial hash grid for broadphase collision detection.
//!
//! Uses `(i32, i32, i32)` cell keys so the same structure works for both
//! 2D (z = 0) and 3D grids.

use std::collections::{BTreeMap, BTreeSet};

pub(crate) struct SpatialHashGrid<K: Copy + Eq + Ord> {
    inv_cell_size: f64,
    cells: BTreeMap<(i32, i32, i32), Vec<K>>,
}

impl<K: Copy + Eq + Ord> SpatialHashGrid<K> {
    pub fn new(cell_size: f64) -> Self {
        Self {
            inv_cell_size: 1.0 / cell_size,
            cells: BTreeMap::new(),
        }
    }

    /// Auto-compute cell size from AABB max-dimensions.
    /// Uses 2x the average AABB max dimension, with a minimum of 0.1.
    pub fn auto_cell_size(max_dims: impl Iterator<Item = f64>, count: usize) -> f64 {
        if count == 0 {
            return 1.0;
        }
        let total: f64 = max_dims.sum();
        (total / count as f64 * 2.0).max(0.1)
    }

    fn cell(&self, x: f64, y: f64, z: f64) -> (i32, i32, i32) {
        (
            (x * self.inv_cell_size).floor() as i32,
            (y * self.inv_cell_size).floor() as i32,
            (z * self.inv_cell_size).floor() as i32,
        )
    }

    /// Insert a key into all cells overlapped by a 2D AABB (z = 0).
    #[allow(dead_code)]
    pub fn insert_2d(&mut self, key: K, min: [f64; 2], max: [f64; 2]) {
        let (min_cx, min_cy, _) = self.cell(min[0], min[1], 0.0);
        let (max_cx, max_cy, _) = self.cell(max[0], max[1], 0.0);

        for cx in min_cx..=max_cx {
            for cy in min_cy..=max_cy {
                self.cells.entry((cx, cy, 0)).or_default().push(key);
            }
        }
    }

    /// Insert a key into all cells overlapped by a 3D AABB.
    #[allow(dead_code)]
    pub fn insert_3d(&mut self, key: K, min: [f64; 3], max: [f64; 3]) {
        let (min_cx, min_cy, min_cz) = self.cell(min[0], min[1], min[2]);
        let (max_cx, max_cy, max_cz) = self.cell(max[0], max[1], max[2]);

        for cx in min_cx..=max_cx {
            for cy in min_cy..=max_cy {
                for cz in min_cz..=max_cz {
                    self.cells.entry((cx, cy, cz)).or_default().push(key);
                }
            }
        }
    }

    /// Return all unique pairs of keys that share at least one cell.
    /// Pairs are canonically ordered (smaller < larger).
    pub fn query_pairs(&self) -> BTreeSet<(K, K)> {
        let mut pairs = BTreeSet::new();

        for cell in self.cells.values() {
            for i in 0..cell.len() {
                for j in (i + 1)..cell.len() {
                    let a = cell[i];
                    let b = cell[j];
                    if a < b {
                        pairs.insert((a, b));
                    } else {
                        pairs.insert((b, a));
                    }
                }
            }
        }

        pairs
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn insert_2d_and_query() {
        let mut grid: SpatialHashGrid<u64> = SpatialHashGrid::new(1.0);
        grid.insert_2d(0, [0.0, 0.0], [0.5, 0.5]);
        grid.insert_2d(1, [0.2, 0.2], [0.8, 0.8]);
        let pairs = grid.query_pairs();
        assert!(pairs.contains(&(0, 1)));
    }

    #[test]
    fn insert_2d_no_overlap() {
        let mut grid: SpatialHashGrid<u64> = SpatialHashGrid::new(1.0);
        grid.insert_2d(0, [0.0, 0.0], [0.5, 0.5]);
        grid.insert_2d(1, [10.0, 10.0], [10.5, 10.5]);
        assert!(grid.query_pairs().is_empty());
    }

    #[test]
    fn insert_3d_and_query() {
        let mut grid: SpatialHashGrid<u64> = SpatialHashGrid::new(2.0);
        grid.insert_3d(0, [0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        grid.insert_3d(1, [0.5, 0.5, 0.5], [1.5, 1.5, 1.5]);
        let pairs = grid.query_pairs();
        assert!(pairs.contains(&(0, 1)));
    }

    #[test]
    fn insert_3d_no_overlap() {
        let mut grid: SpatialHashGrid<u64> = SpatialHashGrid::new(1.0);
        grid.insert_3d(0, [0.0, 0.0, 0.0], [0.5, 0.5, 0.5]);
        grid.insert_3d(1, [10.0, 10.0, 10.0], [10.5, 10.5, 10.5]);
        assert!(grid.query_pairs().is_empty());
    }

    #[test]
    fn auto_cell_size_empty() {
        let size = SpatialHashGrid::<u64>::auto_cell_size(std::iter::empty(), 0);
        assert!((size - 1.0).abs() < 1e-6);
    }

    #[test]
    fn auto_cell_size_values() {
        let dims = vec![1.0, 2.0];
        let size = SpatialHashGrid::<u64>::auto_cell_size(dims.into_iter(), 2);
        // avg = 1.5, *2 = 3.0
        assert!((size - 3.0).abs() < 1e-6);
    }
}

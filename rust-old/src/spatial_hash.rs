//! Generic spatial hash grid for broadphase collision detection.
//!
//! Uses `(i32, i32, i32)` cell keys so the same structure works for both
//! 2D (z = 0) and 3D grids.

use std::collections::{BTreeMap, BTreeSet};

/// Entries spanning more cells than this (huge, non-finite or NaN bounds)
/// are kept in an "oversized" list and paired with every other entry instead
/// of being rasterised, so one bad AABB can't stall the broadphase.
const MAX_CELLS_PER_ENTRY: f64 = 4096.0;

pub(crate) struct SpatialHashGrid<K: Copy + Eq + Ord> {
    inv_cell_size: f64,
    cells: BTreeMap<(i32, i32, i32), Vec<K>>,
    oversized: Vec<K>,
    all: Vec<K>,
}

impl<K: Copy + Eq + Ord> SpatialHashGrid<K> {
    #[must_use]
    pub fn new(cell_size: f64) -> Self {
        let safe_size = if cell_size.is_finite() && cell_size.abs() >= 1e-10 {
            cell_size.abs()
        } else {
            1.0
        };
        Self {
            inv_cell_size: 1.0 / safe_size,
            cells: BTreeMap::new(),
            oversized: Vec::new(),
            all: Vec::new(),
        }
    }

    /// Auto-compute cell size from AABB max-dimensions: twice the average of
    /// the finite, non-negative dimensions, at least 0.1 (1.0 if none).
    #[must_use]
    pub fn auto_cell_size(max_dims: impl Iterator<Item = f64>, _count: usize) -> f64 {
        let (total, n) = max_dims
            .filter(|d| d.is_finite() && *d >= 0.0)
            .fold((0.0, 0usize), |(t, n), d| (t + d, n + 1));
        if n == 0 {
            return 1.0;
        }
        (total / n as f64 * 2.0).max(0.1)
    }

    /// Inclusive cell index range of `[min, max]` on one axis, or `None` if
    /// it is non-finite, inverted, or outside the `i32` cell range.
    #[inline]
    fn axis_range(&self, min: f64, max: f64) -> Option<(i32, i32)> {
        let lo = (min * self.inv_cell_size).floor();
        let hi = (max * self.inv_cell_size).floor();
        let in_range = |v: f64| v >= i32::MIN as f64 && v <= i32::MAX as f64;
        (lo <= hi && in_range(lo) && in_range(hi)).then_some((lo as i32, hi as i32))
    }

    /// Cell ranges for an AABB, or `None` if it should be treated as oversized.
    fn cell_ranges(&self, min: [f64; 3], max: [f64; 3]) -> Option<[(i32, i32); 3]> {
        let rx = self.axis_range(min[0], max[0])?;
        let ry = self.axis_range(min[1], max[1])?;
        let rz = self.axis_range(min[2], max[2])?;
        let count = [rx, ry, rz]
            .iter()
            .map(|(lo, hi)| f64::from(*hi) - f64::from(*lo) + 1.0)
            .product::<f64>();
        (count <= MAX_CELLS_PER_ENTRY).then_some([rx, ry, rz])
    }

    fn insert(&mut self, key: K, min: [f64; 3], max: [f64; 3]) {
        self.all.push(key);
        let Some([(x0, x1), (y0, y1), (z0, z1)]) = self.cell_ranges(min, max) else {
            self.oversized.push(key);
            return;
        };
        for cx in x0..=x1 {
            for cy in y0..=y1 {
                for cz in z0..=z1 {
                    self.cells.entry((cx, cy, cz)).or_default().push(key);
                }
            }
        }
    }

    /// Insert a key into all cells overlapped by a 2D AABB (z = 0).
    #[allow(dead_code)]
    pub fn insert_2d(&mut self, key: K, min: [f64; 2], max: [f64; 2]) {
        self.insert(key, [min[0], min[1], 0.0], [max[0], max[1], 0.0]);
    }

    /// Insert a key into all cells overlapped by a 3D AABB.
    #[allow(dead_code)]
    pub fn insert_3d(&mut self, key: K, min: [f64; 3], max: [f64; 3]) {
        self.insert(key, min, max);
    }

    /// Return all unique pairs of keys that share at least one cell (plus
    /// every pair involving an oversized entry). Pairs are canonically ordered
    /// (smaller < larger); callers still verify actual AABB overlap.
    #[must_use]
    pub fn query_pairs(&self) -> BTreeSet<(K, K)> {
        let mut pairs = BTreeSet::new();
        let ordered = |a: K, b: K| if a < b { (a, b) } else { (b, a) };

        for cell in self.cells.values() {
            for i in 0..cell.len() {
                for j in (i + 1)..cell.len() {
                    if cell[i] != cell[j] {
                        pairs.insert(ordered(cell[i], cell[j]));
                    }
                }
            }
        }
        for &o in &self.oversized {
            for &k in &self.all {
                if k != o {
                    pairs.insert(ordered(o, k));
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

    #[test]
    fn non_finite_and_huge_aabbs_do_not_rasterise() {
        let mut grid: SpatialHashGrid<u64> = SpatialHashGrid::new(0.1);
        grid.insert_2d(0, [f64::NAN, 0.0], [f64::NAN, 1.0]);
        grid.insert_2d(1, [-1e12, -1e12], [1e12, 1e12]);
        grid.insert_2d(2, [f64::NEG_INFINITY, 0.0], [f64::INFINITY, 1.0]);
        grid.insert_2d(3, [0.0, 0.0], [0.05, 0.05]);
        assert!(grid.cells.len() <= 1);
        let pairs = grid.query_pairs();
        // Oversized entries are paired with everything; the caller filters.
        assert!(pairs.contains(&(1, 3)) && pairs.contains(&(0, 3)));
    }

    #[test]
    fn auto_cell_size_ignores_non_finite() {
        let dims = vec![1.0, f64::NAN, f64::NEG_INFINITY, 3.0];
        let size = SpatialHashGrid::<u64>::auto_cell_size(dims.into_iter(), 4);
        assert!((size - 4.0).abs() < 1e-12);
    }
}

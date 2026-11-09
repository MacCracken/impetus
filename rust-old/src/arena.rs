//! Generational arena — O(1) insert, remove, and lookup by index.
//!
//! Replaces HashMap<Handle, T> in the physics backends for body, collider,
//! and joint storage. Handles encode an index + generation to detect
//! use-after-free.

/// A handle into an arena. Encodes index (lower 32 bits) and generation (upper 32 bits).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(crate) struct ArenaHandle(pub(crate) u64);

impl ArenaHandle {
    pub fn new(index: u32, generation: u32) -> Self {
        Self((generation as u64) << 32 | index as u64)
    }

    pub fn index(self) -> u32 {
        self.0 as u32
    }

    pub fn generation(self) -> u32 {
        (self.0 >> 32) as u32
    }
}

/// Entry in the arena — either occupied or free (pointing to next free slot).
enum Entry<T> {
    Occupied {
        value: T,
        generation: u32,
    },
    Free {
        next_free: Option<u32>,
        generation: u32,
    },
}

/// A generational arena for O(1) operations.
pub(crate) struct Arena<T> {
    entries: Vec<Entry<T>>,
    free_head: Option<u32>,
    len: usize,
    /// Set by `insert_at`: the free list must be rebuilt before the next
    /// `insert` (keeps bulk restore O(n) instead of O(n²)).
    free_list_stale: bool,
}

#[allow(dead_code)]
impl<T> Arena<T> {
    pub fn new() -> Self {
        Self {
            entries: Vec::new(),
            free_head: None,
            len: 0,
            free_list_stale: false,
        }
    }

    /// Insert a value, returning its handle.
    pub fn insert(&mut self, value: T) -> ArenaHandle {
        if self.free_list_stale {
            self.rebuild_free_list();
        }
        self.len += 1;

        // Reuse a free slot if the free list points at one.
        if let Some(free_idx) = self.free_head
            && let Some(Entry::Free {
                next_free,
                generation,
            }) = self.entries.get(free_idx as usize)
        {
            let generation = *generation;
            self.free_head = *next_free;
            self.entries[free_idx as usize] = Entry::Occupied { value, generation };
            return ArenaHandle::new(free_idx, generation);
        }

        // Append a new slot.
        let idx = self.entries.len() as u32;
        self.entries.push(Entry::Occupied {
            value,
            generation: 0,
        });
        ArenaHandle::new(idx, 0)
    }

    /// Remove a value by handle. Returns the value if the handle was valid.
    pub fn remove(&mut self, handle: ArenaHandle) -> Option<T> {
        let idx = handle.index() as usize;
        if idx >= self.entries.len() {
            return None;
        }

        match &self.entries[idx] {
            Entry::Occupied { generation, .. } if *generation == handle.generation() => {}
            _ => return None,
        }

        // Swap out the entry
        let new_gen = handle.generation().wrapping_add(1);
        let old = std::mem::replace(
            &mut self.entries[idx],
            Entry::Free {
                next_free: self.free_head,
                generation: new_gen,
            },
        );
        self.free_head = Some(handle.index());
        self.len -= 1;

        match old {
            Entry::Occupied { value, .. } => Some(value),
            Entry::Free { .. } => None,
        }
    }

    /// Get a reference to a value by handle.
    pub fn get(&self, handle: ArenaHandle) -> Option<&T> {
        let idx = handle.index() as usize;
        match self.entries.get(idx)? {
            Entry::Occupied { value, generation } if *generation == handle.generation() => {
                Some(value)
            }
            _ => None,
        }
    }

    /// Get a mutable reference to a value by handle.
    pub fn get_mut(&mut self, handle: ArenaHandle) -> Option<&mut T> {
        let idx = handle.index() as usize;
        match self.entries.get_mut(idx)? {
            Entry::Occupied { value, generation } if *generation == handle.generation() => {
                Some(value)
            }
            _ => None,
        }
    }

    /// Number of occupied entries.
    pub fn len(&self) -> usize {
        self.len
    }

    /// Iterate over all occupied values.
    pub fn values(&self) -> impl Iterator<Item = &T> {
        self.entries.iter().filter_map(|entry| match entry {
            Entry::Occupied { value, .. } => Some(value),
            Entry::Free { .. } => None,
        })
    }

    /// Iterate over all occupied values mutably.
    pub fn values_mut(&mut self) -> impl Iterator<Item = &mut T> {
        self.entries.iter_mut().filter_map(|entry| match entry {
            Entry::Occupied { value, .. } => Some(value),
            Entry::Free { .. } => None,
        })
    }

    /// Iterate over (handle, value) pairs.
    pub fn iter(&self) -> impl Iterator<Item = (ArenaHandle, &T)> {
        self.entries
            .iter()
            .enumerate()
            .filter_map(|(idx, entry)| match entry {
                Entry::Occupied { value, generation } => {
                    Some((ArenaHandle::new(idx as u32, *generation), value))
                }
                Entry::Free { .. } => None,
            })
    }

    /// Allocation state for the oracle dump (see `crate::oracle`).
    #[cfg(feature = "oracle")]
    pub fn oracle_dump(&self) -> crate::oracle::ArenaDump {
        let slots = self
            .entries
            .iter()
            .map(|entry| match entry {
                Entry::Occupied { generation, .. } => crate::oracle::SlotDump {
                    occupied: true,
                    generation: *generation,
                    next_free: None,
                },
                Entry::Free {
                    next_free,
                    generation,
                } => crate::oracle::SlotDump {
                    occupied: false,
                    generation: *generation,
                    next_free: *next_free,
                },
            })
            .collect();
        crate::oracle::ArenaDump {
            slots,
            free_head: self.free_head,
            free_list_stale: self.free_list_stale,
        }
    }

    /// Check if a handle is valid.
    pub fn contains(&self, handle: ArenaHandle) -> bool {
        self.get(handle).is_some()
    }

    /// Insert a value at a specific handle (index + generation).
    /// Used for snapshot restore where handles must be preserved.
    /// Grows the entries vector if needed (new gaps are free slots of
    /// generation 0). Returns false if the slot is already occupied.
    ///
    /// The free list is rebuilt lazily on the next [`insert`](Self::insert),
    /// so restoring `n` entries is O(n + max index). Callers must bound the
    /// index: it allocates up to `index + 1` slots.
    pub fn insert_at(&mut self, handle: ArenaHandle, value: T) -> bool {
        let idx = handle.index() as usize;
        if idx >= self.entries.len() {
            self.entries.resize_with(idx + 1, || Entry::Free {
                next_free: None,
                generation: 0,
            });
        } else if matches!(self.entries[idx], Entry::Occupied { .. }) {
            return false;
        }
        self.entries[idx] = Entry::Occupied {
            value,
            generation: handle.generation(),
        };
        self.len += 1;
        self.free_list_stale = true;
        true
    }

    /// Rebuild the free list from the entries (lowest index first).
    fn rebuild_free_list(&mut self) {
        self.free_head = None;
        for i in (0..self.entries.len()).rev() {
            if let Entry::Free { next_free, .. } = &mut self.entries[i] {
                *next_free = self.free_head;
                self.free_head = Some(i as u32);
            }
        }
        self.free_list_stale = false;
    }

    /// Remove all entries for which the predicate returns `false`.
    pub fn retain<F>(&mut self, mut f: F)
    where
        F: FnMut(ArenaHandle, &T) -> bool,
    {
        for idx in 0..self.entries.len() {
            let should_remove = match &self.entries[idx] {
                Entry::Occupied { value, generation } => {
                    !f(ArenaHandle::new(idx as u32, *generation), value)
                }
                Entry::Free { .. } => false,
            };
            if should_remove {
                let Entry::Occupied { generation, .. } = &self.entries[idx] else {
                    continue;
                };
                let new_gen = generation.wrapping_add(1);
                self.entries[idx] = Entry::Free {
                    next_free: self.free_head,
                    generation: new_gen,
                };
                self.free_head = Some(idx as u32);
                self.len -= 1;
            }
        }
    }

    /// Iterate over (handle, value) pairs mutably.
    pub fn iter_mut(&mut self) -> impl Iterator<Item = (ArenaHandle, &mut T)> {
        self.entries
            .iter_mut()
            .enumerate()
            .filter_map(|(idx, entry)| match entry {
                Entry::Occupied { value, generation } => {
                    Some((ArenaHandle::new(idx as u32, *generation), value))
                }
                Entry::Free { .. } => None,
            })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn insert_and_get() {
        let mut arena = Arena::new();
        let h = arena.insert(42);
        assert_eq!(arena.get(h), Some(&42));
        assert_eq!(arena.len(), 1);
    }

    #[test]
    fn remove_and_reuse() {
        let mut arena = Arena::new();
        let h1 = arena.insert(10);
        let h2 = arena.insert(20);
        assert_eq!(arena.len(), 2);

        arena.remove(h1);
        assert_eq!(arena.len(), 1);
        assert_eq!(arena.get(h1), None); // stale handle

        let h3 = arena.insert(30); // reuses slot 0
        assert_eq!(h3.index(), h1.index()); // same index
        assert_ne!(h3.generation(), h1.generation()); // different generation
        assert_eq!(arena.get(h3), Some(&30));
        assert_eq!(arena.get(h1), None); // old handle still invalid
        assert_eq!(arena.get(h2), Some(&20));
    }

    #[test]
    fn stale_handle_rejected() {
        let mut arena = Arena::new();
        let h = arena.insert(99);
        arena.remove(h);
        assert_eq!(arena.get(h), None);
        assert_eq!(arena.get_mut(h), None);
        assert!(!arena.contains(h));
    }

    #[test]
    fn values_iteration() {
        let mut arena = Arena::new();
        arena.insert(1);
        arena.insert(2);
        arena.insert(3);
        let vals: Vec<_> = arena.values().copied().collect();
        assert_eq!(vals.len(), 3);
        assert!(vals.contains(&1));
        assert!(vals.contains(&2));
        assert!(vals.contains(&3));
    }

    #[test]
    fn values_after_removal() {
        let mut arena = Arena::new();
        arena.insert(1);
        let h2 = arena.insert(2);
        arena.insert(3);
        arena.remove(h2);
        let vals: Vec<_> = arena.values().copied().collect();
        assert_eq!(vals.len(), 2);
        assert!(vals.contains(&1));
        assert!(vals.contains(&3));
    }

    #[test]
    fn iter_handles() {
        let mut arena = Arena::new();
        let h1 = arena.insert(10);
        let h2 = arena.insert(20);
        let pairs: Vec<_> = arena.iter().collect();
        assert_eq!(pairs.len(), 2);
        assert!(pairs.iter().any(|(h, v)| *h == h1 && **v == 10));
        assert!(pairs.iter().any(|(h, v)| *h == h2 && **v == 20));
    }

    #[test]
    fn mut_access() {
        let mut arena = Arena::new();
        let h = arena.insert(5);
        *arena.get_mut(h).unwrap() = 50;
        assert_eq!(arena.get(h), Some(&50));
    }

    #[test]
    fn out_of_bounds_handle() {
        let arena: Arena<i32> = Arena::new();
        let bogus = ArenaHandle::new(999, 0);
        assert_eq!(arena.get(bogus), None);
    }

    #[test]
    fn double_remove() {
        let mut arena = Arena::new();
        let h = arena.insert(1);
        assert!(arena.remove(h).is_some());
        assert!(arena.remove(h).is_none()); // second remove returns None
    }

    #[test]
    fn heavy_churn() {
        let mut arena = Arena::new();
        let mut handles = Vec::new();
        // Insert 100
        for i in 0..100 {
            handles.push(arena.insert(i));
        }
        assert_eq!(arena.len(), 100);
        // Remove odd indices
        for i in (1..100).step_by(2) {
            arena.remove(handles[i]);
        }
        assert_eq!(arena.len(), 50);
        // Insert 50 more (should reuse slots)
        for i in 100..150 {
            arena.insert(i);
        }
        assert_eq!(arena.len(), 100);
    }
}

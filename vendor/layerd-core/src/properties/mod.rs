pub mod graph_properties;
pub mod internal;

use std::any::{Any, TypeId};

use hashbrown::HashMap;

/// A typed property key that uses a marker type for identity and a default value factory.
pub struct PropertyKey<T: Clone + 'static> {
    pub id: TypeId,
    pub default: fn() -> T,
}

impl<T: Clone + 'static> PropertyKey<T> {
    /// Create a property key using a marker type for identity.
    ///
    /// The marker type `Marker` is only used for its `TypeId`; it does not need to be
    /// instantiated. The default value is provided by the `default` function.
    pub fn of<Marker: 'static>(default: fn() -> T) -> Self {
        PropertyKey { id: TypeId::of::<Marker>(), default }
    }
}

/// Type-erased clone+downcast trait for [`PropertyMap`] values.
///
/// Carries a blanket impl for any `Any + Clone` so every property value keeps
/// its clone ability through type erasure, which is required to support
/// `LEdge::copy_properties`.
pub trait AnyClone: Any {
    /// Clone the underlying value and return a raw pointer to its boxed copy.
    ///
    /// The raw pointer form lets callers reconstruct a `Box<dyn AnyClone>`
    /// without making rustc's lifetime inference tie the box to `&self`,
    /// which was a persistent friction when returning `Box<dyn AnyClone>`
    /// directly through a dyn dispatch.
    fn clone_raw(&self) -> *mut ();
}

impl<T: Any + Clone> AnyClone for T {
    fn clone_raw(&self) -> *mut () {
        Box::into_raw(Box::new(self.clone())) as *mut ()
    }
}

/// Clone a `&dyn AnyClone` back to an owned `Box<dyn AnyClone>` by
/// reconstructing the fat pointer with the source's vtable and the newly
/// cloned data pointer.
fn clone_any_box(v: &dyn AnyClone) -> Box<dyn AnyClone> {
    let data_ptr = v.clone_raw();
    let mut fat_ptr: *const dyn AnyClone = v;
    // SAFETY: Swap in the cloned data pointer while keeping the vtable from
    // the original trait object. The Box then owns the cloned T, which has
    // the same concrete type as v, so the vtable remains valid.
    unsafe {
        let data_slot = &mut fat_ptr as *mut *const dyn AnyClone as *mut *mut ();
        *data_slot = data_ptr;
        Box::from_raw(fat_ptr as *mut dyn AnyClone)
    }
}

/// Downcast a `&dyn AnyClone` to a concrete type via the `Any` supertrait
/// (trait upcasting coercion, stable since Rust 1.86).
fn downcast_ref<T: 'static>(value: &dyn AnyClone) -> Option<&T> {
    (value as &dyn Any).downcast_ref::<T>()
}

/// A map from property keys to values, stored as type-erased `Box<dyn AnyClone + 'static>`.
///
/// The map is lazily allocated — no heap allocation occurs until the first `set`.
pub struct PropertyMap {
    map: Option<Box<HashMap<TypeId, Box<dyn AnyClone + 'static>>>>,
}

impl PropertyMap {
    pub fn new() -> Self {
        PropertyMap { map: None }
    }

    /// Set a property value. Allocates the map on first use.
    pub fn set<T: Clone + 'static>(&mut self, key: &PropertyKey<T>, value: T) {
        let map = self.map.get_or_insert_with(|| Box::new(HashMap::new()));
        map.insert(key.id, Box::new(value));
    }

    /// Remove an explicitly stored property value, restoring the key's default.
    pub fn remove<T: Clone + 'static>(&mut self, key: &PropertyKey<T>) {
        let Some(map) = &mut self.map else {
            return;
        };
        map.remove(&key.id);
        if map.is_empty() {
            self.map = None;
        }
    }

    /// Get a clone of the property value, or the default if not set.
    pub fn get<T: Clone + 'static>(&self, key: &PropertyKey<T>) -> T {
        match &self.map {
            Some(map) => match map.get(&key.id) {
                Some(value) => downcast_ref::<T>(value.as_ref()).unwrap().clone(),
                None => (key.default)(),
            },
            None => (key.default)(),
        }
    }

    /// Get a copy of a Copy property value, or the default if not set.
    pub fn get_copy<T: Copy + Clone + 'static>(&self, key: &PropertyKey<T>) -> T {
        match &self.map {
            Some(map) => match map.get(&key.id) {
                Some(value) => *downcast_ref::<T>(value.as_ref()).unwrap(),
                None => (key.default)(),
            },
            None => (key.default)(),
        }
    }

    /// Get a reference to the property value, or None if not set.
    pub fn get_ref<T: Clone + 'static>(&self, key: &PropertyKey<T>) -> Option<&T> {
        let entry = self.map.as_ref()?.get(&key.id)?;
        downcast_ref::<T>(entry.as_ref())
    }

    /// Get a slice view of a `Vec<T>` property, or an empty slice if not set.
    ///
    /// Avoids the heap allocation that `get` incurs for Vec-typed properties.
    /// Only applicable to properties whose value type is `Vec<T>`.
    pub fn get_slice<T: Clone + 'static>(&self, key: &PropertyKey<Vec<T>>) -> &[T] {
        match &self.map {
            Some(map) => match map.get(&key.id) {
                Some(value) => downcast_ref::<Vec<T>>(value.as_ref()).unwrap().as_slice(),
                None => &[],
            },
            None => &[],
        }
    }

    /// Check if the property is set.
    pub fn has<T: Clone + 'static>(&self, key: &PropertyKey<T>) -> bool {
        match &self.map {
            Some(map) => map.contains_key(&key.id),
            None => false,
        }
    }

    /// Number of explicitly stored properties, excluding defaults.
    pub fn explicit_len(&self) -> usize {
        self.map.as_ref().map_or(0, |map| map.len())
    }
}

impl Clone for PropertyMap {
    fn clone(&self) -> Self {
        let cloned_map = self.map.as_ref().map(|m| {
            let mut new_map: HashMap<TypeId, Box<dyn AnyClone + 'static>> =
                HashMap::with_capacity(m.len());
            for (k, v) in m.iter() {
                new_map.insert(*k, clone_any_box(v.as_ref()));
            }
            Box::new(new_map)
        });
        PropertyMap { map: cloned_map }
    }
}

impl Default for PropertyMap {
    fn default() -> Self {
        Self::new()
    }
}

use std::panic;

use crate::accumulator::Accumulator;
use crate::decay_accumulator::DecayAccumulator;
use crate::decimation::DecimationFilter;
use crate::denoise::TemporalFilter;
use crate::event::Event;
use crate::hot_pixel::HotPixelFilter;
use crate::polarity::{PolarityFilter, PolarityMode};
use crate::refractory::RefractoryFilter;
use crate::roi::RoiFilter;
use crate::transform::{SpatialTransform, TransformType};

// --- TemporalFilter FFI ---

/// Creates a new temporal denoising filter.
///
/// Returns a null pointer if `width`, `height`, or `threshold_us` are invalid (zero or negative).
#[no_mangle]
pub extern "C" fn edvs_temporal_filter_create(
    width: u32,
    height: u32,
    threshold_us: i64,
) -> *mut TemporalFilter {
    let result = panic::catch_unwind(|| {
        Box::new(TemporalFilter::new(width, height, threshold_us))
    });
    match result {
        Ok(filter) => Box::into_raw(filter),
        Err(_) => std::ptr::null_mut(),
    }
}

/// # Safety
///
/// - `filter` must be a valid, non-null pointer returned by `edvs_temporal_filter_create`.
/// - `event` must be a valid, non-null pointer to an initialized `Event`.
/// - `filter` must not be used concurrently from multiple threads.
/// - The caller must not have called `edvs_temporal_filter_destroy` on `filter`.
#[no_mangle]
pub unsafe extern "C" fn edvs_temporal_filter_process(
    filter: *mut TemporalFilter,
    event: *const Event,
) -> bool {
    if filter.is_null() || event.is_null() {
        return false;
    }
    let filter = &mut *filter;
    let event = &*event;
    filter.filter(event)
}

/// # Safety
///
/// - `filter` must be a valid pointer returned by `edvs_temporal_filter_create`,
///   or null (in which case this is a no-op).
/// - After this call, `filter` is invalid and must not be used again.
/// - Must not be called concurrently with `edvs_temporal_filter_process` on the same pointer.
#[no_mangle]
pub unsafe extern "C" fn edvs_temporal_filter_destroy(filter: *mut TemporalFilter) {
    if !filter.is_null() {
        drop(Box::from_raw(filter));
    }
}

// --- HotPixelFilter FFI ---

/// Creates a new hot pixel filter.
///
/// Returns a null pointer if `width`, `height`, `window_us`, or `max_rate` are invalid
/// (zero or negative).
#[no_mangle]
pub extern "C" fn edvs_hot_pixel_filter_create(
    width: u32,
    height: u32,
    window_us: i64,
    max_rate: u32,
) -> *mut HotPixelFilter {
    let result = panic::catch_unwind(|| {
        Box::new(HotPixelFilter::new(width, height, window_us, max_rate))
    });
    match result {
        Ok(filter) => Box::into_raw(filter),
        Err(_) => std::ptr::null_mut(),
    }
}

/// # Safety
///
/// - `filter` must be a valid, non-null pointer returned by `edvs_hot_pixel_filter_create`.
/// - `event` must be a valid, non-null pointer to an initialized `Event`.
/// - `filter` must not be used concurrently from multiple threads.
/// - The caller must not have called `edvs_hot_pixel_filter_destroy` on `filter`.
#[no_mangle]
pub unsafe extern "C" fn edvs_hot_pixel_filter_process(
    filter: *mut HotPixelFilter,
    event: *const Event,
) -> bool {
    if filter.is_null() || event.is_null() {
        return false;
    }
    let filter = &mut *filter;
    let event = &*event;
    filter.filter(event)
}

/// # Safety
///
/// - `filter` must be a valid pointer returned by `edvs_hot_pixel_filter_create`,
///   or null (in which case this is a no-op).
/// - After this call, `filter` is invalid and must not be used again.
/// - Must not be called concurrently with `edvs_hot_pixel_filter_process` on the same pointer.
#[no_mangle]
pub unsafe extern "C" fn edvs_hot_pixel_filter_destroy(filter: *mut HotPixelFilter) {
    if !filter.is_null() {
        drop(Box::from_raw(filter));
    }
}

// --- Accumulator FFI ---

/// Creates a new event accumulator.
///
/// Returns a null pointer if `width` or `height` are zero.
#[no_mangle]
pub extern "C" fn edvs_accumulator_create(width: u32, height: u32) -> *mut Accumulator {
    let result = panic::catch_unwind(|| {
        Box::new(Accumulator::new(width, height))
    });
    match result {
        Ok(acc) => Box::into_raw(acc),
        Err(_) => std::ptr::null_mut(),
    }
}

/// # Safety
///
/// - `acc` must be a valid, non-null pointer returned by `edvs_accumulator_create`.
/// - `event` must be a valid, non-null pointer to an initialized `Event`.
/// - `acc` must not be used concurrently from multiple threads.
/// - The caller must not have called `edvs_accumulator_destroy` on `acc`.
#[no_mangle]
pub unsafe extern "C" fn edvs_accumulator_accumulate(
    acc: *mut Accumulator,
    event: *const Event,
) {
    if acc.is_null() || event.is_null() {
        return;
    }
    let acc = &mut *acc;
    let event = &*event;
    acc.accumulate(event);
}

/// # Safety
///
/// - `acc` must be a valid, non-null pointer returned by `edvs_accumulator_create`.
/// - The returned pointer borrows internal memory of the `Accumulator`. It is valid
///   only until the next call to `edvs_accumulator_accumulate`, `edvs_accumulator_reset`,
///   or `edvs_accumulator_destroy` on the same `acc`.
/// - `out_len`, if non-null, will be written with the frame length in bytes.
/// - The caller must copy the frame data if it needs to persist beyond the next mutation.
#[no_mangle]
pub unsafe extern "C" fn edvs_accumulator_get_frame(
    acc: *const Accumulator,
    out_len: *mut usize,
) -> *const u8 {
    if acc.is_null() {
        return std::ptr::null();
    }
    let acc = &*acc;
    let frame = acc.get_frame();
    if !out_len.is_null() {
        *out_len = frame.len();
    }
    frame.as_ptr()
}

/// # Safety
///
/// - `acc` must be a valid, non-null pointer returned by `edvs_accumulator_create`,
///   or null (in which case this is a no-op).
/// - Any frame pointers previously obtained via `edvs_accumulator_get_frame` are
///   invalidated after this call.
#[no_mangle]
pub unsafe extern "C" fn edvs_accumulator_reset(acc: *mut Accumulator) {
    if !acc.is_null() {
        (*acc).reset();
    }
}

/// # Safety
///
/// - `acc` must be a valid pointer returned by `edvs_accumulator_create`,
///   or null (in which case this is a no-op).
/// - After this call, `acc` is invalid and must not be used again.
/// - Any frame pointers previously obtained via `edvs_accumulator_get_frame` are
///   invalidated after this call.
#[no_mangle]
pub unsafe extern "C" fn edvs_accumulator_destroy(acc: *mut Accumulator) {
    if !acc.is_null() {
        drop(Box::from_raw(acc));
    }
}

// --- RefractoryFilter FFI ---

/// Creates a new refractory period filter.
///
/// Returns null if parameters are invalid (zero dimensions or non-positive period).
#[no_mangle]
pub extern "C" fn edvs_refractory_filter_create(
    width: u32,
    height: u32,
    refractory_period_us: i64,
) -> *mut RefractoryFilter {
    let result = panic::catch_unwind(|| {
        Box::new(RefractoryFilter::new(width, height, refractory_period_us))
    });
    match result {
        Ok(filter) => Box::into_raw(filter),
        Err(_) => std::ptr::null_mut(),
    }
}

/// # Safety
///
/// - `filter` must be a valid, non-null pointer returned by `edvs_refractory_filter_create`.
/// - `event` must be a valid, non-null pointer to an initialized `Event`.
#[no_mangle]
pub unsafe extern "C" fn edvs_refractory_filter_process(
    filter: *mut RefractoryFilter,
    event: *const Event,
) -> bool {
    if filter.is_null() || event.is_null() {
        return false;
    }
    let filter = &mut *filter;
    let event = &*event;
    filter.filter(event)
}

/// # Safety
///
/// - `filter` must be a valid pointer returned by `edvs_refractory_filter_create`,
///   or null (no-op).
#[no_mangle]
pub unsafe extern "C" fn edvs_refractory_filter_destroy(filter: *mut RefractoryFilter) {
    if !filter.is_null() {
        drop(Box::from_raw(filter));
    }
}

// --- PolarityFilter FFI ---

/// Creates a new polarity filter.
///
/// `mode`: 0 = OnOnly, 1 = OffOnly, 2 = Both (default for invalid values).
#[no_mangle]
pub extern "C" fn edvs_polarity_filter_create(mode: i32) -> *mut PolarityFilter {
    let result = panic::catch_unwind(|| {
        Box::new(PolarityFilter::new(PolarityMode::from_i32(mode)))
    });
    match result {
        Ok(filter) => Box::into_raw(filter),
        Err(_) => std::ptr::null_mut(),
    }
}

/// # Safety
///
/// - `filter` must be a valid, non-null pointer returned by `edvs_polarity_filter_create`.
/// - `event` must be a valid, non-null pointer to an initialized `Event`.
#[no_mangle]
pub unsafe extern "C" fn edvs_polarity_filter_process(
    filter: *const PolarityFilter,
    event: *const Event,
) -> bool {
    if filter.is_null() || event.is_null() {
        return false;
    }
    let filter = &*filter;
    let event = &*event;
    filter.filter(event)
}

/// # Safety
///
/// - `filter` must be a valid pointer returned by `edvs_polarity_filter_create`, or null.
#[no_mangle]
pub unsafe extern "C" fn edvs_polarity_filter_destroy(filter: *mut PolarityFilter) {
    if !filter.is_null() {
        drop(Box::from_raw(filter));
    }
}

// --- RoiFilter FFI ---

/// Creates a new ROI filter. Bounds are inclusive.
///
/// Returns null if `x_min > x_max` or `y_min > y_max`.
#[no_mangle]
pub extern "C" fn edvs_roi_filter_create(
    x_min: u16,
    y_min: u16,
    x_max: u16,
    y_max: u16,
) -> *mut RoiFilter {
    let result = panic::catch_unwind(|| {
        Box::new(RoiFilter::new(x_min, y_min, x_max, y_max))
    });
    match result {
        Ok(filter) => Box::into_raw(filter),
        Err(_) => std::ptr::null_mut(),
    }
}

/// # Safety
///
/// - `filter` must be a valid, non-null pointer returned by `edvs_roi_filter_create`.
/// - `event` must be a valid, non-null pointer to an initialized `Event`.
#[no_mangle]
pub unsafe extern "C" fn edvs_roi_filter_process(
    filter: *const RoiFilter,
    event: *const Event,
) -> bool {
    if filter.is_null() || event.is_null() {
        return false;
    }
    let filter = &*filter;
    let event = &*event;
    filter.filter(event)
}

/// # Safety
///
/// - `filter` must be a valid pointer returned by `edvs_roi_filter_create`, or null.
#[no_mangle]
pub unsafe extern "C" fn edvs_roi_filter_destroy(filter: *mut RoiFilter) {
    if !filter.is_null() {
        drop(Box::from_raw(filter));
    }
}

// --- DecimationFilter FFI ---

/// Creates a new decimation filter that passes every `factor`-th event.
///
/// Returns null if `factor` is 0.
#[no_mangle]
pub extern "C" fn edvs_decimation_filter_create(factor: u32) -> *mut DecimationFilter {
    let result = panic::catch_unwind(|| {
        Box::new(DecimationFilter::new(factor))
    });
    match result {
        Ok(filter) => Box::into_raw(filter),
        Err(_) => std::ptr::null_mut(),
    }
}

/// # Safety
///
/// - `filter` must be a valid, non-null pointer returned by `edvs_decimation_filter_create`.
/// - `event` must be a valid, non-null pointer to an initialized `Event`.
#[no_mangle]
pub unsafe extern "C" fn edvs_decimation_filter_process(
    filter: *mut DecimationFilter,
    event: *const Event,
) -> bool {
    if filter.is_null() || event.is_null() {
        return false;
    }
    let filter = &mut *filter;
    let event = &*event;
    filter.filter(event)
}

/// # Safety
///
/// - `filter` must be a valid pointer returned by `edvs_decimation_filter_create`, or null.
#[no_mangle]
pub unsafe extern "C" fn edvs_decimation_filter_destroy(filter: *mut DecimationFilter) {
    if !filter.is_null() {
        drop(Box::from_raw(filter));
    }
}

// --- SpatialTransform FFI ---

/// Creates a new spatial transform.
///
/// `transform_type`: 0=FlipH, 1=FlipV, 2=Transpose, 3=Rot90, 4=Rot180, 5=Rot270.
/// Returns null for invalid transform_type or zero dimensions.
#[no_mangle]
pub extern "C" fn edvs_spatial_transform_create(
    transform_type: i32,
    width: u16,
    height: u16,
) -> *mut SpatialTransform {
    let result = panic::catch_unwind(|| {
        let tt = TransformType::from_i32(transform_type)
            .expect("invalid transform type");
        Box::new(SpatialTransform::new(tt, width, height))
    });
    match result {
        Ok(t) => Box::into_raw(t),
        Err(_) => std::ptr::null_mut(),
    }
}

/// # Safety
///
/// - `transform` must be a valid, non-null pointer returned by `edvs_spatial_transform_create`.
/// - `event` must be a valid, non-null pointer to an initialized `Event`.
///   The event's coordinates are modified in-place.
#[no_mangle]
pub unsafe extern "C" fn edvs_spatial_transform_apply(
    transform: *const SpatialTransform,
    event: *mut Event,
) -> bool {
    if transform.is_null() || event.is_null() {
        return false;
    }
    let transform = &*transform;
    let event = &mut *event;
    transform.apply(event)
}

/// # Safety
///
/// - `transform` must be a valid pointer returned by `edvs_spatial_transform_create`, or null.
#[no_mangle]
pub unsafe extern "C" fn edvs_spatial_transform_destroy(transform: *mut SpatialTransform) {
    if !transform.is_null() {
        drop(Box::from_raw(transform));
    }
}

// --- DecayAccumulator FFI ---

/// Creates a new decay accumulator.
///
/// Returns null for invalid parameters (zero dimensions, non-positive decay/contribution,
/// neutral outside [min_val, max_val]).
#[no_mangle]
pub extern "C" fn edvs_decay_accumulator_create(
    width: u32,
    height: u32,
    decay_tau_us: f64,
    contribution: f64,
    neutral: f64,
    min_val: f64,
    max_val: f64,
) -> *mut DecayAccumulator {
    let result = panic::catch_unwind(|| {
        Box::new(DecayAccumulator::new(
            width, height, decay_tau_us, contribution, neutral, min_val, max_val,
        ))
    });
    match result {
        Ok(acc) => Box::into_raw(acc),
        Err(_) => std::ptr::null_mut(),
    }
}

/// # Safety
///
/// - `acc` must be a valid, non-null pointer returned by `edvs_decay_accumulator_create`.
/// - `event` must be a valid, non-null pointer to an initialized `Event`.
#[no_mangle]
pub unsafe extern "C" fn edvs_decay_accumulator_accumulate(
    acc: *mut DecayAccumulator,
    event: *const Event,
) {
    if acc.is_null() || event.is_null() {
        return;
    }
    let acc = &mut *acc;
    let event = &*event;
    acc.accumulate(event);
}

/// Returns a pointer to a heap-allocated u8 frame buffer and its length.
/// The caller must free the returned buffer with `edvs_decay_accumulator_free_frame`.
///
/// # Safety
///
/// - `acc` must be a valid, non-null pointer returned by `edvs_decay_accumulator_create`.
/// - `out_len` must be a valid, non-null pointer.
#[no_mangle]
pub unsafe extern "C" fn edvs_decay_accumulator_get_frame_u8(
    acc: *const DecayAccumulator,
    out_len: *mut usize,
) -> *mut u8 {
    if acc.is_null() || out_len.is_null() {
        return std::ptr::null_mut();
    }
    let acc = &*acc;
    let frame = acc.get_frame_u8().into_boxed_slice();
    *out_len = frame.len();
    Box::into_raw(frame) as *mut u8
}

/// Free a frame buffer allocated by `edvs_decay_accumulator_get_frame_u8`.
///
/// # Safety
///
/// - `ptr` must be a pointer returned by `edvs_decay_accumulator_get_frame_u8`, or null.
/// - `len` must be the length returned via `out_len`.
#[no_mangle]
pub unsafe extern "C" fn edvs_decay_accumulator_free_frame(ptr: *mut u8, len: usize) {
    if !ptr.is_null() && len > 0 {
        drop(Box::from_raw(std::ptr::slice_from_raw_parts_mut(ptr, len)));
    }
}

/// # Safety
///
/// - `acc` must be a valid, non-null pointer returned by `edvs_decay_accumulator_create`, or null.
#[no_mangle]
pub unsafe extern "C" fn edvs_decay_accumulator_reset(acc: *mut DecayAccumulator) {
    if !acc.is_null() {
        (*acc).reset();
    }
}

/// # Safety
///
/// - `acc` must be a valid pointer returned by `edvs_decay_accumulator_create`, or null.
#[no_mangle]
pub unsafe extern "C" fn edvs_decay_accumulator_destroy(acc: *mut DecayAccumulator) {
    if !acc.is_null() {
        drop(Box::from_raw(acc));
    }
}

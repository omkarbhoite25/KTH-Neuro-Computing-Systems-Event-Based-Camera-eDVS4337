use std::panic;

use crate::accumulator::Accumulator;
use crate::denoise::TemporalFilter;
use crate::event::Event;
use crate::hot_pixel::HotPixelFilter;

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

use crate::accumulator::Accumulator;
use crate::denoise::TemporalFilter;
use crate::event::Event;
use crate::hot_pixel::HotPixelFilter;

// --- TemporalFilter FFI ---

#[no_mangle]
pub extern "C" fn edvs_temporal_filter_create(
    width: u32,
    height: u32,
    threshold_us: i64,
) -> *mut TemporalFilter {
    let filter = Box::new(TemporalFilter::new(width, height, threshold_us));
    Box::into_raw(filter)
}

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

#[no_mangle]
pub unsafe extern "C" fn edvs_temporal_filter_destroy(filter: *mut TemporalFilter) {
    if !filter.is_null() {
        drop(Box::from_raw(filter));
    }
}

// --- HotPixelFilter FFI ---

#[no_mangle]
pub extern "C" fn edvs_hot_pixel_filter_create(
    width: u32,
    height: u32,
    window_us: i64,
    max_rate: u32,
) -> *mut HotPixelFilter {
    let filter = Box::new(HotPixelFilter::new(width, height, window_us, max_rate));
    Box::into_raw(filter)
}

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

#[no_mangle]
pub unsafe extern "C" fn edvs_hot_pixel_filter_destroy(filter: *mut HotPixelFilter) {
    if !filter.is_null() {
        drop(Box::from_raw(filter));
    }
}

// --- Accumulator FFI ---

#[no_mangle]
pub extern "C" fn edvs_accumulator_create(width: u32, height: u32) -> *mut Accumulator {
    let acc = Box::new(Accumulator::new(width, height));
    Box::into_raw(acc)
}

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

#[no_mangle]
pub unsafe extern "C" fn edvs_accumulator_reset(acc: *mut Accumulator) {
    if !acc.is_null() {
        (*acc).reset();
    }
}

#[no_mangle]
pub unsafe extern "C" fn edvs_accumulator_destroy(acc: *mut Accumulator) {
    if !acc.is_null() {
        drop(Box::from_raw(acc));
    }
}

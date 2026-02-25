use edvs_processing::accumulator::Accumulator;
use edvs_processing::denoise::TemporalFilter;
use edvs_processing::event::Event;
use edvs_processing::ffi;
use edvs_processing::hot_pixel::HotPixelFilter;

// ---------------------------------------------------------------------------
// Full pipeline integration tests: temporal → hot pixel → accumulator
// ---------------------------------------------------------------------------

#[test]
fn test_full_pipeline_normal_events() {
    let mut temporal = TemporalFilter::new(128, 128, 5000);
    let mut hot_pixel = HotPixelFilter::new(128, 128, 1_000_000, 100);
    let mut acc = Accumulator::new(128, 128);

    // Two neighboring events within threshold — second should pass temporal
    let ev1 = Event::new(10, 10, 100_000, 1);
    let ev2 = Event::new(11, 10, 102_000, -1);

    temporal.filter(&ev1); // rejected (isolated), but timestamps recorded
    let pass_temporal = temporal.filter(&ev2);
    assert!(pass_temporal, "neighbor event should pass temporal filter");

    let pass_hot = hot_pixel.filter(&ev2);
    assert!(pass_hot, "normal-rate event should pass hot pixel filter");

    acc.accumulate(&ev2);
    // Pixel (11, 10): index = 10*128 + 11 = 1291. OFF event → 128 - 1 = 127
    assert_eq!(acc.get_frame()[10 * 128 + 11], 127);
}

#[test]
fn test_pipeline_noise_rejected() {
    let mut temporal = TemporalFilter::new(128, 128, 5000);
    let mut hot_pixel = HotPixelFilter::new(128, 128, 1_000_000, 100);
    let mut acc = Accumulator::new(128, 128);

    // Isolated noise events at scattered pixels — all should fail temporal
    let noise_events = vec![
        Event::new(5, 5, 100_000, 1),
        Event::new(50, 80, 200_000, -1),
        Event::new(120, 3, 300_000, 1),
    ];

    let mut passed = 0u32;
    for ev in &noise_events {
        if temporal.filter(ev) && hot_pixel.filter(ev) {
            acc.accumulate(ev);
            passed += 1;
        }
    }

    assert_eq!(passed, 0, "all isolated noise events should be rejected");
    // Accumulator untouched — all pixels at neutral 128
    assert!(acc.get_frame().iter().all(|&v| v == 128));
}

#[test]
fn test_pipeline_hot_pixel_rejected_after_window() {
    let mut temporal = TemporalFilter::new(4, 4, 50_000);
    let mut hot_pixel = HotPixelFilter::new(4, 4, 100_000, 3);
    let mut acc = Accumulator::new(4, 4);

    // Flood pixel (1,1) with many events in window 1.
    // Also send events to neighbor (2,1) so temporal filter passes them.
    for i in 0..10u32 {
        let ts = 10_000 + i as i64 * 1000;
        // Neighbor event first
        let neighbor = Event::new(2, 1, ts, 1);
        temporal.filter(&neighbor);
        hot_pixel.filter(&neighbor);

        // Hot pixel candidate
        let ev = Event::new(1, 1, ts + 500, 1);
        if temporal.filter(&ev) && hot_pixel.filter(&ev) {
            acc.accumulate(&ev);
        }
    }

    // Advance to next window — triggers hot pixel evaluation
    let trigger_neighbor = Event::new(2, 2, 200_000, 1);
    temporal.filter(&trigger_neighbor);
    hot_pixel.filter(&trigger_neighbor);

    let trigger = Event::new(3, 2, 200_500, 1);
    temporal.filter(&trigger);
    hot_pixel.filter(&trigger);

    // Now (1,1) should be flagged as hot in this window.
    // Send an event to (1,1) with a temporal neighbor so temporal passes it.
    let neighbor_for_hot = Event::new(0, 1, 201_000, 1);
    temporal.filter(&neighbor_for_hot);
    hot_pixel.filter(&neighbor_for_hot);

    let hot_ev = Event::new(1, 1, 201_500, 1);
    let pass_temporal = temporal.filter(&hot_ev);
    let pass_hot = hot_pixel.filter(&hot_ev);

    assert!(pass_temporal, "event has temporal neighbor, should pass temporal");
    assert!(!pass_hot, "pixel (1,1) should be flagged hot and rejected");
}

#[test]
fn test_pipeline_accumulator_frame_after_mixed_events() {
    let mut acc = Accumulator::new(4, 4);

    // Apply known events directly to accumulator
    // Pixel (0,0): 3 ON events → 128 + 3 = 131
    for i in 0..3 {
        acc.accumulate(&Event::new(0, 0, 1000 * i, 1));
    }

    // Pixel (3,3): 2 OFF events → 128 - 2 = 126
    for i in 0..2 {
        acc.accumulate(&Event::new(3, 3, 1000 * i, -1));
    }

    // Pixel (1,2): 1 ON + 1 OFF → 128 + 1 - 1 = 128 (neutral)
    acc.accumulate(&Event::new(1, 2, 1000, 1));
    acc.accumulate(&Event::new(1, 2, 2000, -1));

    let frame = acc.get_frame();
    assert_eq!(frame[0], 131);           // (0,0)
    assert_eq!(frame[3 * 4 + 3], 126);   // (3,3)
    assert_eq!(frame[2 * 4 + 1], 128);   // (1,2)
}

#[test]
fn test_pipeline_accumulator_clamp_after_many_events() {
    let mut acc = Accumulator::new(4, 4);

    // Saturate pixel (0,0) to 255
    for i in 0..200 {
        acc.accumulate(&Event::new(0, 0, i, 1));
    }
    assert_eq!(acc.get_frame()[0], 255);

    // Saturate pixel (1,0) to 0
    for i in 0..200 {
        acc.accumulate(&Event::new(1, 0, i, -1));
    }
    assert_eq!(acc.get_frame()[1], 0);

    // Reset and verify
    acc.reset();
    assert!(acc.get_frame().iter().all(|&v| v == 128));
}

// ---------------------------------------------------------------------------
// FFI null pointer safety tests
// ---------------------------------------------------------------------------

// Call FFI functions through the crate's public ffi module instead of
// re-declaring extern "C" blocks (which would trigger improper_ctypes
// warnings since the filter structs are opaque, non-repr(C) types).

#[test]
fn test_ffi_temporal_null_filter() {
    unsafe {
        let ev = Event::new(0, 0, 100, 1);
        let result = ffi::edvs_temporal_filter_process(std::ptr::null_mut(), &ev);
        assert!(!result, "null filter should return false");
    }
}

#[test]
fn test_ffi_temporal_null_event() {
    unsafe {
        let filter = ffi::edvs_temporal_filter_create(4, 4, 5000);
        assert!(!filter.is_null());

        let result = ffi::edvs_temporal_filter_process(filter, std::ptr::null());
        assert!(!result, "null event should return false");

        ffi::edvs_temporal_filter_destroy(filter);
    }
}

#[test]
fn test_ffi_temporal_destroy_null() {
    unsafe {
        // Should be a no-op, not a crash
        ffi::edvs_temporal_filter_destroy(std::ptr::null_mut());
    }
}

#[test]
fn test_ffi_hot_pixel_null_filter() {
    unsafe {
        let ev = Event::new(0, 0, 100, 1);
        let result = ffi::edvs_hot_pixel_filter_process(std::ptr::null_mut(), &ev);
        assert!(!result, "null filter should return false");
    }
}

#[test]
fn test_ffi_hot_pixel_destroy_null() {
    unsafe {
        ffi::edvs_hot_pixel_filter_destroy(std::ptr::null_mut());
    }
}

#[test]
fn test_ffi_accumulator_null_acc() {
    unsafe {
        let ev = Event::new(0, 0, 100, 1);
        // accumulate with null — should be a no-op
        ffi::edvs_accumulator_accumulate(std::ptr::null_mut(), &ev);

        // get_frame with null — should return null
        let mut len: usize = 0;
        let ptr = ffi::edvs_accumulator_get_frame(std::ptr::null(), &mut len);
        assert!(ptr.is_null());

        // reset with null — should be a no-op
        ffi::edvs_accumulator_reset(std::ptr::null_mut());

        // destroy with null — should be a no-op
        ffi::edvs_accumulator_destroy(std::ptr::null_mut());
    }
}

#[test]
fn test_ffi_accumulator_round_trip() {
    unsafe {
        let acc = ffi::edvs_accumulator_create(4, 4);
        assert!(!acc.is_null());

        let ev = Event::new(1, 1, 100, 1);
        ffi::edvs_accumulator_accumulate(acc, &ev);

        let mut len: usize = 0;
        let ptr = ffi::edvs_accumulator_get_frame(acc, &mut len);
        assert!(!ptr.is_null());
        assert_eq!(len, 16); // 4x4

        let frame = std::slice::from_raw_parts(ptr, len);
        assert_eq!(frame[5], 129); // pixel (1,1) = 128 + 1

        ffi::edvs_accumulator_reset(acc);

        let ptr2 = ffi::edvs_accumulator_get_frame(acc, &mut len);
        let frame2 = std::slice::from_raw_parts(ptr2, len);
        assert!(frame2.iter().all(|&v| v == 128));

        ffi::edvs_accumulator_destroy(acc);
    }
}

#[test]
fn test_ffi_create_invalid_params_returns_null() {
    // Zero dimensions should trigger assertion → catch_unwind → null
    let filter = ffi::edvs_temporal_filter_create(0, 128, 5000);
    assert!(filter.is_null(), "invalid params should return null");

    let hp = ffi::edvs_hot_pixel_filter_create(128, 0, 1_000_000, 100);
    assert!(hp.is_null(), "invalid params should return null");

    let acc = ffi::edvs_accumulator_create(0, 0);
    assert!(acc.is_null(), "invalid params should return null");
}

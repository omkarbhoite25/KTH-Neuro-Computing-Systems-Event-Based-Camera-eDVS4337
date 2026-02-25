use edvs_processing::accumulator::Accumulator;
use edvs_processing::anti_flicker::AntiFlickerFilter;
use edvs_processing::decay_accumulator::DecayAccumulator;
use edvs_processing::decimation::DecimationFilter;
use edvs_processing::denoise::TemporalFilter;
use edvs_processing::event::Event;
use edvs_processing::ffi;
use edvs_processing::hot_pixel::HotPixelFilter;
use edvs_processing::mask::MaskFilter;
use edvs_processing::polarity::{PolarityFilter, PolarityMode};
use edvs_processing::rate_stats::EventRateStats;
use edvs_processing::refractory::RefractoryFilter;
use edvs_processing::roi::RoiFilter;
use edvs_processing::slicer::{EventSlicer, SliceMode};
use edvs_processing::stc::StcFilter;
use edvs_processing::time_surface::TimeSurface;
use edvs_processing::transform::{SpatialTransform, TransformType};

use edvs_processing::corner::HarrisCornerDetector;
use edvs_processing::frequency::FrequencyEstimator;
use edvs_processing::optical_flow::OpticalFlowEstimator;
use edvs_processing::sits::SpeedInvariantTimeSurface;
use edvs_processing::voxel_grid::VoxelGrid;

// ---------------------------------------------------------------------------
// Full pipeline integration tests: temporal -> hot pixel -> accumulator
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
    // Pixel (11, 10): index = 10*128 + 11 = 1291. OFF event -> 128 - 1 = 127
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
    // Pixel (0,0): 3 ON events -> 128 + 3 = 131
    for i in 0..3 {
        acc.accumulate(&Event::new(0, 0, 1000 * i, 1));
    }

    // Pixel (3,3): 2 OFF events -> 128 - 2 = 126
    for i in 0..2 {
        acc.accumulate(&Event::new(3, 3, 1000 * i, -1));
    }

    // Pixel (1,2): 1 ON + 1 OFF -> 128 + 1 - 1 = 128 (neutral)
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
// Phase 1: Refractory filter integration
// ---------------------------------------------------------------------------

#[test]
fn test_pipeline_refractory_suppresses_rapid_events() {
    let mut temporal = TemporalFilter::new(128, 128, 5000);
    let mut refractory = RefractoryFilter::new(128, 128, 2000);
    let mut acc = Accumulator::new(128, 128);

    // Create temporal neighbor
    temporal.filter(&Event::new(10, 10, 100_000, 1));

    // Rapid burst at (11, 10) — only first should pass refractory
    let mut passed = 0u32;
    for i in 0..5 {
        let ev = Event::new(11, 10, 100_500 + i * 200, 1);
        if temporal.filter(&ev) && refractory.filter(&ev) {
            acc.accumulate(&ev);
            passed += 1;
        }
    }

    assert_eq!(passed, 1, "only first event of rapid burst should pass refractory");
}

#[test]
fn test_pipeline_refractory_allows_spaced_events() {
    let mut refractory = RefractoryFilter::new(128, 128, 1000);

    let ev1 = Event::new(10, 10, 100_000, 1);
    let ev2 = Event::new(10, 10, 102_000, 1); // 2000us apart > 1000us period

    assert!(refractory.filter(&ev1));
    assert!(refractory.filter(&ev2));
}

// ---------------------------------------------------------------------------
// Phase 1: Polarity filter integration
// ---------------------------------------------------------------------------

#[test]
fn test_pipeline_polarity_on_only() {
    let polarity = PolarityFilter::new(PolarityMode::OnOnly);
    let mut acc = Accumulator::new(4, 4);

    let events = vec![
        Event::new(0, 0, 100, 1),
        Event::new(1, 0, 200, -1),
        Event::new(2, 0, 300, 1),
        Event::new(3, 0, 400, -1),
    ];

    let mut on_count = 0u32;
    for ev in &events {
        if polarity.filter(ev) {
            acc.accumulate(ev);
            on_count += 1;
        }
    }

    assert_eq!(on_count, 2);
    assert_eq!(acc.get_frame()[0], 129); // ON
    assert_eq!(acc.get_frame()[1], 128); // OFF filtered out
    assert_eq!(acc.get_frame()[2], 129); // ON
    assert_eq!(acc.get_frame()[3], 128); // OFF filtered out
}

#[test]
fn test_pipeline_polarity_off_only() {
    let polarity = PolarityFilter::new(PolarityMode::OffOnly);

    assert!(!polarity.filter(&Event::new(0, 0, 100, 1)));
    assert!(polarity.filter(&Event::new(0, 0, 200, -1)));
}

// ---------------------------------------------------------------------------
// Phase 1: ROI filter integration
// ---------------------------------------------------------------------------

#[test]
fn test_pipeline_roi_filters_outside() {
    let roi = RoiFilter::new(20, 20, 60, 60);
    let mut acc = Accumulator::new(128, 128);

    let events = vec![
        Event::new(30, 30, 100, 1),  // inside
        Event::new(10, 10, 200, 1),  // outside
        Event::new(70, 70, 300, 1),  // outside
        Event::new(50, 50, 400, -1), // inside
    ];

    let mut passed = 0u32;
    for ev in &events {
        if roi.filter(ev) {
            acc.accumulate(ev);
            passed += 1;
        }
    }

    assert_eq!(passed, 2);
}

// ---------------------------------------------------------------------------
// Phase 1: Decimation filter integration
// ---------------------------------------------------------------------------

#[test]
fn test_pipeline_decimation_reduces_rate() {
    let mut decimation = DecimationFilter::new(5);
    let mut acc = Accumulator::new(128, 128);

    let mut passed = 0u32;
    for i in 0..100 {
        let ev = Event::new(10, 10, 100 + i * 10, 1);
        if decimation.filter(&ev) {
            acc.accumulate(&ev);
            passed += 1;
        }
    }

    assert_eq!(passed, 20); // 100 / 5 = 20
}

// ---------------------------------------------------------------------------
// Phase 1: Transform integration
// ---------------------------------------------------------------------------

#[test]
fn test_pipeline_transform_with_accumulator() {
    let transform = SpatialTransform::new(TransformType::FlipHorizontal, 4, 4);
    let mut acc = Accumulator::new(4, 4);

    let mut ev = Event::new(0, 0, 100, 1);
    assert!(transform.apply(&mut ev));
    // FlipH: x=0 -> x=3
    let (x, y) = (ev.x, ev.y);
    assert_eq!(x, 3);
    assert_eq!(y, 0);

    acc.accumulate(&ev);
    assert_eq!(acc.get_frame()[3], 129); // pixel (3,0) got the ON event
    assert_eq!(acc.get_frame()[0], 128); // original (0,0) untouched
}

#[test]
fn test_pipeline_transform_rotate_accumulate() {
    let transform = SpatialTransform::new(TransformType::Rotate180, 4, 4);
    let mut acc = Accumulator::new(4, 4);

    let mut ev = Event::new(1, 0, 100, 1);
    assert!(transform.apply(&mut ev));
    // Rot180: (1,0) -> (2,3)
    let (x, y) = (ev.x, ev.y);
    assert_eq!(x, 2);
    assert_eq!(y, 3);

    acc.accumulate(&ev);
    assert_eq!(acc.get_frame()[3 * 4 + 2], 129);
}

// ---------------------------------------------------------------------------
// Phase 1: Decay accumulator integration
// ---------------------------------------------------------------------------

#[test]
fn test_pipeline_decay_accumulator_fades() {
    let mut acc = DecayAccumulator::new(4, 4, 100_000.0, 10.0, 128.0, 0.0, 255.0);

    // Event at t=100000
    acc.accumulate(&Event::new(0, 0, 100_000, 1));
    let val_immediately = acc.get_values()[0];
    assert!((val_immediately - 138.0).abs() < 0.1); // 128 + 10

    // Much later event at same pixel — value should have decayed toward 128
    acc.accumulate(&Event::new(0, 0, 1_000_000, 1));
    // dt = 900000, tau = 100000, decay = exp(-9) ~ 0.000123
    // decayed = 128 + (138-128)*0.000123 ~ 128.00123
    // + 10 = 138.00123
    let val_later = acc.get_values()[0];
    assert!(
        (val_later - 138.0).abs() < 0.1,
        "after long time, value should be ~neutral+contribution: got {}",
        val_later
    );
}

#[test]
fn test_pipeline_decay_accumulator_u8_frame() {
    let mut acc = DecayAccumulator::new(4, 4, 100_000.0, 50.0, 128.0, 0.0, 255.0);
    acc.accumulate(&Event::new(0, 0, 100_000, 1)); // -> 178
    acc.accumulate(&Event::new(1, 0, 100_000, -1)); // -> 78

    let frame = acc.get_frame_u8();
    // 178 / 255 * 255 = 178
    assert_eq!(frame[0], 178);
    // 78 / 255 * 255 = 78
    assert_eq!(frame[1], 78);
}

// ---------------------------------------------------------------------------
// Phase 1: Combined pipeline — all filters chained
// ---------------------------------------------------------------------------

#[test]
fn test_full_extended_pipeline() {
    let mut temporal = TemporalFilter::new(128, 128, 5000);
    let mut hot_pixel = HotPixelFilter::new(128, 128, 1_000_000, 1000);
    let mut refractory = RefractoryFilter::new(128, 128, 500);
    let polarity = PolarityFilter::new(PolarityMode::OnOnly);
    let roi = RoiFilter::new(0, 0, 63, 63); // left half
    let mut decimation = DecimationFilter::new(1); // pass all
    let mut acc = Accumulator::new(128, 128);

    // Create neighbor for temporal filter
    temporal.filter(&Event::new(30, 30, 100_000, 1));

    // Event that should pass all filters
    let ev = Event::new(31, 30, 101_000, 1); // neighbor, ON, in ROI, first at pixel
    let passes = temporal.filter(&ev)
        && hot_pixel.filter(&ev)
        && refractory.filter(&ev)
        && polarity.filter(&ev)
        && roi.filter(&ev)
        && decimation.filter(&ev);

    assert!(passes, "valid event should pass all filters");
    acc.accumulate(&ev);
    assert_eq!(acc.get_frame()[30 * 128 + 31], 129);
}

#[test]
fn test_extended_pipeline_roi_rejects() {
    let roi = RoiFilter::new(0, 0, 63, 63);
    let polarity = PolarityFilter::new(PolarityMode::Both);

    // Event outside ROI
    let ev = Event::new(100, 100, 100_000, 1);
    assert!(polarity.filter(&ev)); // passes polarity
    assert!(!roi.filter(&ev));     // rejected by ROI
}

// ---------------------------------------------------------------------------
// FFI null pointer safety tests
// ---------------------------------------------------------------------------

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
    // Zero dimensions should trigger assertion -> catch_unwind -> null
    let filter = ffi::edvs_temporal_filter_create(0, 128, 5000);
    assert!(filter.is_null(), "invalid params should return null");

    let hp = ffi::edvs_hot_pixel_filter_create(128, 0, 1_000_000, 100);
    assert!(hp.is_null(), "invalid params should return null");

    let acc = ffi::edvs_accumulator_create(0, 0);
    assert!(acc.is_null(), "invalid params should return null");
}

// ---------------------------------------------------------------------------
// Phase 1: FFI null safety for new filters
// ---------------------------------------------------------------------------

#[test]
fn test_ffi_refractory_null_safety() {
    unsafe {
        let ev = Event::new(0, 0, 100, 1);
        assert!(!ffi::edvs_refractory_filter_process(std::ptr::null_mut(), &ev));
        ffi::edvs_refractory_filter_destroy(std::ptr::null_mut());
    }
}

#[test]
fn test_ffi_refractory_round_trip() {
    unsafe {
        let filter = ffi::edvs_refractory_filter_create(128, 128, 1000);
        assert!(!filter.is_null());

        let ev1 = Event::new(10, 10, 100_000, 1);
        assert!(ffi::edvs_refractory_filter_process(filter, &ev1));

        // Within refractory
        let ev2 = Event::new(10, 10, 100_500, 1);
        assert!(!ffi::edvs_refractory_filter_process(filter, &ev2));

        // Past refractory
        let ev3 = Event::new(10, 10, 101_500, 1);
        assert!(ffi::edvs_refractory_filter_process(filter, &ev3));

        ffi::edvs_refractory_filter_destroy(filter);
    }
}

#[test]
fn test_ffi_refractory_invalid_params() {
    let filter = ffi::edvs_refractory_filter_create(0, 128, 1000);
    assert!(filter.is_null());

    let filter = ffi::edvs_refractory_filter_create(128, 128, 0);
    assert!(filter.is_null());
}

#[test]
fn test_ffi_polarity_null_safety() {
    unsafe {
        let ev = Event::new(0, 0, 100, 1);
        assert!(!ffi::edvs_polarity_filter_process(std::ptr::null(), &ev));
        ffi::edvs_polarity_filter_destroy(std::ptr::null_mut());
    }
}

#[test]
fn test_ffi_polarity_round_trip() {
    unsafe {
        let filter = ffi::edvs_polarity_filter_create(0); // OnOnly
        assert!(!filter.is_null());

        assert!(ffi::edvs_polarity_filter_process(filter, &Event::new(0, 0, 100, 1)));
        assert!(!ffi::edvs_polarity_filter_process(filter, &Event::new(0, 0, 200, -1)));

        ffi::edvs_polarity_filter_destroy(filter);
    }
}

#[test]
fn test_ffi_roi_null_safety() {
    unsafe {
        let ev = Event::new(0, 0, 100, 1);
        assert!(!ffi::edvs_roi_filter_process(std::ptr::null(), &ev));
        ffi::edvs_roi_filter_destroy(std::ptr::null_mut());
    }
}

#[test]
fn test_ffi_roi_round_trip() {
    unsafe {
        let filter = ffi::edvs_roi_filter_create(10, 10, 50, 50);
        assert!(!filter.is_null());

        assert!(ffi::edvs_roi_filter_process(filter, &Event::new(30, 30, 100, 1)));
        assert!(!ffi::edvs_roi_filter_process(filter, &Event::new(5, 5, 100, 1)));

        ffi::edvs_roi_filter_destroy(filter);
    }
}

#[test]
fn test_ffi_roi_invalid_params() {
    let filter = ffi::edvs_roi_filter_create(50, 10, 10, 50); // x_min > x_max
    assert!(filter.is_null());
}

#[test]
fn test_ffi_decimation_null_safety() {
    unsafe {
        let ev = Event::new(0, 0, 100, 1);
        assert!(!ffi::edvs_decimation_filter_process(std::ptr::null_mut(), &ev));
        ffi::edvs_decimation_filter_destroy(std::ptr::null_mut());
    }
}

#[test]
fn test_ffi_decimation_round_trip() {
    unsafe {
        let filter = ffi::edvs_decimation_filter_create(3);
        assert!(!filter.is_null());

        let ev = Event::new(0, 0, 100, 1);
        assert!(!ffi::edvs_decimation_filter_process(filter, &ev)); // 1
        assert!(!ffi::edvs_decimation_filter_process(filter, &ev)); // 2
        assert!(ffi::edvs_decimation_filter_process(filter, &ev));  // 3 -> pass

        ffi::edvs_decimation_filter_destroy(filter);
    }
}

#[test]
fn test_ffi_decimation_invalid_params() {
    let filter = ffi::edvs_decimation_filter_create(0);
    assert!(filter.is_null());
}

#[test]
fn test_ffi_spatial_transform_null_safety() {
    unsafe {
        let mut ev = Event::new(0, 0, 100, 1);
        assert!(!ffi::edvs_spatial_transform_apply(std::ptr::null(), &mut ev));
        ffi::edvs_spatial_transform_destroy(std::ptr::null_mut());
    }
}

#[test]
fn test_ffi_spatial_transform_round_trip() {
    unsafe {
        let t = ffi::edvs_spatial_transform_create(0, 128, 128); // FlipHorizontal
        assert!(!t.is_null());

        let mut ev = Event::new(10, 20, 100, 1);
        assert!(ffi::edvs_spatial_transform_apply(t, &mut ev));
        let (x, y) = (ev.x, ev.y);
        assert_eq!(x, 117); // 128-1-10
        assert_eq!(y, 20);

        ffi::edvs_spatial_transform_destroy(t);
    }
}

#[test]
fn test_ffi_spatial_transform_invalid_type() {
    let t = ffi::edvs_spatial_transform_create(99, 128, 128);
    assert!(t.is_null());
}

#[test]
fn test_ffi_decay_accumulator_null_safety() {
    unsafe {
        let ev = Event::new(0, 0, 100, 1);
        ffi::edvs_decay_accumulator_accumulate(std::ptr::null_mut(), &ev);

        let mut len: usize = 0;
        let ptr = ffi::edvs_decay_accumulator_get_frame_u8(std::ptr::null(), &mut len);
        assert!(ptr.is_null());

        ffi::edvs_decay_accumulator_reset(std::ptr::null_mut());
        ffi::edvs_decay_accumulator_destroy(std::ptr::null_mut());
    }
}

#[test]
fn test_ffi_decay_accumulator_round_trip() {
    unsafe {
        let acc = ffi::edvs_decay_accumulator_create(4, 4, 100_000.0, 1.0, 128.0, 0.0, 255.0);
        assert!(!acc.is_null());

        let ev = Event::new(1, 1, 100_000, 1);
        ffi::edvs_decay_accumulator_accumulate(acc, &ev);

        let mut len: usize = 0;
        let ptr = ffi::edvs_decay_accumulator_get_frame_u8(acc, &mut len);
        assert!(!ptr.is_null());
        assert_eq!(len, 16);

        let frame = std::slice::from_raw_parts(ptr, len);
        // pixel (1,1) = idx 5: value=129, mapped to u8 via 129/255*255 = 129
        assert_eq!(frame[5], 129);

        ffi::edvs_decay_accumulator_free_frame(ptr, len);

        ffi::edvs_decay_accumulator_reset(acc);
        ffi::edvs_decay_accumulator_destroy(acc);
    }
}

#[test]
fn test_ffi_decay_accumulator_invalid_params() {
    let acc = ffi::edvs_decay_accumulator_create(0, 4, 100_000.0, 1.0, 128.0, 0.0, 255.0);
    assert!(acc.is_null());

    let acc = ffi::edvs_decay_accumulator_create(4, 4, -1.0, 1.0, 128.0, 0.0, 255.0);
    assert!(acc.is_null());
}

// =========================================================================
// Phase 2: Time Surface integration
// =========================================================================

#[test]
fn test_pipeline_time_surface_with_filters() {
    let mut temporal = TemporalFilter::new(128, 128, 5000);
    let mut ts = TimeSurface::new(128, 128, 100_000.0);

    // Create temporal neighbor
    temporal.filter(&Event::new(10, 10, 100_000, 1));
    ts.update(&Event::new(10, 10, 100_000, 1));

    let ev = Event::new(11, 10, 102_000, 1);
    if temporal.filter(&ev) {
        ts.update(&ev);
    }

    assert_eq!(ts.get_timestamp(11, 10), 102_000);
    let frame = ts.get_frame_at(102_000);
    // Recently updated pixel should be bright
    assert!(frame[10 * 128 + 11] > 200);
}

// =========================================================================
// Phase 2: Event Slicer integration
// =========================================================================

#[test]
fn test_pipeline_slicer_batches_events() {
    let mut slicer = EventSlicer::new(SliceMode::ByCount, 10);
    let mut batch_count = 0u32;

    for i in 0..50 {
        if slicer.process(&Event::new(0, 0, i * 1000, 1)) {
            batch_count += 1;
        }
    }

    assert_eq!(batch_count, 5);
}

#[test]
fn test_pipeline_slicer_time_window() {
    let mut slicer = EventSlicer::new(SliceMode::ByTime, 100_000);
    let mut batch_count = 0u32;

    for i in 0..500 {
        if slicer.process(&Event::new(0, 0, i * 1000, 1)) {
            batch_count += 1;
        }
    }

    assert!((3..=6).contains(&batch_count));
}

// =========================================================================
// Phase 2: Anti-Flicker integration
// =========================================================================

#[test]
fn test_pipeline_anti_flicker_rejects_periodic() {
    let mut anti_flicker = AntiFlickerFilter::new(128, 128, 10_000, 500);
    let mut acc = Accumulator::new(128, 128);

    // Simulate 50Hz flicker at pixel (10,10)
    let mut passed = 0u32;
    for i in 0..10 {
        let ev = Event::new(10, 10, 100_000 + i * 10_000, 1);
        if anti_flicker.filter(&ev) {
            acc.accumulate(&ev);
            passed += 1;
        }
    }

    // First event always passes, subsequent periodic events should be rejected
    assert!(passed < 5, "most periodic events should be rejected, got {}", passed);
}

// =========================================================================
// Phase 2: STC Filter integration
// =========================================================================

#[test]
fn test_pipeline_stc_polarity_aware() {
    let mut stc = StcFilter::new(128, 128, 5000);

    // ON event at (10,10)
    stc.filter(&Event::new(10, 10, 100_000, 1));

    // Same polarity neighbor — should pass
    assert!(stc.filter(&Event::new(11, 10, 102_000, 1)));

    // Reset with OFF event at (20,20)
    stc.filter(&Event::new(20, 20, 200_000, -1));

    // ON event at neighbor (21,20) — different polarity — should fail
    assert!(!stc.filter(&Event::new(21, 20, 202_000, 1)));
}

// =========================================================================
// Phase 2: Rate Statistics integration
// =========================================================================

#[test]
fn test_pipeline_rate_stats_with_filters() {
    let mut stats = EventRateStats::new(100_000); // 100ms window
    let polarity = PolarityFilter::new(PolarityMode::OnOnly);

    let mut rate_computed = false;
    for i in 0..200 {
        let ev = Event::new((i % 128) as u16, 0, i as i64 * 1000, 1);
        if polarity.filter(&ev) && stats.record(&ev).is_some() {
            rate_computed = true;
        }
    }

    assert!(rate_computed);
    assert!(stats.total_count() > 0);
    assert!(stats.last_rate() > 0.0);
}

// =========================================================================
// Phase 2: Mask Filter integration
// =========================================================================

#[test]
fn test_pipeline_mask_with_hot_pixels() {
    let mut mask = MaskFilter::new(128, 128);
    let mut acc = Accumulator::new(128, 128);

    // Mask out known noisy corner
    mask.set_rect(0, 0, 5, 5, false);

    let events = vec![
        Event::new(3, 3, 100, 1),   // masked
        Event::new(10, 10, 200, 1), // active
        Event::new(0, 0, 300, 1),   // masked
        Event::new(64, 64, 400, 1), // active
    ];

    let mut passed = 0u32;
    for ev in &events {
        if mask.filter(ev) {
            acc.accumulate(ev);
            passed += 1;
        }
    }

    assert_eq!(passed, 2);
}

// =========================================================================
// Phase 2: Combined extended pipeline
// =========================================================================

#[test]
fn test_phase2_full_pipeline() {
    let mut temporal = TemporalFilter::new(128, 128, 5000);
    let mut refractory = RefractoryFilter::new(128, 128, 500);
    let mut stc = StcFilter::new(128, 128, 5000);
    let mut mask = MaskFilter::new(128, 128);
    let mut ts = TimeSurface::new(128, 128, 100_000.0);
    let mut stats = EventRateStats::new(100_000);

    // Mask out borders
    mask.set_rect(0, 0, 127, 0, false);

    // Process a sequence of events
    temporal.filter(&Event::new(30, 30, 100_000, 1));
    let ev = Event::new(31, 30, 101_000, 1);

    let passes = temporal.filter(&ev)
        && refractory.filter(&ev)
        && mask.filter(&ev);

    assert!(passes);
    ts.update(&ev);
    stats.record(&ev);
    // Also try STC — needs same-polarity neighbor
    stc.filter(&Event::new(30, 30, 100_000, 1));
    assert!(stc.filter(&Event::new(31, 30, 101_000, 1)));
}

// =========================================================================
// Phase 2: FFI null safety tests
// =========================================================================

#[test]
fn test_ffi_time_surface_null_safety() {
    unsafe {
        ffi::edvs_time_surface_update(std::ptr::null_mut(), &Event::new(0, 0, 100, 1));
        let mut len: usize = 0;
        let ptr = ffi::edvs_time_surface_get_frame(std::ptr::null(), 100, &mut len);
        assert!(ptr.is_null());
        ffi::edvs_time_surface_reset(std::ptr::null_mut());
        ffi::edvs_time_surface_destroy(std::ptr::null_mut());
    }
}

#[test]
fn test_ffi_time_surface_round_trip() {
    unsafe {
        let ts = ffi::edvs_time_surface_create(4, 4, 100_000.0);
        assert!(!ts.is_null());

        ffi::edvs_time_surface_update(ts, &Event::new(1, 1, 100_000, 1));

        let mut len: usize = 0;
        let ptr = ffi::edvs_time_surface_get_frame(ts, 100_000, &mut len);
        assert!(!ptr.is_null());
        assert_eq!(len, 16);

        let frame = std::slice::from_raw_parts(ptr, len);
        assert_eq!(frame[5], 255); // pixel (1,1) at current time = max

        ffi::edvs_time_surface_free_frame(ptr, len);
        ffi::edvs_time_surface_destroy(ts);
    }
}

#[test]
fn test_ffi_slicer_null_safety() {
    unsafe {
        assert!(!ffi::edvs_slicer_process(std::ptr::null_mut(), &Event::new(0, 0, 100, 1)));
        ffi::edvs_slicer_destroy(std::ptr::null_mut());
    }
}

#[test]
fn test_ffi_slicer_round_trip() {
    unsafe {
        let s = ffi::edvs_slicer_create(0, 3); // ByCount, threshold=3
        assert!(!s.is_null());

        let ev = Event::new(0, 0, 100, 1);
        assert!(!ffi::edvs_slicer_process(s, &ev));
        assert!(!ffi::edvs_slicer_process(s, &ev));
        assert!(ffi::edvs_slicer_process(s, &ev));

        ffi::edvs_slicer_destroy(s);
    }
}

#[test]
fn test_ffi_anti_flicker_null_safety() {
    unsafe {
        assert!(!ffi::edvs_anti_flicker_filter_process(
            std::ptr::null_mut(),
            &Event::new(0, 0, 100, 1)
        ));
        ffi::edvs_anti_flicker_filter_destroy(std::ptr::null_mut());
    }
}

#[test]
fn test_ffi_stc_null_safety() {
    unsafe {
        assert!(!ffi::edvs_stc_filter_process(
            std::ptr::null_mut(),
            &Event::new(0, 0, 100, 1)
        ));
        ffi::edvs_stc_filter_destroy(std::ptr::null_mut());
    }
}

#[test]
fn test_ffi_rate_stats_null_safety() {
    unsafe {
        let mut rate: f64 = 0.0;
        assert!(!ffi::edvs_rate_stats_record(
            std::ptr::null_mut(),
            &Event::new(0, 0, 100, 1),
            &mut rate
        ));
        assert_eq!(ffi::edvs_rate_stats_last_rate(std::ptr::null()), 0.0);
        assert_eq!(ffi::edvs_rate_stats_peak_rate(std::ptr::null()), 0.0);
        assert_eq!(ffi::edvs_rate_stats_total_count(std::ptr::null()), 0);
        ffi::edvs_rate_stats_destroy(std::ptr::null_mut());
    }
}

#[test]
fn test_ffi_rate_stats_round_trip() {
    unsafe {
        let s = ffi::edvs_rate_stats_create(100_000);
        assert!(!s.is_null());

        let mut rate: f64 = 0.0;
        for i in 0..200 {
            ffi::edvs_rate_stats_record(s, &Event::new(0, 0, i * 1000, 1), &mut rate);
        }

        assert!(ffi::edvs_rate_stats_total_count(s) == 200);
        ffi::edvs_rate_stats_destroy(s);
    }
}

#[test]
fn test_ffi_mask_null_safety() {
    unsafe {
        assert!(!ffi::edvs_mask_filter_process(
            std::ptr::null(),
            &Event::new(0, 0, 100, 1)
        ));
        ffi::edvs_mask_filter_set_pixel(std::ptr::null_mut(), 0, 0, false);
        ffi::edvs_mask_filter_destroy(std::ptr::null_mut());
    }
}

#[test]
fn test_ffi_mask_round_trip() {
    unsafe {
        let m = ffi::edvs_mask_filter_create(4, 4);
        assert!(!m.is_null());

        // All enabled by default
        assert!(ffi::edvs_mask_filter_process(m, &Event::new(1, 1, 100, 1)));

        // Disable pixel
        ffi::edvs_mask_filter_set_pixel(m, 1, 1, false);
        assert!(!ffi::edvs_mask_filter_process(m, &Event::new(1, 1, 100, 1)));

        ffi::edvs_mask_filter_destroy(m);
    }
}

// =========================================================================
// Phase 3: Speed-Invariant Time Surface integration
// =========================================================================

#[test]
fn test_pipeline_sits_rank_ordering() {
    let mut temporal = TemporalFilter::new(16, 16, 5000);
    let mut sits = SpeedInvariantTimeSurface::new(16, 16, 1);

    // Create correlated events that pass temporal filter
    temporal.filter(&Event::new(7, 7, 100_000, 1));
    let events = vec![
        Event::new(7, 8, 101_000, 1),
        Event::new(8, 7, 102_000, 1),
        Event::new(8, 8, 103_000, 1),
    ];

    for ev in &events {
        if temporal.filter(ev) {
            sits.update(ev);
        }
    }

    let ctx = sits.get_context(8, 8);
    assert_eq!(ctx.len(), 9);
    // The most recent event at (8,8) should have rank 1.0
    assert!((ctx[4] - 1.0).abs() < 0.01);
}

// =========================================================================
// Phase 3: Voxel Grid integration
// =========================================================================

#[test]
fn test_pipeline_voxel_grid_with_filters() {
    let polarity = PolarityFilter::new(PolarityMode::Both);
    let mut vg = VoxelGrid::new(16, 16, 5);
    vg.set_time_window(0, 1_000_000);

    for i in 0..50 {
        let ev = Event::new((i % 16) as u16, (i / 16) as u16, i as i64 * 20_000, 1);
        if polarity.filter(&ev) {
            vg.add_event(&ev);
        }
    }

    // Grid should have non-zero values
    assert!(vg.get_grid().iter().any(|&v| v != 0.0));
}

// =========================================================================
// Phase 3: Corner Detector integration
// =========================================================================

#[test]
fn test_pipeline_corner_with_denoise() {
    let mut temporal = TemporalFilter::new(16, 16, 5000);
    let mut corner = HarrisCornerDetector::new(16, 16, 0.04, 0.0);

    // Seed the time surface with a corner-like pattern
    // Top-left quadrant at t=100k, top-right at t=200k
    for y in 0..8u16 {
        for x in 0..8u16 {
            temporal.filter(&Event::new(x, y, 100_000, 1));
            corner.process(&Event::new(x, y, 100_000, 1));
        }
        for x in 8..16u16 {
            temporal.filter(&Event::new(x, y, 200_000, 1));
            corner.process(&Event::new(x, y, 200_000, 1));
        }
    }
    for y in 8..16u16 {
        for x in 0..8u16 {
            temporal.filter(&Event::new(x, y, 300_000, 1));
            corner.process(&Event::new(x, y, 300_000, 1));
        }
        for x in 8..16u16 {
            temporal.filter(&Event::new(x, y, 400_000, 1));
            corner.process(&Event::new(x, y, 400_000, 1));
        }
    }

    // The junction at (8,8) should have corner response
    let response = corner.query_response(8, 8);
    assert!(response > 0.0, "expected corner at junction, got {}", response);
}

// =========================================================================
// Phase 3: Frequency Estimator integration
// =========================================================================

#[test]
fn test_pipeline_frequency_with_mask() {
    let mut mask = MaskFilter::new(8, 8);
    let mut freq = FrequencyEstimator::new(8, 8, 8);

    // Mask out a corner
    mask.set_rect(0, 0, 1, 1, false);

    // 200 Hz signal at pixel (4,4) = 5000 us period
    for i in 0..8 {
        let ev = Event::new(4, 4, i * 5000, 1);
        if mask.filter(&ev) {
            freq.record(&ev);
        }
    }

    let f = freq.estimate_frequency(4, 4).unwrap();
    assert!(
        (f - 200.0).abs() < 20.0,
        "expected ~200 Hz, got {} Hz",
        f
    );

    // Masked pixel should have no data
    let ev_masked = Event::new(0, 0, 100_000, 1);
    if mask.filter(&ev_masked) {
        freq.record(&ev_masked);
    }
    assert!(freq.estimate_frequency(0, 0).is_none());
}

// =========================================================================
// Phase 3: Optical Flow integration
// =========================================================================

#[test]
fn test_pipeline_optical_flow_horizontal_motion() {
    let mut of = OpticalFlowEstimator::new(32, 32, 2);

    // Populate a horizontal motion pattern: t increases with x
    for y in 10..22u16 {
        for x in 10..22u16 {
            of.process(&Event::new(x, y, x as i64 * 1000, 1));
        }
    }

    // Query flow at center of the pattern
    let flow = of.process(&Event::new(16, 16, 16000, 1));
    assert!(flow.is_some(), "expected flow to be estimated");
    if let Some(f) = flow {
        // Dominant horizontal motion expected
        assert!(
            f.vx.abs() > 50.0,
            "expected significant horizontal flow, vx={}",
            f.vx
        );
    }
}

// =========================================================================
// Phase 3: Full combined pipeline
// =========================================================================

#[test]
fn test_phase3_full_pipeline() {
    let mut temporal = TemporalFilter::new(32, 32, 5000);
    let mut refractory = RefractoryFilter::new(32, 32, 500);
    let polarity = PolarityFilter::new(PolarityMode::Both);
    let mut sits = SpeedInvariantTimeSurface::new(32, 32, 2);
    let mut corner = HarrisCornerDetector::new(32, 32, 0.04, 0.0);
    let mut freq = FrequencyEstimator::new(32, 32, 8);
    let mut stats = EventRateStats::new(100_000);

    let mut passed = 0u32;
    for i in 0..200 {
        let x = (10 + i % 12) as u16;
        let y = (10 + i / 12) as u16;
        let ts = (i as i64 + 1) * 1000;
        let ev = Event::new(x, y, ts, 1);

        if temporal.filter(&ev) && refractory.filter(&ev) && polarity.filter(&ev) {
            sits.update(&ev);
            corner.process(&ev);
            freq.record(&ev);
            stats.record(&ev);
            passed += 1;
        }
    }

    assert!(passed > 0, "some events should pass the filter pipeline");
    assert!(stats.total_count() > 0);
}

// =========================================================================
// Phase 3: FFI null safety tests
// =========================================================================

#[test]
fn test_ffi_sits_null_safety() {
    unsafe {
        ffi::edvs_sits_update(std::ptr::null_mut(), &Event::new(0, 0, 100, 1));
        ffi::edvs_sits_reset(std::ptr::null_mut());
        ffi::edvs_sits_destroy(std::ptr::null_mut());
    }
}

#[test]
fn test_ffi_sits_round_trip() {
    unsafe {
        let s = ffi::edvs_sits_create(8, 8, 1);
        assert!(!s.is_null());

        ffi::edvs_sits_update(s, &Event::new(4, 4, 100_000, 1));
        ffi::edvs_sits_destroy(s);
    }
}

#[test]
fn test_ffi_voxel_grid_null_safety() {
    unsafe {
        ffi::edvs_voxel_grid_add_event(std::ptr::null_mut(), &Event::new(0, 0, 100, 1));
        let mut len: usize = 0;
        let ptr = ffi::edvs_voxel_grid_get_grid(std::ptr::null(), &mut len);
        assert!(ptr.is_null());
        ffi::edvs_voxel_grid_reset(std::ptr::null_mut());
        ffi::edvs_voxel_grid_destroy(std::ptr::null_mut());
    }
}

#[test]
fn test_ffi_voxel_grid_round_trip() {
    unsafe {
        let vg = ffi::edvs_voxel_grid_create(4, 4, 5);
        assert!(!vg.is_null());

        ffi::edvs_voxel_grid_set_time_window(vg, 0, 1_000_000);
        ffi::edvs_voxel_grid_add_event(vg, &Event::new(1, 1, 500_000, 1));

        let mut len: usize = 0;
        let ptr = ffi::edvs_voxel_grid_get_grid(vg, &mut len);
        assert!(!ptr.is_null());
        assert_eq!(len, 4 * 4 * 5);

        ffi::edvs_voxel_grid_destroy(vg);
    }
}

#[test]
fn test_ffi_corner_detector_null_safety() {
    unsafe {
        let mut response: f64 = 0.0;
        assert!(!ffi::edvs_corner_detector_process(
            std::ptr::null_mut(),
            &Event::new(0, 0, 100, 1),
            &mut response
        ));
        ffi::edvs_corner_detector_reset(std::ptr::null_mut());
        ffi::edvs_corner_detector_destroy(std::ptr::null_mut());
    }
}

#[test]
fn test_ffi_corner_detector_round_trip() {
    unsafe {
        let d = ffi::edvs_corner_detector_create(16, 16, 0.04, 0.0);
        assert!(!d.is_null());

        let mut response: f64 = 0.0;
        ffi::edvs_corner_detector_process(d, &Event::new(4, 4, 100_000, 1), &mut response);

        ffi::edvs_corner_detector_destroy(d);
    }
}

#[test]
fn test_ffi_frequency_estimator_null_safety() {
    unsafe {
        ffi::edvs_frequency_estimator_record(std::ptr::null_mut(), &Event::new(0, 0, 100, 1));
        let f = ffi::edvs_frequency_estimator_estimate(std::ptr::null(), 0, 0);
        assert!((f - (-1.0)).abs() < 0.01);
        ffi::edvs_frequency_estimator_reset(std::ptr::null_mut());
        ffi::edvs_frequency_estimator_destroy(std::ptr::null_mut());
    }
}

#[test]
fn test_ffi_frequency_estimator_round_trip() {
    unsafe {
        let fe = ffi::edvs_frequency_estimator_create(4, 4, 8);
        assert!(!fe.is_null());

        // 100 Hz = 10000 us period
        for i in 0..8 {
            ffi::edvs_frequency_estimator_record(fe, &Event::new(0, 0, i * 10_000, 1));
        }

        let freq = ffi::edvs_frequency_estimator_estimate(fe, 0, 0);
        assert!(freq > 0.0, "expected positive frequency, got {}", freq);

        ffi::edvs_frequency_estimator_destroy(fe);
    }
}

#[test]
fn test_ffi_optical_flow_null_safety() {
    unsafe {
        let mut vx: f64 = 0.0;
        let mut vy: f64 = 0.0;
        assert!(!ffi::edvs_optical_flow_process(
            std::ptr::null_mut(),
            &Event::new(0, 0, 100, 1),
            &mut vx,
            &mut vy
        ));
        ffi::edvs_optical_flow_reset(std::ptr::null_mut());
        ffi::edvs_optical_flow_destroy(std::ptr::null_mut());
    }
}

#[test]
fn test_ffi_optical_flow_round_trip() {
    unsafe {
        let of = ffi::edvs_optical_flow_create(16, 16, 2);
        assert!(!of.is_null());

        let mut vx: f64 = 0.0;
        let mut vy: f64 = 0.0;
        ffi::edvs_optical_flow_process(of, &Event::new(8, 8, 100_000, 1), &mut vx, &mut vy);

        ffi::edvs_optical_flow_destroy(of);
    }
}

#[test]
fn test_ffi_sits_invalid_params() {
    // Zero radius
    let s = ffi::edvs_sits_create(4, 4, 0);
    assert!(s.is_null());
}

#[test]
fn test_ffi_voxel_grid_invalid_params() {
    // num_bins < 2
    let vg = ffi::edvs_voxel_grid_create(4, 4, 1);
    assert!(vg.is_null());
}

#[test]
fn test_ffi_corner_detector_invalid_params() {
    // Negative harris_k
    let d = ffi::edvs_corner_detector_create(4, 4, -1.0, 0.0);
    assert!(d.is_null());
}

#[test]
fn test_ffi_frequency_estimator_invalid_params() {
    // history_len < 3
    let fe = ffi::edvs_frequency_estimator_create(4, 4, 2);
    assert!(fe.is_null());
}

#[test]
fn test_ffi_optical_flow_invalid_params() {
    // Zero radius
    let of = ffi::edvs_optical_flow_create(4, 4, 0);
    assert!(of.is_null());
}

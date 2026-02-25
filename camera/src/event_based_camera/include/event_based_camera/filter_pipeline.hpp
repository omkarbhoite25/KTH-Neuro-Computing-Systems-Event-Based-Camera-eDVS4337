#ifndef FILTER_PIPELINE_HPP
#define FILTER_PIPELINE_HPP

#include <cstdint>

#include "event_based_camera/event_types.hpp"

// Forward-declare Rust FFI types (opaque pointers)
extern "C" {
    struct TemporalFilter;
    struct HotPixelFilter;

    TemporalFilter* edvs_temporal_filter_create(uint32_t width, uint32_t height, int64_t threshold_us);
    bool edvs_temporal_filter_process(TemporalFilter* filter, const edvs::Event* event);
    void edvs_temporal_filter_destroy(TemporalFilter* filter);

    HotPixelFilter* edvs_hot_pixel_filter_create(uint32_t width, uint32_t height,
                                                  int64_t window_us, uint32_t max_rate);
    bool edvs_hot_pixel_filter_process(HotPixelFilter* filter, const edvs::Event* event);
    void edvs_hot_pixel_filter_destroy(HotPixelFilter* filter);
}

/// RAII wrapper for opaque Rust FFI filter pointers.
/// Calls the provided destroy function on destruction.
template <typename T, void (*DestroyFn)(T*)>
class RustFilterPtr {
public:
    RustFilterPtr() : ptr_(nullptr) {}
    explicit RustFilterPtr(T* p) : ptr_(p) {}
    ~RustFilterPtr() { reset(); }

    // Non-copyable
    RustFilterPtr(const RustFilterPtr&) = delete;
    RustFilterPtr& operator=(const RustFilterPtr&) = delete;

    // Movable
    RustFilterPtr(RustFilterPtr&& other) noexcept : ptr_(other.ptr_) { other.ptr_ = nullptr; }
    RustFilterPtr& operator=(RustFilterPtr&& other) noexcept {
        if (this != &other) {
            reset();
            ptr_ = other.ptr_;
            other.ptr_ = nullptr;
        }
        return *this;
    }

    T* get() const { return ptr_; }
    explicit operator bool() const { return ptr_ != nullptr; }

    void reset(T* p = nullptr) {
        if (ptr_) {
            DestroyFn(ptr_);
        }
        ptr_ = p;
    }

private:
    T* ptr_;
};

using TemporalFilterPtr = RustFilterPtr<TemporalFilter, edvs_temporal_filter_destroy>;
using HotPixelFilterPtr = RustFilterPtr<HotPixelFilter, edvs_hot_pixel_filter_destroy>;

/// Composable event filter pipeline using Rust FFI filters.
///
/// Manages the lifecycle of temporal denoising and hot pixel filters.
/// Events are processed through enabled filters in order:
///   temporal denoise → hot pixel rejection.
class FilterPipeline {
public:
    /// Configuration for the filter pipeline.
    struct Config {
        bool enable_denoising = true;
        int64_t denoise_threshold_us = 5000;
        bool enable_hot_pixel = true;
        int64_t hot_pixel_window_us = 1000000;
        uint32_t hot_pixel_max_rate = 1000;
    };

    FilterPipeline() = default;
    ~FilterPipeline() = default;

    // Non-copyable, movable
    FilterPipeline(const FilterPipeline&) = delete;
    FilterPipeline& operator=(const FilterPipeline&) = delete;
    FilterPipeline(FilterPipeline&&) = default;
    FilterPipeline& operator=(FilterPipeline&&) = default;

    /// Initialize filters for the given sensor dimensions and config.
    /// Throws std::runtime_error if filter creation fails.
    void init(uint16_t width, uint16_t height, const Config& config);

    /// Process a single event through all enabled filters.
    /// Returns true if the event passes all filters, false if rejected.
    bool process(const edvs::Event* event);

    /// Release all filter resources.
    void reset();

    bool hasDenoising() const { return static_cast<bool>(temporal_filter_); }
    bool hasHotPixel() const { return static_cast<bool>(hot_pixel_filter_); }

private:
    TemporalFilterPtr temporal_filter_;
    HotPixelFilterPtr hot_pixel_filter_;
};

#endif  // FILTER_PIPELINE_HPP
